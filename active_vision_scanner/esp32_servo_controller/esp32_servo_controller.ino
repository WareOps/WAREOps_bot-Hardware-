/*
 * =============================================================================
 * ESP32 Dev Kit V1 — PCA9685 Servo Controller (WiFi HTTP Server)
 * Active Vision Rack Scanning System
 * =============================================================================
 *
 * Network:
 *   - Connects to mobile hotspot WiFi
 *   - Runs an HTTP server on port 80
 *   - Laptop sends JSON commands via HTTP POST to http://<ESP32_IP>/cmd
 *   - ESP32 responds with JSON status
 *
 * Hardware:
 *   - ESP32 Dev Kit V1
 *   - PCA9685 16-channel PWM driver (I2C: SDA=21, SCL=22)
 *   - Channel 1: Yaw servo (SG90 or equivalent)
 *   - Channel 2: Pitch servo (SG90 or equivalent)
 *
 * Servo Movement:
 *   - Trajectory planning: moves in +1° / -1° increments via a loop
 *   - Never jumps directly to target angle
 *   - Configurable step delay for smooth motion
 *
 * JSON Command Format (HTTP POST to /cmd):
 *   {"cmd":"MOVE","pitch":90,"yaw":0,"speed":20}
 *   {"cmd":"HOME"}
 *   {"cmd":"STOP"}
 *   {"cmd":"START_SCAN"}
 *   {"cmd":"STATUS"}
 *
 * Author: Abhinav
 * Date: 2026-07-25
 * =============================================================================
 */

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// =============================================================================
// WiFi Configuration — UPDATE THESE FOR YOUR MOBILE HOTSPOT
// =============================================================================
const char *WIFI_SSID = "wolf2";        // <-- Change this
const char *WIFI_PASSWORD = "wolf123@"; // <-- Change this

// =============================================================================
// Hardware Configuration
// =============================================================================

// PCA9685
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50 // 50 Hz for standard servos

// Channel assignments
#define YAW_CHANNEL 1   // PCA9685 channel for yaw servo
#define PITCH_CHANNEL 2 // PCA9685 channel for pitch servo

// SG90 Servo pulse limits (in microseconds)
// SG90 datasheet: 500µs (0°) to 2400µs (180°), operating voltage 4.8-6V
#define SERVO_MIN_US 500    // SG90 minimum pulse width (0°)
#define SERVO_MAX_US 2400   // SG90 maximum pulse width (180°)
#define SERVO_MIN_ANGLE 0   // Minimum angle (degrees)
#define SERVO_MAX_ANGLE 180 // Maximum angle (degrees)

// Default / Home / Parked positions
#define HOME_YAW 90           // Parked yaw (degrees)
#define HOME_PITCH 90         // Parked pitch (degrees)
#define DEFAULT_SCAN_YAW 0    // Default scanning yaw
#define DEFAULT_SCAN_PITCH 90 // Default scanning pitch

// Trajectory planning
#define STEP_DEGREES 1           // Move 1° at a time
#define DEFAULT_STEP_DELAY_MS 15 // Delay between each 1° step (ms)
#define MIN_STEP_DELAY_MS 5      // Fastest speed
#define MAX_STEP_DELAY_MS 50     // Slowest speed

// I2C Pins (ESP32 defaults)
#define I2C_SDA 21
#define I2C_SCL 22

// HTTP Server port
#define HTTP_PORT 80

// =============================================================================
// Global Objects
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
WebServer server(HTTP_PORT);

// Current servo positions (in degrees)
int current_yaw = HOME_YAW;
int current_pitch = HOME_PITCH;

// Target servo positions
int target_yaw = HOME_YAW;
int target_pitch = HOME_PITCH;
unsigned long last_step_time = 0;

// Movement control
int step_delay_ms = DEFAULT_STEP_DELAY_MS;
bool is_moving = false;
bool system_active = true;

// =============================================================================
// Servo Helper Functions
// =============================================================================

/**
 * Convert angle (0-180°) to PCA9685 PWM tick count.
 * PCA9685 has 4096 ticks per cycle at 50 Hz (20 ms period).
 */
uint16_t angleToPWM(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  long pulse_us =
      map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
  uint16_t ticks = (uint16_t)((pulse_us * 4096L) / 20000L);
  return ticks;
}

/**
 * Set a servo to a specific angle immediately (no trajectory).
 */
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  uint16_t pwm_val = angleToPWM(angle);
  pwm.setPWM(channel, 0, pwm_val);
}

/**
 * Convert speed parameter (from JSON) to step delay in milliseconds.
 * Higher speed value = faster movement = lower delay.
 * Speed range: 1 (slowest) to 100 (fastest).
 */
int speedToDelay(int speed) {
  speed = constrain(speed, 1, 100);
  return map(speed, 1, 100, MAX_STEP_DELAY_MS, MIN_STEP_DELAY_MS);
}

// =============================================================================
// Trajectory Planning — Smooth Servo Movement (+1° / -1° increments)
// =============================================================================

/**
 * Set the target positions for both servos.
 * The actual movement is handled non-blockingly in the main loop().
 */
void setTargets(int tgt_yaw, int tgt_pitch, int delay_ms) {
  target_yaw = constrain(tgt_yaw, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  target_pitch = constrain(tgt_pitch, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  step_delay_ms = delay_ms;
  is_moving = true;

  Serial.print("[TARGET] Updated to Yaw: ");
  Serial.print(target_yaw);
  Serial.print("°, Pitch: ");
  Serial.print(target_pitch);
  Serial.print("° (delay=");
  Serial.print(delay_ms);
  Serial.println("ms)");
}

// =============================================================================
// HTTP Request Handlers
// =============================================================================

/**
 * Handle POST /cmd — receives JSON commands from the laptop.
 */
void handleCommand() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Use POST\"}");
    return;
  }

  String body = server.arg("plain");
  Serial.print("[HTTP] Received: ");
  Serial.println(body);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    Serial.print("[ERROR] JSON parse failed: ");
    Serial.println(error.c_str());
    server.send(400, "application/json",
                "{\"status\":\"error\",\"msg\":\"Invalid JSON\"}");
    return;
  }

  const char *cmd = doc["cmd"] | "UNKNOWN";
  String response;

  // ---- MOVE ----
  if (strcmp(cmd, "MOVE") == 0) {
    int tgt_pitch = doc["pitch"] | current_pitch;
    int tgt_yaw = doc["yaw"] | current_yaw;
    int speed = doc["speed"] | 20;
    int dly = speedToDelay(speed);

    Serial.print("[MOVE] pitch=");
    Serial.print(tgt_pitch);
    Serial.print("°, yaw=");
    Serial.print(tgt_yaw);
    Serial.print("°, speed=");
    Serial.println(speed);

    // Send immediate acknowledgment before starting movement
    // (movement happens after response is sent)
    response = "{\"status\":\"ok\",\"cmd\":\"MOVE_ACK\",\"target_yaw\":" +
               String(tgt_yaw) + ",\"target_pitch\":" + String(tgt_pitch) + "}";
    server.send(200, "application/json", response);

    // Now set the new targets for the non-blocking loop
    setTargets(tgt_yaw, tgt_pitch, dly);

    Serial.println("[MOVE] Command accepted.");
    return; // Response already sent
  }

  // ---- HOME ----
  else if (strcmp(cmd, "HOME") == 0) {
    Serial.println("[HOME] Moving to home position...");
    response = "{\"status\":\"ok\",\"cmd\":\"HOME_ACK\"}";
    server.send(200, "application/json", response);

    setTargets(HOME_YAW, HOME_PITCH, DEFAULT_STEP_DELAY_MS);
    return;
  }

  // ---- STOP ----
  else if (strcmp(cmd, "STOP") == 0) {
    Serial.println("[STOP] Stopping all movement.");
    system_active = false;
    response = "{\"status\":\"ok\",\"cmd\":\"STOPPED\"}";
    server.send(200, "application/json", response);
    return;
  }

  // ---- START_SCAN ----
  else if (strcmp(cmd, "START_SCAN") == 0) {
    Serial.println("[START_SCAN] Resuming system.");
    system_active = true;
    response = "{\"status\":\"ok\",\"cmd\":\"SCAN_STARTED\"}";
    server.send(200, "application/json", response);
    return;
  }

  // ---- STATUS ----
  else if (strcmp(cmd, "STATUS") == 0) {
    StaticJsonDocument<256> resp_doc;
    resp_doc["status"] = "ok";
    resp_doc["cmd"] = "STATUS";
    resp_doc["yaw"] = current_yaw;
    resp_doc["pitch"] = current_pitch;
    resp_doc["target_yaw"] = target_yaw;
    resp_doc["target_pitch"] = target_pitch;
    resp_doc["is_moving"] = is_moving;
    resp_doc["active"] = system_active;

    String resp_str;
    serializeJson(resp_doc, resp_str);
    server.send(200, "application/json", resp_str);
    return;
  }

  // ---- UNKNOWN ----
  else {
    Serial.print("[ERROR] Unknown command: ");
    Serial.println(cmd);
    response =
        "{\"status\":\"error\",\"msg\":\"Unknown command: " + String(cmd) +
        "\"}";
    server.send(400, "application/json", response);
  }
}

/**
 * Handle GET / — simple status page.
 */
void handleRoot() {
  String html =
      "<!DOCTYPE html><html><head><title>ESP32 Servo Controller</title>";
  html +=
      "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:monospace;background:#1a1a2e;color:#e0e0e0;"
          "padding:20px;}";
  html += "h1{color:#00d4ff;} "
          ".val{color:#00ff88;font-weight:bold;}</style></head><body>";
  html += "<h1>ESP32 Servo Controller</h1>";
  html += "<p>Yaw:   <span class='val'>" + String(current_yaw) +
          "&deg;</span> (Channel " + String(YAW_CHANNEL) + ")</p>";
  html += "<p>Pitch: <span class='val'>" + String(current_pitch) +
          "&deg;</span> (Channel " + String(PITCH_CHANNEL) + ")</p>";
  html += "<p>Moving: <span class='val'>" + String(is_moving ? "YES" : "NO") +
          "</span></p>";
  html += "<p>Active: <span class='val'>" +
          String(system_active ? "YES" : "NO") + "</span></p>";
  html += "<hr><p>POST JSON commands to <b>/cmd</b></p>";
  html +=
      "<p>Example: "
      "<code>{\"cmd\":\"MOVE\",\"pitch\":90,\"yaw\":0,\"speed\":20}</code></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

/**
 * Handle GET /status — JSON status endpoint.
 */
void handleStatus() {
  StaticJsonDocument<256> doc;
  doc["yaw"] = current_yaw;
  doc["pitch"] = current_pitch;
  doc["is_moving"] = is_moving;
  doc["active"] = system_active;
  doc["ip"] = WiFi.localIP().toString();

  String resp;
  serializeJson(doc, resp);
  server.send(200, "application/json", resp);
}

// =============================================================================
// WiFi Connection
// =============================================================================

void connectToWiFi() {
  Serial.print("[WIFI] Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts > 40) { // 20 second timeout
      Serial.println("\n[WIFI] Connection failed! Restarting...");
      ESP.restart();
    }
  }

  Serial.println();
  Serial.print("[WIFI] Connected! IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("[WIFI] Signal strength: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("==============================================");
  Serial.println("  ESP32 PCA9685 Servo Controller (WiFi)");
  Serial.println("  Active Vision Rack Scanning System");
  Serial.println("==============================================");
  Serial.print("  Yaw Channel   : ");
  Serial.println(YAW_CHANNEL);
  Serial.print("  Pitch Channel : ");
  Serial.println(PITCH_CHANNEL);
  Serial.print("  Home Position : yaw=");
  Serial.print(HOME_YAW);
  Serial.print("°, pitch=");
  Serial.print(HOME_PITCH);
  Serial.println("°");
  Serial.println("==============================================");

  // Initialize I2C and PCA9685
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  Serial.println("[INIT] PCA9685 initialized at 50 Hz.");

  // Move servos to home position on startup
  Serial.println("[INIT] Moving to home position...");
  setServoAngle(YAW_CHANNEL, HOME_YAW);
  setServoAngle(PITCH_CHANNEL, HOME_PITCH);
  current_yaw = HOME_YAW;
  current_pitch = HOME_PITCH;
  delay(500);

  // Connect to WiFi (mobile hotspot)
  connectToWiFi();

  // Setup HTTP routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/cmd", HTTP_POST, handleCommand);
  server.begin();

  Serial.println("[HTTP] Server started on port 80.");
  Serial.print("[HTTP] Send commands to: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/cmd");
  Serial.println("[INIT] Ready!");
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
  // Handle incoming HTTP requests
  server.handleClient();

  // Trajectory movement (non-blocking)
  if (system_active && is_moving) {
    if (millis() - last_step_time >= step_delay_ms) {
      last_step_time = millis();
      bool reached_yaw = (current_yaw == target_yaw);
      bool reached_pitch = (current_pitch == target_pitch);

      if (!reached_yaw) {
        int yaw_dir = (target_yaw > current_yaw) ? 1 : -1;
        current_yaw += yaw_dir;
        setServoAngle(YAW_CHANNEL, current_yaw);
      }
      if (!reached_pitch) {
        int pitch_dir = (target_pitch > current_pitch) ? 1 : -1;
        current_pitch += pitch_dir;
        setServoAngle(PITCH_CHANNEL, current_pitch);
      }

      if (reached_yaw && reached_pitch) {
        is_moving = false;
        Serial.print("[TRAJECTORY] Reached Yaw=");
        Serial.print(current_yaw);
        Serial.print("°, Pitch=");
        Serial.println(current_pitch);
      }
    }
  }

  // Reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Disconnected! Reconnecting...");
    connectToWiFi();
  }

  delay(1);
}
