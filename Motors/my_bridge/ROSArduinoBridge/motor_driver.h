/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

#ifdef CYTRON_MDD10A
  /* The Cytron MDD10A uses PWM + DIR control per channel (2 pins per
     motor) instead of the 4-pin H-bridge style used by the L298.
     This assumes the driver's onboard mode switch is set to
     "PWM/DIR" mode (the factory default), NOT "IN1/IN2" mode.
     Wire M1 = LEFT motor, M2 = RIGHT motor (swap these numbers instead
     of re-wiring the driver if yours is backwards). Both PWM pins must
     be PWM-capable Arduino Nano pins. */
  #define LEFT_MOTOR_DIR   7   // to M1DIR
  #define LEFT_MOTOR_PWM   10  // to M1PWM
  #define RIGHT_MOTOR_DIR  8   // to M2DIR
  #define RIGHT_MOTOR_PWM  9   // to M2PWM

  /* Optional: only needed if you've wired the MDD10A's EN pin to an
     Arduino pin instead of tying it straight to +5V. Uncomment and
     set the pin number if you're using it. */
  //#define MOTOR_DRIVER_ENABLE_PIN 12
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
