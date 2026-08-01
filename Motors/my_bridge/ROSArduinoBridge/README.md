# Arduino Motor Controller

This code turns an Arduino into a motor controller!
It provides a simple serial interface to communicate with a high-level computer (e.g. running ROS), and generates the appropriate PWM signals for a motor driver, to drive two motors.

This is a fork of the original code, with some changes, and removal of the ROS nodes (see [this repo](https://github.com/joshnewans/serial_motor_demo) for an alternative). Check out `README-orig.md` for the original README.

As I only have need for a subset of the functionality, I have no idea what does and doesn't work, beyond what is detailed below.
Feedback/improvements are welcome (though no promises on how quickly I'll respond). I currently only use the **Cytron MDD10A** driver, and the Arduino encoder mode.

TODO
- Finish this README


## Hardware: Cytron MDD10A wiring

This fork is configured for an Arduino Nano + Cytron MDD10A dual motor driver, using the MDD10A's **PWM/DIR** mode (the driver's factory-default mode; make sure the onboard mode switch is set to PWM, not IN1/IN2).

Default pin mapping (edit in `motor_driver.h` if yours differs):

| Signal      | Arduino Nano pin | MDD10A pin |
|-------------|------------------|------------|
| Left DIR    | D7               | M1DIR      |
| Left PWM    | D10              | M1PWM      |
| Right DIR   | D8               | M2DIR      |
| Right PWM   | D9               | M2PWM      |

The MDD10A's `EN` pin can simply be tied to +5V (recommended, saves a pin). If you'd rather drive it from the Arduino, wire it to a spare pin and uncomment/set `MOTOR_DRIVER_ENABLE_PIN` in `motor_driver.h`.

If a motor spins the wrong direction, don't rewire the driver — just flip the `reverse == 0 / 1` logic for that motor in `motor_driver.ino`.

Encoders are still wired directly to the Arduino as before (`ARDUINO_ENC_COUNTER` mode): D2/D3 for left, A4/A5 for right.


## Functionality

The main functionality provided is to receive motor speed requests over a serial connection, and provide encoder feedback.
The original code has provisions for other features - e.g. read/write of digital/analog pins, servo control, but I've never used them.

The main commands to know are

- `e` - Motor responds with current encoder counts for each motor
- `r` - Reset encoder values
- `o <PWM1> <PWM2>` - Set the raw PWM speed of each motor (-255 to 255)
- `m <Spd1> <Spd2>` - Set the closed-loop speed of each motor in *counts per loop* (Default loop rate is 30, so `(counts per sec)/30`
- `p <Kp> <Kd> <Ki> <Ko>` - Update the PID parameters


## Gotchas

Some quick things to note

- There is an auto timeout (default 2s) so you need to keep sending commands for it to keep moving
- PID parameter order is PDI (?)
- Motor speed is in counts per loop
- Default baud rate 57600
- Needs carriage return (CR)
- Make sure serial is enabled (user in dialout group)
- Check out the original readme for more


## TODO (maybe)
- Document PID tuning
- Make the speed input counts per second
- Add/test more driver boards
- Add/test other functionality