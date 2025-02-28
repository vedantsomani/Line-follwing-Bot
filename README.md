# Line Following Robot with PID Control, Obstacle Avoidance, and Bluetooth Control
This project is a line-following robot built using an Arduino with PID control, obstacle avoidance using an ultrasonic sensor, and Bluetooth control. The robot uses 4 motors, 2 motor drivers, and is capable of remote operation via Bluetooth (HC-05). Additionally, 4 LEDs are used to visually indicate the status of the IR sensors.

Features
PID Control: Smooth line-following with proportional, integral, and derivative control.
4 Motors: The robot uses 4 motors for movement (front-left, front-right, back-left, back-right).
4 IR Sensors: Two IR sensors for the front and two for the back, allowing precise tracking of the line.
4 LEDs: Each corresponding to the 4 IR sensors, to visually indicate their state (active or inactive).
2 Motor Drivers (L298N): Control the motors for moving the robot forward, backward, and turning.
Bluetooth Control (HC-05 or HC-06): Remotely control the robot using Bluetooth commands.
Obstacle Avoidance: Uses an Ultrasonic Sensor to detect obstacles and avoid them while following the line.
12V LiPo Battery: Provides the necessary power for the motors and components.
Components Needed
Arduino (Uno, Mega, etc.)
2 x L298N Motor Driver
4 x DC Motors (2 front motors, 2 back motors)
4 x IR Sensors (Analog or Digital)
4 x LEDs (to indicate sensor states)
Bluetooth Module (HC-05 or HC-06)
Ultrasonic Sensor (HC-SR04)
12V LiPo Battery (or equivalent)
Power Supply for Arduino (e.g., 5V step-down regulator)
Wires and Connectors
Wiring Diagram
IR Sensors:

Left Front IR Sensor → A0
Right Front IR Sensor → A1
Left Back IR Sensor → A2
Right Back IR Sensor → A3
LED Indicators (for IR Sensors):

Left Front LED → Pin 8
Right Front LED → Pin 7
Left Back LED → Pin 6
Right Back LED → Pin 5
Motor Driver (L298N):

Front Motor Driver (1st L298N):
IN1 → Pin 4
IN2 → Pin 3
ENA → Pin 9 (PWM)
ENB → Pin 10 (PWM)
Back Motor Driver (2nd L298N):
IN3 → Pin 11
IN4 → Pin 12
ENA → Pin 2 (PWM)
ENB → Pin 13 (PWM)
Bluetooth Module (HC-05):

TX → Pin 14
RX → Pin 15
Ultrasonic Sensor (HC-SR04):

Trig → Pin 18
Echo → Pin 19
VCC → 5V
GND → GND
Power Supply:

12V LiPo Battery to the motor drivers (for motors)
5V step-down regulator to supply power to Arduino and Bluetooth module
GND to GND (common ground for the entire circuit)
Code Explanation
PID Control
The robot uses a simple PID controller to adjust its movement based on the error calculated from the IR sensors. The error represents the difference in sensor readings, indicating whether the robot needs to move left or right to stay on the line.

The PID control adjusts the robot’s movement by:

Proportional (P): Based on the current error.
Integral (I): Based on the sum of all past errors.
Derivative (D): Based on the rate of change of the error.
Motor Control
The robot uses 2 motor drivers (L298N) to control 4 motors (front-left, front-right, back-left, back-right). Each motor driver controls two motors: one driver for the front motors and one driver for the back motors. The motors are controlled based on the output from the PID controller.

Bluetooth Control
You can control the robot via Bluetooth commands (e.g., 'F' for forward, 'B' for backward, 'L' for left, 'R' for right, 'S' for stop). The robot listens for commands from a Bluetooth app or custom controller connected via the HC-05 Bluetooth module.

IR Sensors and LEDs
The 4 IR sensors are used to detect the line:

Left Front IR and Right Front IR are used to detect if the robot is off-track.
Left Back IR and Right Back IR provide additional feedback for fine-tuning movement.
Each IR sensor has an associated LED that lights up when the sensor detects the line. These LEDs visually indicate which sensors are active or inactive.

Ultrasonic Sensor (HC-SR04)
The ultrasonic sensor is used to detect obstacles in the robot's path. The robot will stop or turn if an obstacle is detected within a certain range.

Setup Instructions
Connect the components to the Arduino according to the wiring diagram.
Upload the code to the Arduino using the Arduino IDE.
Power the robot using the 12V LiPo battery connected to the motor drivers (L298N).
Use a 5V step-down regulator to provide power to the Arduino and Bluetooth module.
Pair the Bluetooth module (HC-05/HC-06) with your mobile device for Bluetooth control.
Test the robot on a line-following track and observe how it follows the line based on the IR sensor inputs, avoids obstacles using the ultrasonic sensor, and responds to Bluetooth commands.
PID Tuning
The PID constants (Kp, Ki, Kd) are critical for achieving smooth line-following behavior. You may need to tune these values based on your robot's performance:

Kp: Proportional gain (affects how aggressively the robot corrects its position)
Ki: Integral gain (affects long-term error correction)
Kd: Derivative gain (affects the reaction to changes in error)
To tune the PID constants:

Start with Kp = 1.0, Ki = 0.0, and Kd = 0.0.
Increase Kp to make the robot more responsive.
Add Ki to reduce steady-state errors.
Adjust Kd to reduce oscillations.
Troubleshooting
Robot not following the line: Check the calibration of the IR sensors. Ensure they are aligned properly to detect the line.
Inconsistent movement: Adjust the PID constants for smoother control.
Bluetooth not working: Ensure the HC-05 is correctly paired with your mobile device, and the baud rate is set to 9600.
Obstacle avoidance not working: Ensure the ultrasonic sensor is wired correctly, and check its range.
Future Improvements
Add more sensors for better line detection and more precise control.
Implement a better PID algorithm for more precise control.
Add manual control options through Bluetooth for speed and direction adjustments.
Implement advanced obstacle avoidance algorithms, such as following the wall or turning around obstacles.
