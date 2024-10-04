# Libraries and Constants
Included Libraries:

Wire.h: This library is essential for I2C communication. I use it to communicate with the BNO055 sensor, which provides orientation data.
Adafruit_Sensor.h and Adafruit_BNO055.h: These libraries are specifically for the Adafruit BNO055 sensor. They allow me to retrieve orientation data (e.g., heading) from the sensor easily.
Servo.h: This library is used to control the servo motor for steering the vehicle.
# Pin Assignments:

I define several constants to represent the pin numbers for various sensors and components:
Ultrasonic Sensors:
rt (right trigger pin), re (right echo pin)
lt (left trigger pin), le (left echo pin)
ft (front trigger pin), fe (front echo pin)
Servo and Motor Pins:
SERVO_PIN: The pin connected to the steering servo.
mf, mb, me, ms: Pins for controlling the motors (forward, backward, enable, standby).
Servo and Motor Settings:

I set constants for servo angles:
STRAIGHT_ANGLE (82°): Neutral position for straight movement.
MAX_LEFT (160°) and MAX_RIGHT (20°): Maximum angles for left and right turns.
# PID Control Variables
I implement a PID controller to manage steering:
Kp, Ki, Kd: Constants for the PID control, which determine how the steering adjustments are made based on the error.
error: The difference between the target heading and the current heading.
integral: Accumulates past errors to help eliminate steady-state error.
derivative: The rate of change of the error, which helps to predict future behavior.
steeringAdjustment: The calculated adjustment to the steering based on PID output.
newServoPosition: The adjusted position for the servo after applying PID control.
Sensor Initialization and Setup
Setup Function:
In the setup() function, I initialize all the sensors and components:
Pin Modes: I set the appropriate pin modes for outputs (for the servo and motors) and inputs (for the sensors).
Serial Communication: I initialize serial communication for debugging purposes.
BNO055 Initialization: I check if the BNO055 sensor is properly connected. If not, the program halts.
Servo Setup: I attach the servo to its designated pin and set it to the straight angle.
Motor Initialization: I set the motor pins, enabling the motors by bringing the standby pin low and setting the enable pin to the desired speed.
# Main Loop Logic
Loop Function:

The loop() function contains the core logic for vehicle operation:
Forward Movement: The vehicle starts moving forward.
Distance Measurement: I call the distCalc() function to get the distance from the front ultrasonic sensor.
Wall Detection: If the distance measured is below a certain threshold (forntthreshold):
I measure the distances from the right and left sensors.
Based on the readings, I determine the turn direction:
If the right distance is sufficient, I set the mode to 'r' (turn right).
If the left distance is sufficient, I set the mode to 'l' (turn left).
Turning Logic:
If the front distance is below the threshold and I need to turn (left or right):
I check if enough time has passed since the last turn (to avoid rapid turning).
If conditions are met, I call the respective turning function (turnleft() or turnright()).
I then revert to normal forward speed.
PID Control During Forward Movement:

In the forward() function:
I retrieve the current heading from the BNO055 sensor.
I calculate the error using the calculateAngleError() function to determine how far off the current heading is from the target.
I update the PID variables (integral, derivative) to calculate the steering adjustment.
I constrain the steeringAdjustment to prevent excessive adjustments.
Finally, I calculate the new servo position and write it to the servo.
Turning Functions
Turning Functions:
In turnleft() and turnright(), I:
Set the dontSense flag to true, preventing further sensor readings during the turn.
Update the target heading based on the direction of the turn.
Adjust the servo position to the maximum left or right angle.
Continuously read the current heading until I reach the target heading, ensuring the turn is accurate.
Increment the turns counter to track how many turns the vehicle has completed.
Distance Calculation Functions
Distance Calculation:

The distCalc() function uses ultrasonic sensors to measure distance:
It triggers the ultrasonic sensor and measures the echo time.
The time is converted to distance (in cm) and returns the value.
If the distance is out of reasonable bounds (greater than 150 cm or less than 0), it sets a default value of 150 cm.
Averaging Function (commented out):

The distCalcAVG() function (not currently used) would average multiple distance readings for better reliability. It runs a loop to take several measurements and calculates the average distance.

![Icon](https://github.com/tecnoplasma/2EZ/blob/77ed19f3b2dcfc9a7587c0bad67232a2a57ee0c0/flow%20chart%20for%20first%20round.jpg)
