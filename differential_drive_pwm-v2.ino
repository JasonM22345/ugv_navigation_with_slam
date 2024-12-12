#include <ros.h>
#include <geometry_msgs/Twist.h>

// Motor control pins
const int LEFT_MOTOR_PIN1 = 2;   // A Pin for left motor forward
const int LEFT_MOTOR_PIN2 = 4;   // B Pin for left motor backward
const int RIGHT_MOTOR_PIN1 = 8;  // Pin for right motor forward
const int RIGHT_MOTOR_PIN2 = 5;  // Pin for right motor backward
const int LEFT_MOTOR_SPEED_PIN = 3;  // PWM pin for left motor speed
const int RIGHT_MOTOR_SPEED_PIN = 6; // PWM pin for right motor speed

// Robot parameters
const float R_w = 0.05;  // Radius of the wheels (in meters)
const float l = 0.3;     // Distance between wheels (in meters)
const int n = 30;        // Gear reduction ratio

// Speed variables
int left_motor_speed = 0;
int right_motor_speed = 0;

// ROS node handle
ros::NodeHandle nh;

// Callback function to handle incoming cmd_vel messages
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  // Extract linear and angular velocities from cmd_vel
  float linear_velocity = cmd_vel_msg.linear.x;   // Forward/backward speed
  float angular_velocity = cmd_vel_msg.angular.z; // Rotational speed

  // Differential drive calculations
  // Using the following formulas:
  // Angular velocity: w = (R_w / (l * n)) * (w_r - w_l)
  // Linear velocity:  v = (R_w * w_m) / n

  // Calculate left and right wheel speeds based on linear and angular velocities
  // Note: Here we assume that (l / 2) is half the distance between the wheels.
  float left_wheel_speed = linear_velocity - angular_velocity * (l / 2);
  float right_wheel_speed = linear_velocity + angular_velocity * (l / 2);

  // Convert wheel speeds to motor speeds (scale as needed)
  left_motor_speed = constrain(left_wheel_speed * 255, -255, 255);
  right_motor_speed = constrain(right_wheel_speed * 255, -255, 255);

  // Set motor directions and speeds
  setMotorSpeeds();
}

// ROS subscriber to cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

// Function to set motor directions and speeds
void setMotorSpeeds() {
  // Left motor control
  if (left_motor_speed > 0) {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  } else if (left_motor_speed < 0) {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  } else {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  }
  analogWrite(LEFT_MOTOR_SPEED_PIN, abs(left_motor_speed));

  // Right motor control
  if (right_motor_speed > 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  } else if (right_motor_speed < 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  } else {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  }
  analogWrite(RIGHT_MOTOR_SPEED_PIN, abs(right_motor_speed));
}

void setup() {
  // Initialize motor control pins as outputs
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);

  // Initialize ROS node handle
  nh.initNode();

  // Subscribe to cmd_vel topic
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  // Spin ROS to process callbacks
  nh.spinOnce();
  
  // Delay to prevent CPU overload
  delay(10);
}

