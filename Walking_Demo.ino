#include <my_ArduinoHardware.h>
#include <ros.h>
#include "ros.h"
#include <MyDynamixel_Walker.h>
// #include <MyDynamixel.h>
#include <VoltageAndTemperature.h>
#include <PositionAndSpeed.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

//Front Right Leg Motors
#define DXL_ID_11  11
#define DXL_ID_12  12
#define DXL_ID_13  13
//Front Left Leg Motors
#define DXL_ID_21  21
#define DXL_ID_22  22
#define DXL_ID_23  23
//BackLeft Leg Motors
#define DXL_ID_31  31
#define DXL_ID_32  32
#define DXL_ID_33  33
//Back Right Leg Motors
#define DXL_ID_41  41
#define DXL_ID_42  42
#define DXL_ID_43  43
//Pan Joint Motor
#define DXL_ID_71   71
#define DXL_ID_PAN  71
//Tilt Joint Motor
#define DXL_ID_72   72
#define DXL_ID_TILT 72
//Ammo Feeder Wheel Motor
#define DXL_ID_73   73
#define DXL_ID_LOAD 73

uint8_t DXL_MOTOR_IDS[] = {DXL_ID_11, DXL_ID_12, DXL_ID_13, DXL_ID_21, DXL_ID_22, DXL_ID_23, DXL_ID_31, DXL_ID_32, DXL_ID_33, DXL_ID_41, DXL_ID_42, DXL_ID_43, DXL_ID_71, DXL_ID_72, DXL_ID_73};


ros::NodeHandle nh;
MyDynamixelWalker Robot;

my_ros_robotics::VoltageAndTemperature temp_and_volt_msg;
my_ros_robotics::PositionAndSpeed position_and_speed_msg;

ros::Publisher pub_temp_and_volt_array("/walker/motor_temp_and_voltage", &temp_and_volt_msg);
ros::Publisher pub_position_and_speed("/walker/motor_position_and_speed", &position_and_speed_msg);

uint8_t motor_id[] = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 71, 72, 73};
uint16_t motor_position[15];
uint16_t motor_velocity[15];

uint8_t motor_id12[] = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};
uint8_t motor_id1[] = {11};
uint16_t motor_position1[] = {512};
uint16_t motor_velocity1[] = {0};

uint8_t temperatures[15];
uint8_t voltages[15];
// We write speeds to all 15 motors
uint8_t speeds[15];
// Don't write position to motor 15 as it is in wheel mode
// We only write positions to motors in joint mode
uint8_t positions[14];

uint32_t pre_pub_time_temp_volt;
uint32_t pre_pub_time_pos_spd;
uint32_t pre_time_move;

uint8_t robotMoving;
uint8_t robotStep;

RobotCommandType RobotCommand;

void messageMoveForwardBack( const std_msgs::Int8& move_forward_back_msg){
  Robot.currentMFB = move_forward_back_msg.data/128.0;
}
void messageMoveLeftRight( const std_msgs::Int8& move_left_right_msg){
  Robot.currentMLR = move_left_right_msg.data/128.0;
}
void messageTwistLeftRight( const std_msgs::Int8& twist_left_right_msg){
  Robot.currentTLR = twist_left_right_msg.data/128.0;
}

void messageRelaxLegs( const std_msgs::Int8& button_press_msg){
  // TrackSpeed.button_press_number = button_press_msg.data;
}
//
void messageLoaderSpeed( const std_msgs::UInt16& loader_speed_msg){
  Robot.loaderSpeed = loader_speed_msg.data;
}
void messageAimLeftRight( const std_msgs::UInt16& aim_left_right_msg){
  Robot.gunAimLR = aim_left_right_msg.data;
}
void messageAimUpDown( const std_msgs::UInt16& aim_up_down_msg){
  Robot.gunAimUD = aim_up_down_msg.data;
}

ros::Subscriber<std_msgs::Int8> subMoveForwardBack("/walker/move_forward_back_value", &messageMoveForwardBack );
ros::Subscriber<std_msgs::Int8> subMoveRightLeft("/walker/move_left_right_value", &messageMoveLeftRight );
ros::Subscriber<std_msgs::Int8> subTwistLeftRight("/walker/twist_left_right_value", &messageTwistLeftRight );
//
ros::Subscriber<std_msgs::Int8> subLeftStickButtonPress("/walker/relax_legs", &messageRelaxLegs );
//
ros::Subscriber<std_msgs::UInt16> subLoaderSpeed("/walker/load_speed_value", &messageLoaderSpeed );
ros::Subscriber<std_msgs::UInt16> subAimingLeftRight("/walker/aim_left_right_value", &messageAimLeftRight );
ros::Subscriber<std_msgs::UInt16> subAimingUpDown("/walker/aim_up_down_value", &messageAimUpDown );

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };

int delayTime = 250;
float walkingTime = .5;


void setup() {
  // Serial.begin(57600);
  // delay(500);
  pinMode(BDPIN_DIP_SW_1, INPUT);
  // pinMode(BDPIN_DIP_SW_2, INPUT);

  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);

  Robot.initializeRobot(15, DXL_MOTOR_IDS);

  uint16_t velocity[] = {100,100,100, 100,100,100, 100,100,100, 100,100,100};
  Robot.syncMotorVelocity(12, DXL_MOTOR_IDS, velocity);
  robotMoving = 0;

  // Robot.zeroFootPosition();

  nh.initNode();

  nh.subscribe(subMoveForwardBack);
  nh.subscribe(subMoveRightLeft);
  nh.subscribe(subTwistLeftRight);
  nh.subscribe(subLoaderSpeed);
  nh.subscribe(subAimingLeftRight);
  nh.subscribe(subAimingUpDown);
  nh.subscribe(subLeftStickButtonPress);

  nh.advertise(pub_temp_and_volt_array);
  nh.advertise(pub_position_and_speed);

  uint32_t time_now = millis();
  while (millis()-time_now < 1000){ } //buying time before zeroing feet

  digitalWrite(led_pin_user[2], LOW);
  // Robot.moveTrotToStep(0,0.5);
  Robot.moveWalkToStep(10,2.0);
  delay(500);
  Robot.moveWalkToStep(0,0.5);
  delay(500);
  digitalWrite(led_pin_user[2], HIGH);
  Robot.currentMFB = 1;

  // delayTime = 1500; too much time
  // walkingTime = 2.0;
  delayTime = 100; //maybe fasted for walk
  walkingTime = 0.200;
  delayTime = 75; //maybe fasted for walk
  walkingTime = 0.150;
}

void loop() {
  // Serial.print("looping:");
  // Serial.print(" Robot.currentMFB=");
  // Serial.print(Robot.currentMFB);
  // Serial.print(" Robot.currentMLR=");
  // Serial.print(Robot.currentMLR);
  // Serial.print(" Robot.currentTLR=");
  // Serial.print(Robot.currentTLR);
  // Serial.print(" ");
  // Serial.println("");
  // if (robotMoving == 0 and millis()-pre_pub_time_temp_volt >= 1000){
  // if (millis()-pre_pub_time_temp_volt >= 1000){
  //   Robot.reportMotorTemperatures(15,DXL_MOTOR_IDS, temperatures);
  //   Robot.reportMotorVoltages(15,DXL_MOTOR_IDS, voltages);
  //
  //   temp_and_volt_msg.header.stamp    = nh.now();
  //   temp_and_volt_msg.header.frame_id = "VT";
  //   temp_and_volt_msg.address = motor_id;
  //   temp_and_volt_msg.temperature = temperatures;
  //   temp_and_volt_msg.voltage = voltages;
  //   temp_and_volt_msg.address_length = 15;
  //   temp_and_volt_msg.temperature_length = 15;
  //   temp_and_volt_msg.voltage_length = 15;
  //   temp_and_volt_msg.alarm_led_error = 0;
  //   temp_and_volt_msg.shutdown_error = 0;
  //   pub_temp_and_volt_array.publish(&temp_and_volt_msg);
  //   pre_pub_time_temp_volt = millis();
  // }
  // float time_per_step = 1.0/8.0;
  // if (millis()-pre_time_move >= (int)(8000 * time_per_step) ){
  // if (millis()-pre_time_move >= (2000) ){
    // Robot.moveRobotViaTrot(time_per_step);
    // Robot.moveRobotViaTrot(1000);
    // Robot.moveRobotViaWalk(time_per_step);

  digitalWrite(led_pin_user[0], LOW);
  digitalWrite(led_pin_user[1], HIGH);
  digitalWrite(led_pin_user[2], LOW);
  digitalWrite(led_pin_user[3], LOW);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(1,walkingTime);
  }
  else {
    Robot.moveTrotToStep(1,walkingTime);
  }
  delay(delayTime);

  digitalWrite(led_pin_user[2], HIGH);
  digitalWrite(led_pin_user[3], HIGH);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(2,walkingTime);
  }
  else {
    Robot.moveTrotToStep(2,walkingTime);
  }
  delay(delayTime);

  digitalWrite(led_pin_user[3], LOW);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(3,walkingTime);
  }
  else {
    Robot.moveTrotToStep(3,walkingTime);
  }

  delay(delayTime);
  digitalWrite(led_pin_user[3], HIGH);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(4,walkingTime);
  }
  else {
    Robot.moveTrotToStep(4,walkingTime);
  }

  delay(delayTime);

  digitalWrite(led_pin_user[0], HIGH);
  digitalWrite(led_pin_user[1], LOW);
  digitalWrite(led_pin_user[3], LOW);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(5,walkingTime);
  }
  else {
    Robot.moveTrotToStep(5,walkingTime);
  }
  delay(delayTime);
  digitalWrite(led_pin_user[3], HIGH);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(6,walkingTime);
  }
  else {
    Robot.moveTrotToStep(6,walkingTime);
  }
  delay(delayTime);
  digitalWrite(led_pin_user[3], LOW);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(7,walkingTime);
  }
  else {
    Robot.moveTrotToStep(7,walkingTime);
  }
  delay(delayTime);
  digitalWrite(led_pin_user[3], HIGH);
  if (digitalRead(BDPIN_DIP_SW_1)) {
    Robot.moveWalkToStep(8,walkingTime);
  }
  else {
    Robot.moveTrotToStep(8,walkingTime);
  }
  delay(delayTime);
    // pre_time_move = millis();
    // robotMoving = Robot.currentMoving;
  // }
  //
  // if (millis()-pre_pub_time_pos_spd >= 1000){
  // Robot.reportNextMotorPosition(motor_position);
  // Robot.reportMotorVelocities(motor_velocity);
  // position_and_speed_msg.header.stamp    = nh.now();
  // position_and_speed_msg.header.frame_id = "PS";
  // position_and_speed_msg.address = motor_id;
  // position_and_speed_msg.position = motor_position;
  // position_and_speed_msg.speed = motor_velocity;
  // position_and_speed_msg.address_length = 15;
  // position_and_speed_msg.position_length = 15;
  // position_and_speed_msg.speed_length = 15;
  // pub_position_and_speed.publish(&position_and_speed_msg);
  // pre_pub_time_pos_spd = millis();
  // }
  // nh.spinOnce();
  // delay(1);

}
