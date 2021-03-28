#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>

#include <Dynamixel2Arduino.h>
//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
#define MOVING_SPEED_ADDR 32
#define MOVING_SPEED_ADDR_LEN 2
#define TIMEOUT 10

uint16_t movingSpeed = 400;
const uint8_t DXL_DIR_PIN = 28;
const float DXL_PROTOCOL_VERSION = 1.0;

char *joint_names[12] = {"lf1", "lf2", "lf3",   //ending in 1 - hip connection
                         "rf1", "rf2", "rf3",   //ending in 2 - upper leg to hip
                         "lh1", "lh2", "lh3",   //ending in 3 - lower leg to hip  
                         "rh1", "rh2", "rh3"
                        };

const float offset[12] = {2.61799, 2.61799, 4.18879,   //radian offset from 0 servo position to 0 joint position on URDF
                          2.61799, 2.61799, 1.0472,
                          2.61799, 2.61799, 1.0472,
                          2.61799, 2.61799, 4.18879
                         };

const float orientation[12] = { 1,  1,  1,
                                1, -1, -1,
                               -1,  1,  1,
                               -1, -1, -1
                              };

float pos[12] = {0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0
                };

float desired[12];

ros::NodeHandle nh;
sensor_msgs::JointState joint_states;
trajectory_msgs::JointTrajectoryPoint points;

void jointtraj_callback(const trajectory_msgs::JointTrajectory& msg) {
  points = msg.points[0];
  for (int x = 0; x < 12; x++) {
    desired[x] = points.positions[x];
  }
}


//sensor_msgs::Imu imu_msg;
ros::Subscriber <trajectory_msgs::JointTrajectory> jointtraj_sub("/joint_group_position_controller/command", &jointtraj_callback);
ros::Publisher joint_states_pub("joint_states", &joint_states);
//ros::Publisher imu_pub("imu", &imu_msg);

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  nh.initNode();
  nh.advertise(joint_states_pub);
  //nh.advertise(imu_pub);
  nh.subscribe(jointtraj_sub);
  joint_states.name_length = 12;
  joint_states.position_length = 12;
  joint_states.name = joint_names;

  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int id = 1; id <= 12; id++) {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
  }
  //bno.setExtCrystalUse(true);
}

void loop() {
  //read_imu();
  read_servos();
  write_servos();
  nh.spinOnce();
}
/*
void read_imu() {
  imu::Quaternion quat = bno.getQuat();
  imu_msg.orientation.w = quat.w();
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_pub.publish(&imu_msg);
}
*/
void read_servos() {
  for (int i = 0; i < 12; i++) {
    pos[i] = ((((dxl.getPresentPosition((i + 1), UNIT_DEGREE)) * (3.14159 / 180)) - offset[i]) * orientation[i]);
  }
  joint_states.position = pos;
  joint_states.header.stamp = nh.now();
  joint_states_pub.publish(&joint_states);
}

void write_servos() {
  for (int i = 0; i < 12; i++) {
    dxl.write(i+1, MOVING_SPEED_ADDR, (uint8_t*)&movingSpeed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
    dxl.setGoalPosition((i + 1), ((((desired[i] * orientation[i]) + offset[i]) * 180) / 3.14159), UNIT_DEGREE);
  }
}
