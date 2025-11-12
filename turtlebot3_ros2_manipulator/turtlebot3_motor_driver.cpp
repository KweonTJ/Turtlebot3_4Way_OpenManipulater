/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "turtlebot3_motor_driver.h"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337; 
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882; 

/* DYNAMIXEL Information for controlling motors and  */
// const uint8_t DXL_MOTOR_ID_LEFT = 1; // ID of left motor
// const uint8_t DXL_MOTOR_ID_RIGHT = 2; // ID of right motor

const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);

// Turtlebot3MotorDriver::Turtlebot3MotorDriver()
// : left_wheel_id_(DXL_MOTOR_ID_LEFT),
//   right_wheel_id_(DXL_MOTOR_ID_RIGHT),
//   torque_(false)
// {
// }
Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: torque_(false)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  // sync_write_param.id_count = 4;
  // sync_write_param.xel[LEFT].id = left_wheel_id_;
  // sync_write_param.xel[RIGHT].id = right_wheel_id_;
  sync_write_param.id_count = 4;
  sync_write_param.xel[MortorLocation::LEFT_REAR].id  = DXL_MOTOR_ID_LEFT_REAR;
  sync_write_param.xel[MortorLocation::RIGHT_REAR].id = DXL_MOTOR_ID_RIGHT_REAR;
  sync_write_param.xel[MortorLocation::LEFT_FRONT].id = DXL_MOTOR_ID_LEFT_FRONT;
  sync_write_param.xel[MortorLocation::RIGHT_FRONT].id = DXL_MOTOR_ID_RIGHT_FRONT;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  // sync_read_param.id_count = 2;
  // sync_read_param.xel[LEFT].id = left_wheel_id_;
  // sync_read_param.xel[RIGHT].id = right_wheel_id_;
  sync_read_param.id_count = 4;
  sync_read_param.xel[MortorLocation::LEFT_REAR].id  = DXL_MOTOR_ID_LEFT_REAR;
  sync_read_param.xel[MortorLocation::RIGHT_REAR].id = DXL_MOTOR_ID_RIGHT_REAR;
  sync_read_param.xel[MortorLocation::LEFT_FRONT].id = DXL_MOTOR_ID_LEFT_FRONT;
  sync_read_param.xel[MortorLocation::RIGHT_FRONT].id = DXL_MOTOR_ID_RIGHT_FRONT;

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

Dynamixel2Arduino& Turtlebot3MotorDriver::getDxl()
{
  return dxl;
}

bool Turtlebot3MotorDriver::is_connected()
{
  // return (dxl.ping(DXL_MOTOR_ID_LEFT_REAR) == true &&
  //         dxl.ping(DXL_MOTOR_ID_RIGHT_REAR) == true &&
  //         dxl.ping(DXL_MOTOR_ID_LEFT_FRONT) == true &&
  //         dxl.ping(DXL_MOTOR_ID_RIGHT_FRONT) == true);
  return  dxl.ping(DXL_MOTOR_ID_LEFT_REAR)
       && dxl.ping(DXL_MOTOR_ID_RIGHT_REAR)
       && dxl.ping(DXL_MOTOR_ID_LEFT_FRONT)
       && dxl.ping(DXL_MOTOR_ID_RIGHT_FRONT);
}

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  // sync_write_param.xel[LEFT].data[0] = onoff;
  // sync_write_param.xel[RIGHT].data[0] = onoff;
  sync_write_param.xel[MortorLocation::LEFT_REAR].data[0]   = onoff;
  sync_write_param.xel[MortorLocation::RIGHT_REAR].data[0]  = onoff;
  sync_write_param.xel[MortorLocation::LEFT_FRONT].data[0]  = onoff;
  sync_write_param.xel[MortorLocation::RIGHT_FRONT].data[0] = onoff;


  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

// bool Turtlebot3MotorDriver::get_torque()
// {
//   if(dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_LEFT_REAR) == true
//     && dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_RIGHT_REAR) == true
//     && dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_LEFT_FRONT) == true
//     && dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_RIGHT_FRONT) == true){
//     torque_ = true;
//   }
//   else{
//     torque_ = false;
//   }

//   return torque_;
// }

bool Turtlebot3MotorDriver::get_torque()
{
  torque_ =
      dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_LEFT_REAR)
   && dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_RIGHT_REAR)
   && dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_LEFT_FRONT)
   && dxl.readControlTableItem(TORQUE_ENABLE, DXL_MOTOR_ID_RIGHT_FRONT);
  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool Turtlebot3MotorDriver::read_present_position(int32_t &left_value, int32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    // memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    // memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    int32_t pos_lr, pos_rr, pos_lf, pos_rf;
    
    // [수정] 네임스페이스 추가
    memcpy(&pos_lr, read_result.xel[MortorLocation::LEFT_REAR].data, read_result.xel[MortorLocation::LEFT_REAR].length);
    memcpy(&pos_rr, read_result.xel[MortorLocation::RIGHT_REAR].data, read_result.xel[MortorLocation::RIGHT_REAR].length);
    memcpy(&pos_lf, read_result.xel[MortorLocation::LEFT_FRONT].data, read_result.xel[MortorLocation::LEFT_FRONT].length);
    memcpy(&pos_rf, read_result.xel[MortorLocation::RIGHT_FRONT].data, read_result.xel[MortorLocation::RIGHT_FRONT].length);

    // [수정] 좌/우 각각 앞뒤 바퀴의 평균값 계산
    left_value  = (pos_lr + pos_lf) / 2;
    right_value = (pos_rr + pos_rf) / 2;

    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_velocity(int32_t &left_value, int32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    // memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    // memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    int32_t vel_lr, vel_rr, vel_lf, vel_rf;
    
    memcpy(&vel_lr, read_result.xel[MortorLocation::LEFT_REAR].data, read_result.xel[MortorLocation::LEFT_REAR].length);
    memcpy(&vel_rr, read_result.xel[MortorLocation::RIGHT_REAR].data, read_result.xel[MortorLocation::RIGHT_REAR].length);
    memcpy(&vel_lf, read_result.xel[MortorLocation::LEFT_FRONT].data, read_result.xel[MortorLocation::LEFT_FRONT].length);
    memcpy(&vel_rf, read_result.xel[MortorLocation::RIGHT_FRONT].data, read_result.xel[MortorLocation::RIGHT_FRONT].length);
    // [수정] 좌/우 각각 앞뒤 바퀴의 평균값 계산
    left_value  = (vel_lr + vel_lf) / 2;
    right_value = (vel_rr + vel_rf) / 2;

    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_current(int16_t &left_value, int16_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;

  if(dxl.syncRead(sync_read_param, read_result)){
    // memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    // memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    int16_t curr_lr, curr_rr, curr_lf, curr_rf;
    
    memcpy(&curr_lr, read_result.xel[MortorLocation::LEFT_REAR].data, read_result.xel[MortorLocation::LEFT_REAR].length);
    memcpy(&curr_rr, read_result.xel[MortorLocation::RIGHT_REAR].data, read_result.xel[MortorLocation::RIGHT_REAR].length);
    memcpy(&curr_lf, read_result.xel[MortorLocation::LEFT_FRONT].data, read_result.xel[MortorLocation::LEFT_FRONT].length);
    memcpy(&curr_rf, read_result.xel[MortorLocation::RIGHT_FRONT].data, read_result.xel[MortorLocation::RIGHT_FRONT].length);

    // [수정] 좌/우 각각 앞뒤 바퀴의 평균값 계산
    left_value  = (curr_lr + curr_lf) / 2;
    right_value = (curr_rr + curr_rf) / 2;

    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t &left_value, uint32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    // memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    // memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    uint32_t a_lr, a_rr, a_lf, a_rf;
    memcpy(&a_lr, read_result.xel[MortorLocation::LEFT_REAR].data,  read_result.xel[MortorLocation::LEFT_REAR].length);
    memcpy(&a_rr, read_result.xel[MortorLocation::RIGHT_REAR].data, read_result.xel[MortorLocation::RIGHT_REAR].length);
    memcpy(&a_lf, read_result.xel[MortorLocation::LEFT_FRONT].data, read_result.xel[MortorLocation::LEFT_FRONT].length);
    memcpy(&a_rf, read_result.xel[MortorLocation::RIGHT_FRONT].data,read_result.xel[MortorLocation::RIGHT_FRONT].length);
    left_value = (a_lr + a_lf) / 2;
    right_value = (a_rr + a_rf) / 2;

    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::write_velocity(int32_t left_value, int32_t right_value)
{
  bool ret = false;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  // memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
  // memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::LEFT_REAR].data,   &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::RIGHT_REAR].data,  &right_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::LEFT_FRONT].data,  &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::RIGHT_FRONT].data, &right_value, sync_write_param.length);


  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t left_value, uint32_t right_value)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  // memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
  // memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::LEFT_REAR].data,   &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::RIGHT_REAR].data,  &right_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::LEFT_FRONT].data,  &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[MortorLocation::RIGHT_FRONT].data, &right_value, sync_write_param.length);


  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::control_motors(const float wheel_separation, float linear_value, float angular_value)
{
  bool dxl_comm_result = false;
  
  // float wheel_velocity[MortorLocation::MOTOR_NUM_MAX];
  // float lin_vel = linear_value;
  // float ang_vel = angular_value;

  // wheel_velocity[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  // wheel_velocity[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  // wheel_velocity[LEFT]  = constrain(wheel_velocity[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  // wheel_velocity[RIGHT] = constrain(wheel_velocity[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  float wheel_velocity[MortorSide::SIDE_NUM_MAX];
  float lin_vel = linear_value;
  float ang_vel = angular_value;

  wheel_velocity[MortorSide::LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[MortorSide::RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  // wheel_velocity[LEFT]  = constrain(
  //   wheel_velocity[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  // wheel_velocity[RIGHT] = constrain(
  //   wheel_velocity[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  wheel_velocity[MortorSide::LEFT]  = constrain(
      wheel_velocity[MortorSide::LEFT]  * VELOCITY_CONSTANT_VALUE, -((float)LIMIT_X_MAX_VELOCITY), ((float)LIMIT_X_MAX_VELOCITY));
  wheel_velocity[MortorSide::RIGHT] = constrain(
      wheel_velocity[MortorSide::RIGHT] * VELOCITY_CONSTANT_VALUE, -((float)LIMIT_X_MAX_VELOCITY), ((float)LIMIT_X_MAX_VELOCITY));


  dxl_comm_result = write_velocity((int32_t)wheel_velocity[MortorSide::LEFT], (int32_t)wheel_velocity[MortorSide::RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
