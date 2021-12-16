#ifndef __robot_H
#define __robot_H

#include "head.h"
#include "control.h"
#include "spi.h"
#include "communicate.h"
#include "usart.h"
#include "stepper.h"
#include "pmw.h"
#include "mpu6050.h"
#include "end_joint.h"


#define motorMax 890
//max omega of the motor
#define TotalTime 100*1000
// 4 and half minute in a game which express in seconds


#define DistanceBetweenWheel   170  //565  //mm

#define DistanceBetweenEnc  90.75

struct ROBOT{
	 int PastTime;
	 int timeCount;
	 int ready;
	 struct Control control;
	 pmw pmw3901;
	 int deltaXSum;
	 int deltaYSum;
	 Scara* arm_ptr;
	 MPU6050_t imu;
	 struct End_Joint end_joint;
//	 struct Locate locate;
	 struct Communicate communicate;
	 short pose_init_cplt;
	 void (*setMotor)(struct MOTOR * motor,bool orientation,int speed);
	 void (*stopActing)(struct ROBOT *robot);

};



void Initiate(struct ROBOT * robot);
void ControlInit(struct ROBOT * robot);
void CommunicateInit(struct Communicate * communicate);
void Deinit(struct ROBOT * robot);

void driving_module_init(struct ROBOT * robot);

void wheel_init(struct MOTOR *motor);
void wheel_hardware_init(struct  Hardware_Info *hardware_info, GPIO_TypeDef * GPIOx_A, uint16_t GPIO_PIN_INA, GPIO_TypeDef * GPIOx_B, uint16_t GPIO_PIN_INB, TIM_HandleTypeDef * pwm_htim, uint32_t pwm_channel, TIM_HandleTypeDef *enc_htim, double wheelDiameter,double ReductionRatio, short encDirection, int encResolution);
void wheel_kinematics_init(struct  Kinematics *kinematics, double acceleration, double deceleration, double jerk, double max_acc, double min_acc, double max_dec, double min_dec, double threshold_vel1,double threshold_vel2);


void setMotorSpeed(struct MOTOR * motor, bool Orientation, int speed);
void SysInput(struct MOTOR * motor,int InputType,int timeCount);
void setLocation(struct ROBOT * robot,int whichDirectionFirst);
void setDegree(struct ROBOT * robot, double targetDegree);
void wheel_calibration(struct ROBOT * robot, int spin_or_move_forward,int speed);


#endif
