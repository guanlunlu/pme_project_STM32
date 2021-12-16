#ifndef __control_H
#define __control_H

#include "head.h"
#include "gpio.h"
//#include "locate.h"
//#include "tim.h"
extern struct ROBOT ROBOT;

struct Kinematics{
	double 						acceleration;
	double						deceleration;
	double 						jerk;
	double						max_acc;
	double						min_acc;
	double						max_dec;
	double						min_dec;
	double						threshold_vel_for_smaller_acc;
	double						threshold_vel_for_smaller_dec;
};

struct Hardware_Info{
	GPIO_TypeDef 				*Cw_port;
	uint16_t 					Cw_pin;
	GPIO_TypeDef 				*Ccw_port;
	uint16_t 					Ccw_pin;
	TIM_HandleTypeDef 			*pwm_timer;
	TIM_HandleTypeDef 			*encTimer;
	uint32_t 					pwm_channel;
	double 						wheelDiameter;
	double						ReductionRatio;
	short 						encDirection;
	int  						encResolution;
};

struct MOTOR{
	int 						outputSaturation;
	int 						startSaturation;
	int							startSaturationCount;
	double 						speedMargin;
	double 						command;
	double						previous_speed;
	double						delta_vel;
	double 						omega;
	double						target;
	double						error;
	double						accumulated_error;

	struct	Kinematics			kinematics;
	struct  Hardware_Info		hardware_info;
	int 						counter_value;
	int							previous_counter_value;
	double						delta_counter;
	double						outputCNT;
};

struct Control{
	double Kp, Ki;
	double controller_output[3];
	double preConstant;
	struct MOTOR motor[3];
	//variables for internal encoder
	double angularVelocity;
	double linearVelocity_x;
	double linearVelocity_y;
	double ControlFreq;
	double counterRecordL, counterRecordR;
	double max_PWM_counter;
};






void PID(struct ROBOT *robot);
void interpolation(struct ROBOT *robot,double linearV, double angularV);
void s_curve(struct ROBOT * robot,double linearVx,double linearVy, double angularV);

#endif
