/*
 * stepper.c
 *
 *  Created on: 2021年8月25日
 *      Author: lu
 */

#include "stepper.h"
static int stepper_precision = 16;			//microstep
//static float stop_tolerance = 0.031;		//speed lower limit
static float stop_tolerance = 0.1;		//speed lower limit
static float angle_per_step = 1.8/16;
static float reduction_ratio = 5.18;
float control_frequency = 1000.0;
float stepper_tracking_tolerance = 0.05;
Scara arm;
int a = 0;
float k = 0;

void stepper_init(Stepper_state *stepper, int motor_id,float x, float y, float init_angle, float min_angle, float max_angle){
	stepper->id = motor_id;
	stepper->origin_pose.x = x;
	stepper->origin_pose.y = y;
	stepper->span[0] = min_angle;
	stepper->span[1] = max_angle;
	stepper->hardware.init_angle = init_angle;
//	stepper->initialized = 1;
}

float stepper_angle_accessor(Stepper_state *stepper){
	return stepper->angle;
}

void stepper_hardware_set(Stepper_state *stepper,GPIO_TypeDef *dir_port,uint16_t dir_pin,GPIO_TypeDef *step_port,uint16_t step_pin,GPIO_TypeDef *MS1_port,uint16_t MS1_pin,GPIO_TypeDef * MS2_port,uint16_t MS2_pin,GPIO_TypeDef *sw_port,uint16_t sw_pin,TIM_HandleTypeDef *pwm_timer, unsigned int pwm_channel){
	stepper->hardware.DIR_port = dir_port;
	stepper->hardware.DIR_pin = dir_pin;

	stepper->hardware.STEP_port = step_port;
	stepper->hardware.STEP_pin = step_pin;

	stepper->hardware.MS1_port = MS1_port;
	stepper->hardware.MS1_pin = MS1_pin;

	stepper->hardware.MS2_port = MS2_port;
	stepper->hardware.MS2_pin = MS2_pin;

	stepper->hardware.switch_port = sw_port;
	stepper->hardware.switch_pin = sw_pin;

	stepper->hardware.pwm_timer = pwm_timer;
	stepper->hardware.pwm_channel = pwm_channel;

	HAL_TIM_PWM_Start_IT(pwm_timer,pwm_channel);
}

void linkage_init(Linkage *link,float x,float y,int length,float angle){
	link->angle = angle;
	link->length = length;
	link->origin.x = x;
	link->origin.y = y;
}

void arm_initialize(){
	arm.cmdvel_x = 0;
	arm.cmdvel_y = 0;

	arm.left.param_init = stepper_init;
	arm.right.param_init = stepper_init;
	arm.left.hardware_init = stepper_hardware_set;
	arm.right.hardware_init = stepper_hardware_set;
	arm.link_L1.init = linkage_init;
	arm.link_R1.init = linkage_init;
	arm.link_L2.init = linkage_init;
	arm.link_R2.init = linkage_init;




	arm.right.param_init(&arm.right,0,-21,0,theta_domain_convert(radian(-10.01)),theta_domain_convert(radian(90)),theta_domain_convert(radian(190.01)));
	arm.right.hardware_init(&arm.right,GPIOF,GPIO_PIN_11,GPIOF,GPIO_PIN_8,GPIOF,GPIO_PIN_14,GPIOF,GPIO_PIN_13,GPIOG,GPIO_PIN_5,&htim13,TIM_CHANNEL_1);
//	// ---------------------------------------
	arm.left.param_init(&arm.left,1,21,0,theta_domain_convert(radian(-170)),theta_domain_convert(radian(-10.01)),theta_domain_convert(radian(90)));
	arm.left.hardware_init(&arm.left,GPIOF,GPIO_PIN_15,GPIOB,GPIO_PIN_9,GPIOE,GPIO_PIN_7,GPIOG,GPIO_PIN_1,GPIOG,GPIO_PIN_6,&htim11,TIM_CHANNEL_1);
//
//	/* mechanical parameter */
	arm.link_L1.init(&arm.link_L1,-21,0,144,theta_domain_convert(radian(-170)));
	arm.link_R1.init(&arm.link_R1,-21,0,144,theta_domain_convert(radian(-10)));
	arm.link_L2.init(&arm.link_L2,0,0,184,theta_domain_convert(radian(-170)));
	arm.link_R2.init(&arm.link_R2,0,0,184,theta_domain_convert(radian(-10)));
//
	update_linkage_endpose(&arm);
	scara_init(&arm);
}

Scara* scara_accessor(){
	return &arm;
}

void stepper_set(){
//	left stepper
//  stepper3
	HAL_GPIO_WritePin(arm.left.hardware.MS1_port, arm.left.hardware.MS1_pin, GPIO_PIN_SET);	//MS1
	HAL_GPIO_WritePin(arm.left.hardware.MS2_port, arm.left.hardware.MS2_pin, GPIO_PIN_SET);	//MS2

//	right motor
//	stepper4
	HAL_GPIO_WritePin(arm.right.hardware.MS1_port, arm.right.hardware.MS1_pin, GPIO_PIN_SET);	//MS1
	HAL_GPIO_WritePin(arm.right.hardware.MS2_port, arm.right.hardware.MS2_pin, GPIO_PIN_SET);	//MS2
}

void stepper_move(Scara *arm, float L_vel, float R_vel){
//	velocity in rad/s

//	if(HAL_GPIO_ReadPin(arm->right.hardware.switch_port, arm->right.hardware.switch_pin)==0 && R_vel < 0){
//		R_vel = 0;
//	}
//	if(HAL_GPIO_ReadPin(arm->left.hardware.switch_port, arm->left.hardware.switch_pin)==0 && L_vel > 0){
//		L_vel = 0;
//	}
//	L_vel /= 1.05;
//	R_vel /= 1.05;
//	if( L_vel < 0 &&  R_vel > 0 &&( || ))
//	left stepper
//	arm->left.hardware.pwm_timer->Instance->ARR = 10000000 / (reduction_ratio * abs(L_vel) * 180/PI/(1.8/stepper_precision));
	arm->left.hardware.pwm_timer->Instance->ARR = 10000000 / (reduction_ratio * abs(L_vel) * 509.2958179);
//	arm->left.hardware.pwm_timer->Instance->ARR = 10000000 / (reduction_ratio * abs(k) * 180/PI/(1.8/stepper_precision));
	if (fabs(L_vel) < stop_tolerance)
		HAL_TIM_PWM_Stop(arm->left.hardware.pwm_timer, arm->left.hardware.pwm_channel);
	else if(L_vel > 0){
		arm->left.direction = 1;
		HAL_TIM_PWM_Start_IT(arm->left.hardware.pwm_timer, arm->left.hardware.pwm_channel);
		stepper_set(&arm);
		HAL_GPIO_WritePin(arm->left.hardware.DIR_port, arm->left.hardware.DIR_pin, GPIO_PIN_RESET);	//DIR
		__HAL_TIM_SetCompare(arm->left.hardware.pwm_timer, arm->left.hardware.pwm_channel, arm->left.hardware.pwm_timer->Instance->ARR/2);
	}
	else if(L_vel < 0){
		arm->left.direction = 0;
		HAL_TIM_PWM_Start_IT(arm->left.hardware.pwm_timer, arm->left.hardware.pwm_channel);
		stepper_set(&arm);
		HAL_GPIO_WritePin(arm->left.hardware.DIR_port, arm->left.hardware.DIR_pin, GPIO_PIN_SET);//DIR
		__HAL_TIM_SetCompare(arm->left.hardware.pwm_timer, arm->left.hardware.pwm_channel, arm->left.hardware.pwm_timer->Instance->ARR/2);
	}

//	right stepper
	arm->right.hardware.pwm_timer->Instance->ARR = 10000000 / (stepper_precision * reduction_ratio * abs(R_vel) * 180/PI/1.8);
	if (fabs(R_vel) < stop_tolerance)
		HAL_TIM_PWM_Stop(arm->right.hardware.pwm_timer, arm->right.hardware.pwm_channel);
	else if(R_vel > 0){
		arm->right.direction = 1;
		HAL_TIM_PWM_Start_IT(arm->right.hardware.pwm_timer, arm->right.hardware.pwm_channel);
		stepper_set(&arm);
		HAL_GPIO_WritePin(arm->right.hardware.DIR_port, arm->right.hardware.DIR_pin, GPIO_PIN_RESET); //DIR
		__HAL_TIM_SetCompare(arm->right.hardware.pwm_timer, arm->right.hardware.pwm_channel, arm->right.hardware.pwm_timer->Instance->ARR/2);
	}
	else if(R_vel < 0){
		arm->right.direction = 0;
		HAL_TIM_PWM_Start_IT(arm->right.hardware.pwm_timer, arm->right.hardware.pwm_channel);
		stepper_set(&arm);
		HAL_GPIO_WritePin(arm->right.hardware.DIR_port, arm->right.hardware.DIR_pin, GPIO_PIN_SET); //DIR
		__HAL_TIM_SetCompare(arm->right.hardware.pwm_timer, arm->right.hardware.pwm_channel, arm->right.hardware.pwm_timer->Instance->ARR/2);
	}
}

void stepper_angle_tracking(Scara *arm, float L_goal, float R_goal){
	float vel = 2*PI/3;
	float L_vel = vel;
	float R_vel = vel;
//	left stepper
	if (arm->left.initialized){
		if(fabs(arm->left.angle - L_goal) < stepper_tracking_tolerance){
			L_vel = 0;
		}
		if(arm->left.angle * L_goal < 0){
			if (L_goal - arm->left.angle > 0)
				L_vel = -vel;
		}
		else{
			if (L_goal - arm->left.angle < 0)
				L_vel = -vel;
		}
	}
	
//  right stepper
	if(arm->right.initialized){
		if(fabs(arm->right.angle - R_goal) < stepper_tracking_tolerance)
			R_vel = 0;
		if(arm->right.angle > R_goal){
			R_vel = -vel;
		}
	}
	stepper_move(arm, L_vel, R_vel);
}

void scara_init(Scara *arm){
	int b = 0;
//	arm->left.angle = 0;
	b = arm->left.angle;
	while(HAL_GPIO_ReadPin(arm->left.hardware.switch_port, arm->left.hardware.switch_pin)!=0){
		stepper_move(arm, PI/2, 0);
	}
	stepper_move(arm, 0, 0);
//	arm->left.initialized = 1;
//	arm->left.step = 0;
	arm->left.angle = arm->left.hardware.init_angle;
	arm->link_L1.angle = arm->left.angle;

	while(fabs(arm->left.angle - (3*PI / 4)) > 0.1){
		stepper_move(arm, -PI/2, 0);
	}
	arm->left.tracking_angle = arm->left.angle;
	stepper_move(arm, 0, 0);
	arm->left.initialized = 1;
	arm->left.step = 0;


	while(HAL_GPIO_ReadPin(arm->right.hardware.switch_port, arm->right.hardware.switch_pin)!=0){
		stepper_move(arm, 0, -PI/2);
		a --;
//		HAL_GPIO_WritePin(arm->right.hardware.DIR_port, arm->right.hardware.DIR_pin, GPIO_PIN_RESET);	//DIR
//		__HAL_TIM_SetCompare(arm->right.hardware.pwm_timer, arm->right.hardware.pwm_channel, arm->right.hardware.pwm_timer->Instance->ARR/2);
	}
	stepper_move(arm, 0, 0);

	arm->right.angle = arm->right.hardware.init_angle;
	arm->link_R1.angle = arm->right.angle;
	while(fabs(arm->right.angle - (PI / 4)) > 0.05){
			stepper_move(arm, 0, PI/2);
	}

	stepper_move(arm,0,0);
	arm->right.initialized = 1;
	arm->right.step = 0;
	arm->right.tracking_angle = arm->right.angle;

	update_linkage_endpose(arm);
}

void update_linkage_endpose(Scara *arm){
	arm->link_L2.origin.x = arm->link_L1.origin.x + arm->link_L1.length * cos(arm->link_L1.angle);
	arm->link_L2.origin.y = arm->link_L1.origin.y + arm->link_L1.length * sin(arm->link_L1.angle);

	arm->link_R2.origin.x = arm->link_R1.origin.x + arm->link_R1.length * cos(arm->link_R1.angle);
	arm->link_R2.origin.y = arm->link_R1.origin.y + arm->link_R1.length * sin(arm->link_R1.angle);

	struct pose vec_t1;
	float len_t1 = 0;
	float len_t2 = 0;
	vec_t1.x = (arm->link_R2.origin.x - arm->link_L2.origin.x) / 2;
	vec_t1.y = (arm->link_R2.origin.y - arm->link_L2.origin.y) / 2;
	len_t1 = get_vec_length(vec_t1.x, vec_t1.y);
	len_t2 = sqrt(pow(arm->link_L2.length,2) - pow(len_t1,2));
	arm->link_L2.angle = theta_domain_convert(atan2(len_t2, len_t1) + atan2(vec_t1.y, vec_t1.x));

	arm->endjoint.x = arm->link_L2.origin.x + arm->link_L2.length * cos(arm->link_L2.angle);
	arm->endjoint.y = arm->link_L2.origin.y + arm->link_L2.length * sin(arm->link_L2.angle);
}

float inverse_kinematic(Stepper_state *motor, Pose target){
	Pose *motor_origin;
	Pose mt_vec;
	float l1 = 0;
	float l2 = 0;
	float l3 = 0;
	float theta_l2 = 0;
	float angle = 0;

	if(motor->id == 0)
		motor_origin = &arm.link_L1.origin; 
	if(motor->id == 1)
		motor_origin = &arm.link_R1.origin;

	mt_vec.x = target.x - motor_origin->x;
	mt_vec.y = target.y - motor_origin->y;
	l3 = get_vec_length(mt_vec.x, mt_vec.y);
	l1 = arm.link_L1.length;
	l2 = arm.link_L2.length;
	theta_l2 = acos( (pow(l1, 2)+pow(l3, 2)-pow(l2, 2) ) / (2 *l1 *l3) );

	if(motor->id == 0)
		angle = atan2(mt_vec.y, mt_vec.x) + theta_l2;
	else
		angle = atan2(mt_vec.y, mt_vec.x) - theta_l2;
	return theta_domain_convert(angle);
}

double theta_domain_convert(double input){
	double output = 0;
	if(input >=0){
		input = fmod(input, 2*PI);
		if(input > PI){
			input -= 2*PI;
			output = input;
		}
		else{
			output = input;
		}
	}
	else{
		input *= -1;
		input = fmod(input, 2*PI);
		if(input > PI){
			input -= 2*PI;
			output = input *-1;
		}
		else{
			output = input *-1;
		}
	}
	return output;
}

void arm_move(Scara *arm, float vel_x, float vel_y){
	Pose nextpoint;
	float d_t =  1 / control_frequency;
	float next_angle_L = 0;
	float next_angle_R = 0;
	float vel_L = 0;
	float vel_R = 0;

	update_linkage_endpose(arm);
	nextpoint.x = arm->endjoint.x + vel_x * d_t;
	nextpoint.y = arm->endjoint.y + vel_y * d_t;
	next_angle_L = inverse_kinematic(&arm->left, nextpoint);
	next_angle_R = inverse_kinematic(&arm->right, nextpoint);


	vel_L = (next_angle_L - arm->left.angle)/d_t;
	vel_R = (next_angle_R - arm->right.angle)/d_t;
	stepper_move(arm, vel_L, vel_R);
}

float radian(float degree){
	return degree * PI / 180;
}

float degree(float radian){
	return radian * 180 / PI;
}

float get_vec_length(float x, float y){
	return sqrt(pow(x,2) + pow(y,2));
}

int inspan(Stepper_state *motor, float angle){
	float limit_l = theta_domain_convert(arm.left.span[1]);
	float limit_h = theta_domain_convert(arm.right.span[1]);
	int inspan = 0;
	angle = theta_domain_convert(angle);
	if(angle > limit_l && angle < limit_h){
		inspan = 0;
	}
	else{
		inspan = 1;
		if(motor->id == 0){
			if(angle < motor->span[0])
				inspan = 0;
		}
		if(motor->id == 1){
			if(angle > motor->span[0])
				inspan = 0;
		}
	}
	return inspan;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == arm.left.hardware.pwm_timer) {
		if (arm.left.direction == 1) {
      		arm.left.step++;
			arm.left.angle += radian(angle_per_step/reduction_ratio);
		}
		if (arm.left.direction == 0) {
      		arm.left.step--;
			arm.left.angle -= radian(angle_per_step/reduction_ratio);
		}
		arm.left.angle = theta_domain_convert(arm.left.angle);
		arm.link_L1.angle = arm.left.angle;
//		if(arm.left.initialized == 1)
//			update_linkage_endpose(&arm);
	}

	if (htim == arm.right.hardware.pwm_timer) {
		if (arm.right.direction == 1) {
			arm.right.step ++;
			arm.right.angle += radian(angle_per_step/reduction_ratio);
		}
		if (arm.right.direction == 0) {
			arm.right.step--;
			arm.right.angle -= radian(angle_per_step/reduction_ratio);
		}
		arm.right.angle = theta_domain_convert(arm.right.angle);
		arm.link_R1.angle = arm.right.angle;
//		if(arm.right.initialized == 1)
//			update_linkage_endpose(&arm);
	}
}
