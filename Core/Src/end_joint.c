/*
 * end_joint.c
 *
 *  Created on: Aug 31, 2021
 *      Author: gogo
 */

#include "end_joint.h"

#define uppose 0.0
#define downpose1 8.5
#define downpose2 6.7

void endjoint_Init(struct End_Joint *end_joint){

	endjoint_hardware_set(end_joint);
	end_joint->init = false;
	end_joint->last_command = 2; //0 grab , 1 drop , 2 hold
	end_joint->work_state = stay;
	end_joint->_downpose = downpose1;
	end_joint->return_state = 1;


	//set init param
	end_joint->joint_length = 10;
	end_joint->real_init_height = 1.8159;
	end_joint->real_init_angle = 40*M_PI/180.0; // degree to rad
	end_joint->speed = 2;
	end_joint->compensate = 0.0;

	end_joint->cur_height = 0.0;
	end_joint->cur_angle = 0.0;
	end_joint->cur_duty = 1180;

	end_joint->desire_height = 0.0;
	end_joint->desire_angle = 0.0;
	end_joint->desire_duty = 1180;
	end_joint->direction = 0;

	end_joint->suck_duty = 4490;
	end_joint->sucker_state = false;
	end_joint->if_SuckObject = 2;
	end_joint->noLoad_bound = 3000;
	end_joint->Load_bound = 80;
	end_joint->adc_value = 0;

	servo_reset_pose(end_joint);
	sucker_reset(end_joint);

}

void endjoint_hardware_set(struct End_Joint *end_joint){
	// joint hardware_info set

	end_joint->joint_hardware.PWM_TIM_idx = TIM10;
	end_joint->joint_hardware.PWM_TIM = &htim10;
	end_joint->joint_hardware.CHANNEL = TIM_CHANNEL_1;

	HAL_TIM_PWM_Start_IT(end_joint->joint_hardware.PWM_TIM, end_joint->joint_hardware.CHANNEL);

	// sucker hardware_info set
//	end_joint->sucker_hardware.PWM_TIM = &htim12;
//	end_joint->sucker_hardware.PWM_CHANNEL = TIM_CHANNEL_2;

	end_joint->sucker_hardware.suck_port = GPIOB;
	end_joint->sucker_hardware.suck_pin = GPIO_PIN_15;

	end_joint->sucker_hardware.CW_port = GPIOD;
	end_joint->sucker_hardware.CW_pin = GPIO_PIN_8;
	end_joint->sucker_hardware.CCW_port = GPIOD;
	end_joint->sucker_hardware.CCW_pin = GPIO_PIN_14;

	end_joint->sucker_hardware.Relay_port = GPIOC;
	end_joint->sucker_hardware.Relay_pin = GPIO_PIN_0;

	end_joint->sucker_hardware.Psensor_adc = &hadc1;

	HAL_ADC_Start_IT(end_joint->sucker_hardware.Psensor_adc);
//	HAL_TIM_PWM_Start(end_joint->sucker_hardware.PWM_TIM, end_joint->sucker_hardware.PWM_CHANNEL);
}

void servo_reset_pose(struct End_Joint *end_joint){
//	end_joint->joint_hardware.CHANNEL_REGISTER = 360; // 0 degree
//	while(!end_joint->init){
		__HAL_TIM_SetCompare(end_joint->joint_hardware.PWM_TIM, end_joint->joint_hardware.CHANNEL,end_joint->desire_duty);
		end_joint->init = true;
//	}
}

void sucker_reset(struct End_Joint *end_joint){
	HAL_GPIO_WritePin(end_joint->sucker_hardware.CW_port, end_joint->sucker_hardware.CW_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(end_joint->sucker_hardware.CCW_port, end_joint->sucker_hardware.CCW_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(end_joint->sucker_hardware.suck_port, end_joint->sucker_hardware.suck_pin, 0);
//	__HAL_TIM_SetCompare(end_joint->sucker_hardware.PWM_TIM, end_joint->sucker_hardware.PWM_CHANNEL,0);

//	HAL_TIM_PWM_Stop(end_joint->sucker_hardware.PWM_TIM,end_joint->sucker_hardware.PWM_CHANNEL);

	HAL_GPIO_WritePin(end_joint->sucker_hardware.Relay_port, end_joint->sucker_hardware.Relay_pin, GPIO_PIN_RESET);
}

void endjoint_script(struct End_Joint *end_joint, int command){

	if(command == 0)
		end_joint->_downpose = downpose1+end_joint->compensate;
	else if(command == 1)
		end_joint->_downpose = downpose2;

	if( command != end_joint->last_command
		 && (command == 0 || command == 1) ) // 2->1 or 2->0
		end_joint->work_state = downward;
	else if(end_joint->cur_height == end_joint->_downpose && end_joint->work_state == downward ){
		end_joint->stay_start = HAL_GetTick();
		end_joint->work_state = waiting;
	}
	else if(end_joint->cur_height == uppose && end_joint->work_state == upward ){
		if(command == 0)
			end_joint->work_state = hold;
		else if(command == 1)
			end_joint->work_state = stay;
	}

	suck_check(end_joint);
	switch(end_joint->work_state){
	case hold:
		endjoint_work(end_joint,uppose,true);
		if(end_joint->if_SuckObject == 1)
			end_joint->return_state = 1;
		else{
			end_joint->return_state = 0;
			end_joint->compensate += 0.5;
			if(end_joint->compensate>1.5)
				end_joint->compensate = 1.5;
		}
		break;
	case stay:
		endjoint_work(end_joint,uppose,false);
		if(end_joint->if_SuckObject == 1)
			end_joint->return_state = 0;
		else
			end_joint->return_state = 1;
		break;
	case downward:
		endjoint_work(end_joint,end_joint->_downpose,true); //sucker open or close
		end_joint->return_state = 2;
		break;
	case upward:
		endjoint_work(end_joint,uppose,!command);
		end_joint->return_state = 2;
		break;
	case waiting:
		end_joint->stay_cur = HAL_GetTick();
		if(end_joint->stay_cur - end_joint->stay_start >= 1000)
			end_joint->work_state = upward;
		endjoint_work(end_joint,end_joint->_downpose,!command);
		end_joint->return_state = 2;
		break;
	}
	end_joint->last_command = command;


	//return success, fail or working



}

void endjoint_work(struct End_Joint *end_joint, double desire_height, bool sucker_active){
		//sucker_work
		end_joint->work = end_joint->work+1;
		end_joint->sucker_state = sucker_active;
		if(sucker_active==true){
			HAL_GPIO_WritePin(end_joint->sucker_hardware.suck_port, end_joint->sucker_hardware.suck_pin, 1);
//			__HAL_TIM_SetCompare(end_joint->sucker_hardware.PWM_TIM, end_joint->sucker_hardware.PWM_CHANNEL,end_joint->suck_duty);
			HAL_GPIO_WritePin(end_joint->sucker_hardware.Relay_port, end_joint->sucker_hardware.Relay_pin, GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(end_joint->sucker_hardware.suck_port, end_joint->sucker_hardware.suck_pin, 0);

//			__HAL_TIM_SetCompare(end_joint->sucker_hardware.PWM_TIM, end_joint->sucker_hardware.PWM_CHANNEL,0);

			if(end_joint->work_state == stay )
				HAL_GPIO_WritePin(end_joint->sucker_hardware.Relay_port, end_joint->sucker_hardware.Relay_pin, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(end_joint->sucker_hardware.Relay_port, end_joint->sucker_hardware.Relay_pin, GPIO_PIN_SET);
		}

		//servo work
		end_joint->desire_height = desire_height;
		double H = desire_height+end_joint->real_init_height;
		double fake_joint = pow( (4+pow(H,2)),0.5 );
		double angle1 = atan(H/2.0);
		double angle2 = acos(fake_joint/(2*end_joint->joint_length));
		end_joint->desire_angle =  fabs(angle2 - angle1 - end_joint->real_init_angle);
		end_joint->desire_duty = end_joint->desire_angle*(360-1180)/2.86234 + 1180 + 0.5;//+0.5 for rounding
		servo_move(end_joint);
}

void servo_move(struct End_Joint *end_joint){
	double duty_error = end_joint->desire_duty-end_joint->cur_duty;
	end_joint->direction = duty_error/fabs(duty_error);
	if( end_joint->speed < fabs(duty_error)){
		end_joint->cur_duty += end_joint->direction*end_joint->speed;
		__HAL_TIM_SetCompare(end_joint->joint_hardware.PWM_TIM, end_joint->joint_hardware.CHANNEL,end_joint->cur_duty);

		end_joint->cur_angle =  (end_joint->cur_duty-1180)*2.86234/(360-1190);
		double real_theta = 40*M_PI/180.0 - end_joint->cur_angle;
		double temp_theta = asin((end_joint->joint_length-2/cos(real_theta))*sin(M_PI_2+real_theta)/end_joint->joint_length);
		end_joint->cur_height = end_joint->joint_length/sin(M_PI_2+end_joint->cur_angle)*sin(M_PI_2-real_theta-temp_theta)
								-2*tan(real_theta)-end_joint->real_init_height;
	}
	else{
		__HAL_TIM_SetCompare(end_joint->joint_hardware.PWM_TIM, end_joint->joint_hardware.CHANNEL,end_joint->desire_duty);
		end_joint->cur_duty = end_joint->desire_duty;
		end_joint->cur_angle = end_joint->desire_angle;
		end_joint->cur_height = end_joint->desire_height;
	}
}

void suck_check(struct End_Joint *end_joint){
	if(end_joint->adc_value <= end_joint->Load_bound)
		end_joint->if_SuckObject = 1; // suck success
	else if(end_joint->adc_value <= end_joint->noLoad_bound)
		end_joint->if_SuckObject = 0; // suck failed
	else
		end_joint->if_SuckObject = 2; // pump is close
}


