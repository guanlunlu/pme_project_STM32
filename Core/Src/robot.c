#include "robot.h"



void Initiate(struct ROBOT *robot){

	ControlInit(robot);
//	LocateInit(robot);
	driving_module_init(robot);



	endjoint_Init(&robot->end_joint);
	endjoint_work(&robot->end_joint, 0, 0);
	robot->end_joint.command = 2;


	robot->arm_ptr = scara_accessor();
	arm_initialize();

	CommunicateInit(&robot->communicate);

	robot->setMotor = setMotorSpeed;
	robot->stopActing = Deinit;
	robot->ready = 0;
	robot->timeCount = 0;
	robot->PastTime = 0;
	robot->pose_init_cplt = 0;
	robot->pmw3901.hspi = &hspi1;
	robot->pmw3901._cs_port = GPIOA;
	robot->pmw3901._cs_pin = GPIO_PIN_4;
	robot->pmw3901.startReq  = true;
	robot->pmw3901.endReg = true;
	robot->pmw3901.getData = 0;
	robot->deltaXSum = 0;
	robot->deltaYSum = 0;
	for(int i = 0; i < 6; i ++){
		robot->imu.Rec_Data[i] = 0;
	}

}


void Deinit(struct ROBOT * robot){
	for(int i = 0; i < 3; i++)
		HAL_TIM_PWM_Stop(robot->control.motor[i].hardware_info.pwm_timer, robot->control.motor[i].hardware_info.pwm_channel);

	HAL_TIM_Base_Stop(&htim6);
	HAL_TIM_Base_Stop(&htim11);
}

void driving_module_init(struct ROBOT * robot){

	wheel_hardware_init(&robot->control.motor[2].hardware_info,GPIOC,GPIO_PIN_13,GPIOE,GPIO_PIN_4,&htim9,TIM_CHANNEL_2,&htim4,99.7,10, -1 ,512 * 4);
	wheel_kinematics_init(&robot->control.motor[2].kinematics,0,0,887500,162500,10,-162500,-10,0,0);

	wheel_hardware_init(&robot->control.motor[0].hardware_info,GPIOE,GPIO_PIN_2,GPIOE,GPIO_PIN_3,&htim9,TIM_CHANNEL_1,&htim3,99.7,10, -1 ,512 * 4);
	wheel_kinematics_init(&robot->control.motor[0].kinematics,0,0,887500,162500,10,-162500,-10,0,0);

	wheel_hardware_init(&robot->control.motor[1].hardware_info,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_13,&htim12,TIM_CHANNEL_1,&htim2,99.7,10, 1 ,512 * 4);
	wheel_kinematics_init(&robot->control.motor[1].kinematics,0,0,887500,162500,10,-162500,-10,0,0);
	for(int i  = 0; i < 3; i ++){
			wheel_init(&robot->control.motor[i]);
			HAL_TIM_Encoder_Start(robot->control.motor[i].hardware_info.encTimer, TIM_CHANNEL_ALL);
			HAL_TIM_PWM_Start(robot->control.motor[i].hardware_info.pwm_timer,robot->control.motor[i].hardware_info.pwm_channel);
	}


}

void wheel_init(struct MOTOR *motor){
	motor->speedMargin = 1;
	motor->counter_value = 0;
	motor->omega = 0;
	motor->delta_counter = 0;
	motor->accumulated_error = 0;
	motor->target = 0;
	motor->command = 0;
	motor->error = 0;
	motor->outputSaturation = 0;
	motor->previous_speed = 0;
	motor->previous_counter_value = 0;
	motor->delta_vel = 0;
	motor->startSaturation = 0;
	motor->startSaturationCount = 0;
	motor->outputCNT = 0;
	motor->outputSaturation = 0;
	motor->previous_counter_value = 0;

}


void wheel_hardware_init(struct  Hardware_Info *hardware_info, GPIO_TypeDef * GPIOx_A, uint16_t GPIO_PIN_INA, GPIO_TypeDef * GPIOx_B, uint16_t GPIO_PIN_INB, TIM_HandleTypeDef * pwm_htim, uint32_t pwm_channel, TIM_HandleTypeDef *enc_htim, double wheelDiameter,double reductionRatio, short encDirection, int encResolution){

	hardware_info->Ccw_port 		=  			GPIOx_B;
	hardware_info->Ccw_pin 		=  			GPIO_PIN_INB;

	hardware_info->Cw_port 		=  			GPIOx_A;
	hardware_info->Cw_pin 		=  			GPIO_PIN_INA;

	hardware_info->pwm_timer 		=  			pwm_htim ;
	hardware_info->pwm_channel    =  			pwm_channel;

	hardware_info->encTimer 		= 			enc_htim;
	hardware_info->encTimer->Instance->CNT = 0;
	hardware_info->encDirection   =   		encDirection;
	hardware_info->encResolution  =   		encResolution;
	hardware_info->ReductionRatio =  		reductionRatio;
	hardware_info->wheelDiameter  =   		wheelDiameter;

}



void wheel_kinematics_init(struct Kinematics *kinematics, double acceleration, double deceleration, double jerk, double max_acc, double min_acc, double max_dec, double min_dec, double threshold_vel1,double threshold_vel2){
	kinematics->acceleration 		= 			acceleration;
	kinematics->deceleration 		= 			deceleration;
	kinematics->jerk 				=    		jerk;
	kinematics->max_acc 			= 			max_acc;
	kinematics->min_acc 			=  			min_acc;
	kinematics->max_dec 			= 			max_dec;
	kinematics->min_dec 			= 			min_dec;
	kinematics->threshold_vel_for_smaller_acc = threshold_vel1;
	kinematics->threshold_vel_for_smaller_dec = threshold_vel2;

}






void ControlInit(struct ROBOT *robot){
//	robot->control.Kp = 18.8;
	robot->control.Kp = 40;
	robot->control.Ki = 234 * 2;
	robot->control.preConstant = 4;
	robot->control.ControlFreq = 1000;
	robot->control.max_PWM_counter = 4500;
	robot->control.counterRecordL = 0;
	robot->control.counterRecordR = 0;
	robot->control.linearVelocity_x = 0;
	robot->control.linearVelocity_y = 0;
	robot->control.angularVelocity = 0;


}

void LocateInit(struct ROBOT *robot){
//	robot->locate.x = 690;
//	robot->locate.y = 2820;
//	robot->locate.deg = 180;
//	robot->locate.expr = 0;
//	robot->locate.arc = 3.14;
//	robot->locate.cosNow = 0;
//	robot->locate.sinNow = 0;
//	robot->locate.round = 0;
//	robot->locate.pathLengthMargin = 0.001;
//	robot->locate.micro_sw[0].sw_port = GPIOC;
//	robot->locate.micro_sw[0].sw_pin = GPIO_PIN_0;
//	robot->locate.micro_sw[1].sw_port = GPIOC;
//	robot->locate.micro_sw[1].sw_pin = GPIO_PIN_1;
//	robot->locate.deg_margin = 3;
}

void UartInit(struct UART * uart){
	uart->huart = &huart3;
	uart->tx_length = 12;
	/**
	 * 0. boardId
	 * 1. Vx
	 * 2. Vy
	 * 3. Omega
	 * 4. stepperL angle
	 * 5. stepperR angle
	 * 6. suck success or not
	 * 7. imu X
	 * 8. imu y
	 * 9. gyro Z
	 * 10. communication error
	 */
//	uart->rx_length = 8;
	uart->rx_length = 7;
	/*
	 * 0. boardID
	 * 1. Vx
	 * 2. Vy
	 * 3. Omega
	 * 4. left stepper vel
	 * 5. right stepper vel
	 * 6. claw target height， suck or not
	 *
	 *
	 * when counting length we didn't take CRC key into consideration
	 * therefore the actual length should be rx_length + 1
	 */

	uart->dataValid = 0;
	uart->start = 1;
	uart->recev_count = 0;
	uart->trans_count = 0;
	uart->error_count = 0;
	uart->byteCount = 0;
	uart->healthy_communication = 0;
//	memset(uart->buf,0,100);
//	memset(uart->data_buffer.buf8_t,0,100);
//	memset(uart->data_buffer.buf32_t,0,15);
	uart->UartFreq = 200;
	for (int count = 0; count < 20; count++){
		uart->tx[count] = 0;
	}
	uart->tx[0] = BoardID;
	uart->tx[uart->tx_length + 2] = 0x3F3E0000;
	uart->starter_number = 0;
	uart->byteCount = 0;
	uart->crc_value = 0;
	uart->dataValid = false;
	uart->error_count = 0;
	uart->data_buffer.starter_buf = 0;
	for(int i = 0; i < 100; i++){
		uart->data_buffer.buf8_t[i] = 0;
	}

	for(int i = 0; i < 15; i ++){
		uart->data_buffer.buf32_t[i] = 0;
	}
	HAL_UART_Receive_DMA(uart->huart, (uint8_t*) &uart->data_buffer.starter_buf, 1);

}

void CommunicateInit(struct Communicate * communicate){
	UartInit(&communicate->uart);
}


void setMotorSpeed(struct MOTOR * motor, bool Orientation, int speed){
	HAL_GPIO_WritePin(motor->hardware_info.Cw_port,motor->hardware_info.Cw_pin,Orientation);
	HAL_GPIO_WritePin(motor->hardware_info.Ccw_port,motor->hardware_info.Ccw_pin,(int)(!(bool)Orientation));
	__HAL_TIM_SetCompare(motor->hardware_info.pwm_timer,motor->hardware_info.pwm_channel,speed);
}


void SysInput(struct MOTOR * motor ,int InputType, int timeCount){
	switch(InputType){
		case 0:
			setMotorSpeed(motor, 1, 4500);
			break;
		case 1:
			if(timeCount <= 4500){
				setMotorSpeed(motor, 1, timeCount);
			}
			else{
				setMotorSpeed(motor, 1, 4500);
			}
			break;
		case 2:
			if(pow(timeCount,2) <= 4500){
				setMotorSpeed(motor, 1, pow(timeCount,2));
			}
			else{
				setMotorSpeed(motor, 1, 4500);
			}
			break;
		default:
			break;
	}
}



void setDegree(struct ROBOT * robot, double targetDegree){
//	 robot->locate.deg_error = targetDegree - robot->locate.deg;
//	 if(robot->locate.deg_error < 0)
//		robot->locate.deg_error += 360;
//	 double max_speed = 100;
//	 double process;
//
//	while((fabs(targetDegree - robot->locate.deg) > 2)){
////		External_Locating(robot);
//		robot->locate.deg_error = targetDegree - robot->locate.deg;
//		if(robot->locate.deg_error < 0){
//			robot->locate.deg_error += 360;
//		}
//		process = (fabs(targetDegree - robot->locate.deg) / 360);
//		if(robot->locate.deg_error > robot->locate.deg_margin && robot->locate.deg_error <= 180){
//			robot->control.motor1.target =  -(max_speed * process + 20);
//			robot->control.motor2.target =   (max_speed * process + 20);
//		}
//		else if(robot->locate.deg_error < 360 - robot->locate.deg_margin && robot->locate.deg_error > 180){
//			robot->control.motor1.target =   (max_speed * process + 20);
//			robot->control.motor2.target =  -(max_speed * process + 20);
//		}
//
//	}
	for(int i = 0; i < 3; i++)
		robot->control.motor[i].target = 0;
}



void wheel_calibration(struct ROBOT * robot, int spin_or_move_forward,int speed){
//	double target;
//	if(robot->yellow_purple){
//		setDegree(robot,90);
//		target = robot->locate.y + 1000;
//	}
//	else{
//		setDegree(robot,270);
//		target = robot->locate.y - 1000;
//	}
//	switch (spin_or_move_forward){
//	case 1:
//		while(1){
//			robot->control.motor1.target = speed;
//			robot->control.motor2.target = -speed;
//		}
//		break;
//	case 0:
//		while(fabs(robot->locate.y - target) > 0.1){
//			robot->control.motor1.target = speed;
//			robot->control.motor2.target = speed;
//		}
//		robot->control.motor1.target = 0;
//		robot->control.motor2.target = 0;
//		break;
//	default: break;
//	}
}
