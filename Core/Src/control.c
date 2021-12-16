#include "robot.h"
#include "main.h"
#include "tim.h"


void PID(struct ROBOT *robot) {
	for(int i = 0; i < 3; i++){
		robot->control.motor[i].previous_counter_value = robot->control.motor[i].counter_value;
		robot->control.motor[i].counter_value = __HAL_TIM_GetCounter(robot->control.motor[i].hardware_info.encTimer)
				* robot->control.motor[i].hardware_info.encDirection;
		robot->control.motor[i].delta_counter = (double) (robot->control.motor[i].counter_value - robot->control.motor[i].previous_counter_value);

		if (robot->control.motor[i].delta_counter < -32768)   //32768 = 65535 / 2
			robot->control.motor[i].omega = (65536 + robot->control.motor[i].delta_counter) / 0.001; //cw  overflow
		else if (robot->control.motor[i].delta_counter > 32768)
			robot->control.motor[i].omega = (robot->control.motor[i].delta_counter - 65536) / 0.001; //ccw  overflow
		else
			robot->control.motor[i].omega = robot->control.motor[i].delta_counter / 0.001;
	//count per second


		robot->control.motor[i].omega /= (robot->control.motor[i].hardware_info.encResolution
				* robot->control.motor[i].hardware_info.ReductionRatio);
	//rps

		robot->control.motor[i].omega *= (M_PI * robot->control.motor[i].hardware_info.wheelDiameter);

	//mm/s

		robot->control.motor[i].error = robot->control.motor[i].target - robot->control.motor[i].omega;


		if (robot->control.motor[i].outputSaturation == 0)
			robot->control.motor[i].accumulated_error += robot->control.motor[i].error * 0.001;
	//線下面積

		robot->control.controller_output[i] = robot->control.motor[i].error * robot->control.Kp + robot->control.Ki * robot->control.motor[i].accumulated_error;


		robot->control.motor[i].outputCNT = abs(robot->control.controller_output[i]);


		if (robot->control.controller_output[i] <= 0) {
			HAL_GPIO_WritePin(robot->control.motor[i].hardware_info.Cw_port, robot->control.motor[i].hardware_info.Cw_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(robot->control.motor[i].hardware_info.Ccw_port, robot->control.motor[i].hardware_info.Ccw_pin, GPIO_PIN_SET);
		} else if (robot->control.controller_output[i] > 0) {
			HAL_GPIO_WritePin(robot->control.motor[i].hardware_info.Cw_port, robot->control.motor[i].hardware_info.Cw_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(robot->control.motor[i].hardware_info.Ccw_port, robot->control.motor[i].hardware_info.Ccw_pin, GPIO_PIN_RESET);
		}



		if (robot->control.motor[i].outputCNT >= robot->control.max_PWM_counter){
			robot->control.motor[i].outputCNT = robot->control.max_PWM_counter;
			robot->control.motor[i].outputSaturation = 1;
			robot->control.motor[i].accumulated_error -= robot->control.motor[i].error * 0.001;
			if(!robot->control.motor[i].startSaturation){
				robot->control.motor[i].startSaturation = 1;
				robot->control.motor[i].startSaturationCount = robot->timeCount;
			}
		}
		else{
			robot->control.motor[i].outputSaturation = 0;
			robot->control.motor[i].startSaturation = 0;
		}
	}
//
//
//
	for(int i = 0; i < 3; i++)
		__HAL_TIM_SET_COMPARE(robot->control.motor[i].hardware_info.pwm_timer, robot->control.motor[i].hardware_info.pwm_channel, robot->control.motor[i].outputCNT);


}

void interpolation(struct ROBOT *robot, double linearV, double angularV) {



//	double command1, command2;
//	command1 = (linearV) - (angularV * 0.5 * DistanceBetweenWheel);
//	command2 = (linearV) + (angularV * 0.5 * DistanceBetweenWheel);
//	if(fabs(command1- robot->control.motor1.command ) > robot->control.motor1.speedMargin)
//		robot->control.motor1.command = command1;
//
//	if(fabs(command2- robot->control.motor2.command ) > robot->control.motor2.speedMargin)
//		robot->control.motor2.command = command2;
//
//	int ControlCommandRatio = robot->control.ControlFreq / robot->communicate.uart.UartFreq;
//	if(ControlCommandRatio == 0)
//		ControlCommandRatio = 1;
//	if(fabs(command1 - robot->control.motor1.omega) >  robot->control.motor1.speedMargin){
//		robot->control.motor1.target = robot->control.motor1.omega;
//
//		robot->control.motor1.kinematics.acceleration = (command1 - robot->control.motor1.omega) / ControlCommandRatio;
//		robot->control.motor1.target += robot->control.motor1.kinematics.acceleration;
//	}
//	else if(fabs(command1 - robot->control.motor1.omega) <=  robot->control.motor1.speedMargin ){
//
//		robot->control.motor1.kinematics.acceleration = 0;
//	}
//
//	if(fabs(command2 - robot->control.motor2.omega) >  robot->control.motor2.speedMargin){
//		robot->control.motor2.target = robot->control.motor2.omega;
//		robot->control.motor2.kinematics.acceleration = (command2 - robot->control.motor2.omega) / ControlCommandRatio;
//		robot->control.motor2.target += robot->control.motor2.kinematics.acceleration;
//	}
//	else if(fabs(command2 - robot->control.motor2.omega) <= robot->control.motor2.speedMargin){
//
//		robot->control.motor2.kinematics.acceleration = 0;
//
//
//	}





}

void feedForward(struct ROBOT *robot) {
	for(int i = 0; i < 3; i++)
		robot->control.motor[i].outputCNT += robot->control.motor[i].target * robot->control.preConstant;
}


void s_curve(struct ROBOT * robot,double linearVx,double linearVy, double angularV){

	double command[3];
	double rotating_diameter = DistanceBetweenWheel / 1000.0;  //convert to meter
	double vx = -linearVy;
	double vy = linearVx;
	command[0] = (-0.5 * vx) - ((1.73205 / 2) * vy) + (angularV * rotating_diameter);
	command[1] = vx + (angularV * rotating_diameter);
	command[2] = (-0.5 * vx) + ((1.73205 / 2) * vy) + (angularV * rotating_diameter);
//	robot->control.linearVelocity_x = linearVx;
//	robot->control.linearVelocity_y = linearVy;
//	robot->control.angularVelocity = angularV;
	for(int i = 0; i < 3; i++){
		robot->control.motor[i].target = command[i] * 1000;
	}
//	for(int i = 0; i < 3; i ++){
//		command[0] *= 1000;
//		robot->control.motor[i].target = command[i];
//		robot->control.motor[i].delta_vel = (robot->control.motor[i].omega - robot->control.motor[i].previous_speed) * (1 / robot->control.ControlFreq);
//		if(fabs(command[i]- robot->control.motor[i].command ) > robot->control.motor[i].speedMargin){
//			robot->control.motor[i].command = command[i];
//			robot->control.motor[i].kinematics.acceleration = 0;
//			robot->control.motor[i].kinematics.deceleration = 0;
//		}
//
//		robot->control.motor[i].kinematics.threshold_vel_for_smaller_acc = pow(robot->control.motor[i].delta_vel,2) / (2 * robot->control.motor[i].kinematics.jerk);
//		robot->control.motor[i].kinematics.threshold_vel_for_smaller_dec = pow(robot->control.motor[i].delta_vel,2) / (2 * robot->control.motor[i].kinematics.jerk);
//
//		if(fabs(command[i] - robot->control.motor[i].omega) > 0.1){
//			if(command[i] >  robot->control.motor[i].omega){
//				robot->control.motor[i].kinematics.deceleration = 0;
//				robot->control.motor[i].kinematics.acceleration += robot->control.motor[i].kinematics.jerk * (1/ robot->control.ControlFreq);
//				if(fabs(command[i] - robot->control.motor[i].omega) < robot->control.motor[i].kinematics.threshold_vel_for_smaller_acc){
//					robot->control.motor[i].kinematics.acceleration -= pow(robot->control.motor[i].delta_vel,2) / (2 * (robot->control.motor[i].command - robot->control.motor[i].omega)) * (1/robot->control.ControlFreq);
//					if(robot->control.motor[i].kinematics.acceleration < robot->control.motor[i].kinematics.min_acc)
//						robot->control.motor[i].kinematics.acceleration = robot->control.motor[i].kinematics.min_acc;
//				}
//				if(robot->control.motor[i].kinematics.acceleration > robot->control.motor[i].kinematics.max_acc)
//					robot->control.motor[i].kinematics.acceleration = robot->control.motor[i].kinematics.max_acc;
//
//				robot->control.motor[i].target = robot->control.motor[i].omega + robot->control.motor[i].kinematics.acceleration * (1/ robot->control.ControlFreq);
//			}
//			else if(command[i] < robot->control.motor[i].omega){
//				robot->control.motor[i].kinematics.acceleration = 0;
//				robot->control.motor[i].kinematics.deceleration -= robot->control.motor[i].kinematics.jerk * (1/ robot->control.ControlFreq);
//				if(fabs(command[i] - robot->control.motor[i].omega) < robot->control.motor[i].kinematics.threshold_vel_for_smaller_dec){
//					robot->control.motor[i].kinematics.deceleration += pow(robot->control.motor[i].delta_vel,2) / (2 * (robot->control.motor[i].omega - robot->control.motor[i].command)) * (1/robot->control.ControlFreq);
//					if(robot->control.motor[i].kinematics.min_dec < robot->control.motor[i].kinematics.deceleration)
//						robot->control.motor[i].kinematics.deceleration = robot->control.motor[i].kinematics.min_dec;
//				}
//				if(robot->control.motor[i].kinematics.deceleration < robot->control.motor[i].kinematics.max_dec)
//					robot->control.motor[i].kinematics.deceleration = robot->control.motor[i].kinematics.max_dec;
//				robot->control.motor[i].target =robot->control.motor[i].omega + robot->control.motor[i].kinematics.deceleration * (1/ robot->control.ControlFreq);
//
//			}
//		}
//		else{
//			robot->control.motor[i].target = command[i];
//		}
//
//		robot->control.motor[i].previous_speed = robot->control.motor[i].omega;


}
