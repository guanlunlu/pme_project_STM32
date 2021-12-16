#include "crc.h"
#include "communicate.h"
#include "robot.h"
#include "stdlib.h"


extern int p;
extern int previous_p;
extern int v;
extern double omega1;
extern double omega2;
extern double omega3;
extern int    count_time;
static volatile int uart_check = 1;
static volatile int uart_cplt = 0;
extern int rxcpltCount;


void Uart_Transmit(struct ROBOT *robot) {
	robot->communicate.uart.tx[robot->communicate.uart.tx_length] = HAL_CRC_Calculate(&hcrc, (uint32_t*)&robot->communicate.uart.tx[1], robot->communicate.uart.tx_length - 1);
	HAL_UART_Transmit_DMA(robot->communicate.uart.huart,(uint8_t*) robot->communicate.uart.tx, 4 * (robot->communicate.uart.tx_length + 3));
	//8 is for crc and trash
	robot->communicate.uart.trans_count++;

}



short Uart_Crc_Check(struct ROBOT *robot) {
	if (robot->communicate.uart.data_buffer.buf32_t[robot->communicate.uart.rx_length] == (volatile int32_t)HAL_CRC_Calculate(&hcrc, (uint32_t*) &robot->communicate.uart.data_buffer.buf32_t[1], robot->communicate.uart.rx_length - 1))
		return 1; //crc success
	return 0; //crc fail
}

void Uart_RxCplt(struct ROBOT *robot) {

	    rxcpltCount ++;
//		uart_check = 0;
//	    if(robot->communicate.uart.error_count > 1500){
//	    	robot->communicate.uart.error_count = 0;
//	    			robot->communicate.uart.healthy_communication = 0;
//	    			robot->communicate.uart.byteCount = 0;
//	    			robot->communicate.uart.getStarter = 0;
//	    			HAL_UART_AbortReceive(robot->communicate.uart.huart);
////	    			robot->communicate.uart.tx[robot->communicate.uart.tx_length + 1] = 255;
//	    			HAL_UART_Receive_DMA(robot->communicate.uart.huart, (uint8_t*) &robot->communicate.uart.data_buffer.starter_buf, 1);
//	    }
//	    else{
			if(robot->communicate.uart.byteCount < 4){
				robot->communicate.uart.starter_signal[robot->communicate.uart.byteCount ++] = robot->communicate.uart.data_buffer.starter_buf;
			}
			if (!robot->communicate.uart.healthy_communication) {
				if(robot->communicate.uart.getStarter){
					if(robot->communicate.uart.data_buffer.buf32_t[robot->communicate.uart.rx_length] == HAL_CRC_Calculate(&hcrc,(uint32_t *)&robot->communicate.uart.data_buffer.buf32_t[1],robot->communicate.uart.rx_length - 1)){
						robot->communicate.uart.healthy_communication = 1;
						robot->communicate.uart.dataValid = 1;
						double temp[8];
						temp[0] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[1]; //Vx
						temp[1] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[2]; //Vy
						temp[2] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[3]; //Omega
						temp[3] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[4]; // stepper V_L
						temp[4] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[5]; // stepper V_R
						temp[5] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[6]; // claw height
						robot->control.linearVelocity_x  =  temp[0] / 1000.0;
						robot->control.linearVelocity_y  =  temp[1] / 1000.0;
						robot->control.angularVelocity   =  temp[2] / 1000.0;
						// robot->arm_ptr->left.velocity    =  temp[3] / 1000.0;
						// robot->arm_ptr->right.velocity   =  temp[4] / 1000.0;
						robot->arm_ptr->left.tracking_angle = temp[3] / 1000.0;
						robot->arm_ptr->right.tracking_angle = temp[4] / 1000.0;
						robot->end_joint.command = temp[5];
	//					robot->arm_ptr->left.velocity    =  temp[0] / 1000.0;
	//					robot->arm_ptr->right.velocity   =  temp[1] / 1000.0;
	//					if(!robot->pose_init_cplt){
	//						if((int32_t)robot->communicate.uart.data_buffer.buf32_t[3] < 0)
	//							robot->pose_init_cplt = 1;
	//						else{
	//							robot->locate.x = (int32_t)robot->communicate.uart.data_buffer.buf32_t[3];
	//							robot->locate.y = (int32_t)robot->communicate.uart.data_buffer.buf32_t[4];
	//							robot->locate.arc = (int32_t)robot->communicate.uart.data_buffer.buf32_t[5] / 1000.0;
	//						}
	//					}
						HAL_UART_AbortReceive(robot->communicate.uart.huart);
						HAL_UART_Receive_DMA(robot->communicate.uart.huart, (uint8_t*) robot->communicate.uart.data_buffer.buf8_t, 4 * (robot->communicate.uart.rx_length + 1));
					}
					else{
						robot->communicate.uart.byteCount = 0;
						robot->communicate.uart.getStarter = 0;
						robot->communicate.uart.error_count ++;
					}
				}
				else if(!robot->communicate.uart.getStarter && robot->communicate.uart.byteCount == 4){
//					robot->communicate.uart.byteCount ++;
					robot->communicate.uart.starter_number = 0;
					for(int i = 0; i < 4; i ++)
						robot->communicate.uart.starter_number += (uint8_t)robot->communicate.uart.starter_signal[i] << (8 * i);
					if(robot->communicate.uart.starter_number == 0x31){
						robot->communicate.uart.getStarter = 1;
						HAL_UART_AbortReceive(robot->communicate.uart.huart);
						HAL_UART_Receive(robot->communicate.uart.huart, (uint8_t *)robot->communicate.uart.data_buffer.buf8_t, 4 * (robot->communicate.uart.rx_length), 10);
						robot->communicate.uart.crc_value = HAL_CRC_Calculate(&hcrc,(uint32_t *)robot->communicate.uart.data_buffer.buf32_t,robot->communicate.uart.rx_length - 1);
						if(robot->communicate.uart.data_buffer.buf32_t[robot->communicate.uart.rx_length -1] == robot->communicate.uart.crc_value){
							robot->communicate.uart.starter_number = 0x31;
							HAL_UART_AbortReceive(robot->communicate.uart.huart);
							HAL_UART_Receive_DMA(robot->communicate.uart.huart, (uint8_t*) robot->communicate.uart.data_buffer.buf8_t, 4 * (robot->communicate.uart.rx_length + 1));
						}
						else{
							robot->communicate.uart.byteCount = 0;
							robot->communicate.uart.healthy_communication = 0;
							robot->communicate.uart.getStarter = 0;
			    			HAL_UART_AbortReceive(robot->communicate.uart.huart);
			    			HAL_UART_Receive_DMA(robot->communicate.uart.huart, (uint8_t*) &robot->communicate.uart.data_buffer.starter_buf, 1);
						}
					}
					else{
						if(robot->communicate.uart.starter_signal[0] != 0x31){
							robot->communicate.uart.starter_signal[0] = robot->communicate.uart.starter_signal[1];
							robot->communicate.uart.starter_signal[1] = robot->communicate.uart.starter_signal[2];
							robot->communicate.uart.starter_signal[2] = robot->communicate.uart.starter_signal[3];
						}
						robot->communicate.uart.byteCount --;
						robot->communicate.uart.error_count ++;
					}
				}

			}
			else{
				robot->communicate.uart.recev_count ++;
				if (robot->communicate.uart.data_buffer.buf32_t[robot->communicate.uart.rx_length] == HAL_CRC_Calculate(&hcrc, (uint32_t*) &robot->communicate.uart.data_buffer.buf32_t[1], robot->communicate.uart.rx_length - 1)) {
					double temp[8];
					temp[0] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[1]; //Vx
					temp[1] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[2]; //Vy
					temp[2] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[3]; //Omega
					temp[3] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[4]; // stepper V_L
					temp[4] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[5]; // stepper V_R
					temp[5] = (int32_t)robot->communicate.uart.data_buffer.buf32_t[6]; // claw height
					robot->control.linearVelocity_x  =  temp[0] / 1000.0;
					robot->control.linearVelocity_y  =  temp[1] / 1000.0;
					robot->control.angularVelocity   =  temp[2] / 1000.0;
					// robot->arm_ptr->left.velocity    =  temp[3] / 1000.0;
					// robot->arm_ptr->right.velocity   =  temp[4] / 1000.0;
					robot->arm_ptr->left.tracking_angle = temp[3] / 1000.0;
					robot->arm_ptr->right.tracking_angle = temp[4] / 1000.0;
					robot->end_joint.command = temp[5];
	//					robot->arm_ptr->left.velocity    =  temp[0] / 1000.0;
	//					robot->arm_ptr->right.velocity   =  temp[1] / 1000.0;
	//				if(!robot->pose_init_cplt){
	//					if((int32_t)robot->communicate.uart.data_buffer.buf32_t[3] < 0)
	//						robot->pose_init_cplt = 1;
	//					else{
	//						robot->locate.x = (int32_t)robot->communicate.uart.data_buffer.buf32_t[3];
	//						robot->locate.y = (int32_t)robot->communicate.uart.data_buffer.buf32_t[4];
	//						robot->locate.arc = (int32_t)robot->communicate.uart.data_buffer.buf32_t[5] / 1000.0;
	//					}
	//				}
				}
				else{
					robot->communicate.uart.getStarter = 0;
					robot->communicate.uart.healthy_communication = 0;
					HAL_UART_AbortReceive(robot->communicate.uart.huart);
					HAL_UART_Receive_DMA(robot->communicate.uart.huart, (uint8_t*) robot->communicate.uart.data_buffer.starter_buf, 1);
					robot->communicate.uart.byteCount = 0;
					robot->communicate.uart.error_count ++;
				}
			}
//	    }


}


void Tx_Data_Update(struct ROBOT *robot) {

	double omega_1 = omega1 / 1000.0;
	double omega_2 = omega2 / 1000.0;
	double omega_3 = omega3 / 1000.0;


	omega_1 = robot->control.motor[0].omega;
	omega_2 = robot->control.motor[1].omega;
	omega_3 = robot->control.motor[2].omega;

//	omega_1 /= count_time;
//	omega_2 /= count_time;
//	omega_3 /= count_time;
//	omega1 = 0;
//	omega2 = 0;
//	omega3 = 0;
	count_time = 0;
//	double duty1 = 100 * robot->control.motor[0].hardware_info.pwm_timer->Instance->CCR2 / 4500;
//	double duty2 = 100 * robot->control.motor[1].hardware_info.pwm_timer->Instance->CCR1 / 4500;
	int linearVelx  =  ((2*omega_2 - omega_1 - omega_3) * 1000) / 3;    //times 1000 cuz we send a integer
	int linearVely  =  (1.73205 *(omega_3 - omega_1) * 1000) / 3;
	int angularVel =  1000 * ((omega_1 + omega_2 + omega_3) / ((float)(3.0 * DistanceBetweenWheel)));
	robot->communicate.uart.tx[1] = linearVely;
	robot->communicate.uart.tx[2] = -linearVelx;
	robot->communicate.uart.tx[3] = angularVel;
	robot->communicate.uart.tx[4] = robot->arm_ptr->left.angle * 1000;
	robot->communicate.uart.tx[5] = robot->arm_ptr->right.angle * 1000;
//	robot->communicate.uart.tx[4] = 100 * robot->control.motor[2].hardware_info.pwm_timer->Instance->CCR2 / 4500;
	robot->communicate.uart.tx[6] = robot->end_joint.return_state;
//	robot->communicate.uart.tx[6] = robot->communicate.uart.error_count;
//	if(!robot->imu.data_processing){
		robot->communicate.uart.tx[9] = robot->imu.Gz * 1000000;
		robot->communicate.uart.tx[7] = robot->imu.Ax * 1000000;
		robot->communicate.uart.tx[8] = robot->imu.Ay * 1000000;
		robot->communicate.uart.tx[10] = robot->arm_ptr->left.tracking_angle * 1000000.0 + robot->arm_ptr->right.tracking_angle * 1000.0;
		robot->communicate.uart.tx[11] = robot->communicate.uart.error_count;
//		robot->communicate.uart.tx[9] = 0;
//		robot->communicate.uart.tx[7] = 0;
//		robot->communicate.uart.tx[8] = 0;
//	}

//		robot->communicate.uart.tx[10] = robot->communicate.uart.error_count;
	//	robot->communicate.uart.tx[3] = angularVel;
//	robot->communicate.uart.tx[1] = robot->control.motor1.omega;
//	robot->communicate.uart.tx[2] = robot->control.motor2.omega;
//	robot->communicate.uart.tx[3] = robot->locate.x * 1000;
//	robot->communicate.uart.tx[4] = robot->locate.y * 1000;
//	robot->communicate.uart.tx[5] = robot->locate.arc * 1000000;
//	robot->communicate.uart.tx[6] = 10000 * robot->control.motor[0].command + robot->control.motor[1].command + 4000 + 40000000;
//	robot->communicate.uart.tx[7] = uart_check;
//	robot->communicate.uart.tx[8] = 10000 * duty1 + duty2 + 4000 + 40000000;
//	robot->communicate.uart.tx[8] = 10000 * robot->control.motor1.omega + robot->control.motor2.omega + 4000 + 40000000;

	//4__dutyL__4__dutyR
//	if(uart_cplt){
//		uart_check = 0;
//		uart_cplt = 0;
//	}
//	int inA1;	//VNH inputA for wheel1
//	int inB1;
//	int inA2;
//	int inB2;
//	int inA3;
//	int inB3;
//	inA1 = HAL_GPIO_ReadPin(robot->control.motor[0].hardware_info.Cw_port,robot->control.motor[0].hardware_info.Cw_pin);
//	inB1 = HAL_GPIO_ReadPin(robot->control.motor[0].hardware_info.Ccw_port,robot->control.motor[0].hardware_info.Ccw_pin);
//	inA2 = HAL_GPIO_ReadPin(robot->control.motor[1].hardware_info.Cw_port,robot->control.motor[1].hardware_info.Cw_pin);
//	inB2 = HAL_GPIO_ReadPin(robot->control.motor[1].hardware_info.Ccw_port,robot->control.motor[1].hardware_info.Ccw_pin);
//	inA3 = HAL_GPIO_ReadPin(robot->control.motor[2].hardware_info.Cw_port,robot->control.motor[2].hardware_info.Cw_pin);
//	inB3 = HAL_GPIO_ReadPin(robot->control.motor[2].hardware_info.Ccw_port,robot->control.motor[2].hardware_info.Ccw_pin);
//	robot->communicate.uart.tx[9] = 4000000 + 100000 * inA1 + 10000 * inB1 + 10000 * inA2 + 100 * inB2 + 10 *inA3 + inB3;
	if (robot->communicate.uart.tx[robot->communicate.uart.tx_length + 1] == 255) {
		HAL_UART_Receive_DMA(robot->communicate.uart.huart, (uint8_t*) &robot->communicate.uart.data_buffer.starter_buf, 1);
		robot->communicate.uart.tx[robot->communicate.uart.tx_length + 1] = 0;
		robot->communicate.uart.start = 1;
	}


}


