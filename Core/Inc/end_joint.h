#ifndef INC_END_JOINT_H_
#define INC_END_JOINT_H_



#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "adc.h"

typedef enum state state;
enum state{
	downward, //moving downward
	upward,   //moving upward
	hold,     //stay at top and holding cube
	stay,     //stay at top but no holding cube
	waiting   //waiting at downpose
};

struct joint_hardware_info{
	// Switch
    GPIO_TypeDef * SW_port;
    uint16_t SW_pin;

    // Servo pwm output
    TIM_TypeDef * PWM_TIM_idx;
    TIM_HandleTypeDef * PWM_TIM;
    unsigned int CHANNEL;
	__IO uint32_t CHANNEL_REGISTER;

};

struct sucker_hardware_info{
	// Air pump (VNH)
	TIM_HandleTypeDef * PWM_TIM;
	unsigned int PWM_CHANNEL;
	GPIO_TypeDef *suck_port;
	uint16_t suck_pin;

	GPIO_TypeDef * CW_port;
	uint16_t CW_pin;
	GPIO_TypeDef * CCW_port;
	uint16_t CCW_pin;

	// Relay
	GPIO_TypeDef * Relay_port;
	uint16_t Relay_pin;

	// Pressure Sensor
	ADC_HandleTypeDef * Psensor_adc;
};

struct End_Joint{
	struct joint_hardware_info joint_hardware;
	struct sucker_hardware_info sucker_hardware;
	bool init;

	int last_command;
	int command;
	double _downpose;
	state work_state;
	uint32_t stay_start;
	uint32_t stay_cur;

	int return_state; // return to ros - mission success or not

	//servo
	double joint_length; //cm
	double real_init_height;
	double real_init_angle;
	int speed;
	double compensate;

	double cur_height; //cm
	double cur_angle; //rad
	int cur_duty;

	double desire_height;
	double desire_angle;
	int desire_duty;
	int direction;

	//sucker
	int suck_duty;
	bool sucker_state;  // for ROS signal
	int if_SuckObject;  // check if suck the object 0->false 1->true 2->pump is close
	float noLoad_bound; // pump is open but no load
	float Load_bound;   // pump is open and load
	uint32_t adc_value; // ADC read

	//for check and debug
	int state;
	int work;

};

void endjoint_Init(struct End_Joint *end_joint);
void endjoint_hardware_set(struct End_Joint *end_joint);
void servo_reset_pose(struct End_Joint *end_joint);
void sucker_reset(struct End_Joint *end_joint);

void endjoint_script(struct End_Joint *end_joint, int command);
void endjoint_work(struct End_Joint *end_joint, double desire_height, bool sucker_active);
void servo_move(struct End_Joint *end_joint);
void suck_check(struct End_Joint *end_joint);



#endif /* INC_END_JOINT_H_ */
