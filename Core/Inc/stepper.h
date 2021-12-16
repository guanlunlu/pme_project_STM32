#ifndef __stepper_H
#define __stepper_H


//#ifdef __cplusplus
//extern "C" {
//#endif

#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <tim.h>
#include "stdint.h"
#define PI 3.1415926

typedef struct hardware{
    GPIO_TypeDef * DIR_port;
    uint16_t DIR_pin;
    
    GPIO_TypeDef * STEP_port;
    uint16_t STEP_pin;

    GPIO_TypeDef * MS1_port;
    uint16_t MS1_pin;
    
    GPIO_TypeDef * MS2_port;
    uint16_t MS2_pin;

    GPIO_TypeDef * switch_port;
    uint16_t switch_pin;

    TIM_HandleTypeDef * pwm_timer;
    unsigned int pwm_channel;

    float init_angle;
}Hardware;


typedef struct pose{
    float x;
    float y;

    void (*init)(struct pose *,float,float);

}Pose;

typedef struct stepper_state{
    struct hardware hardware;
    int id;
    float velocity;
    float tracking_angle;
    int step;
    float angle;
    int direction;
    int initialized;
    float span[2];
    Pose origin_pose;


    void  (*hardware_init)(struct stepper_state* stepper,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,TIM_HandleTypeDef *, unsigned int);
    void  (*param_init)(struct stepper_state *,int id,float x,float y, float init_angle, float min_angle, float max_angle);
    float (*get_angle)(struct stepper_state);
}Stepper_state;


typedef struct linkage{
    Pose origin;
    float length;
    float angle;

    void (*init)(struct linkage *,float x,float y, int length , float angle);
    Pose* (*get_endjoint)(struct linkage);
}Linkage;

typedef struct Scara{
	float cmdvel_x;
	float cmdvel_y;
    struct stepper_state right;
    struct stepper_state left;
    Pose endjoint;
    Linkage link_L1;
    Linkage link_L2;
    Linkage link_R1;
    Linkage link_R2;
    Pose end_joint;

}Scara;



double theta_domain_convert(double a);

Scara* scara_accessor();


void linkage_init(Linkage *link, float x, float y, int length, float angle);
void stepper_hardware_set(Stepper_state* stepper,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,GPIO_TypeDef *,uint16_t,TIM_HandleTypeDef *, unsigned int);
void arm_hardware_set();

void stepper_set();
void arm_initialize();

float stepper_angle_accessor(Stepper_state *);

/**
 * stepper
 * motor_id
 * origin_x
 * origin_y
 * origin_angle
 * min_angle
 * max_angle
 */
void stepper_init(Stepper_state *,int,float,float, float, float,float);

void linkage_init(Linkage *,float,float , int , float);
void scara_init(Scara *);

void stepper_move(Scara *, float, float);

float inverse_kinematic(Stepper_state *, Pose);

void update_linkage_endpose(Scara *);

float get_vec_length(float, float );

void arm_move(Scara *, float , float);

int inspan(Stepper_state *motor, float angle);

float radian(float degree);

float degree(float radian);

void stepper_angle_tracking(Scara *arm, float L_goal, float R_goal);

//#ifdef __cplusplus
//}
//#endif
#endif
