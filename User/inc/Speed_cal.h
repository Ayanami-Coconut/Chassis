#ifndef __SPEED_CAL_H__
#define __SPEED_CAL_H__

#include "math.h"
#include "main.h"
#include "mathFunc.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3
#define Deg2Rad 3.1415926/180
#define Wheel2Center 20
#define Rad2Deg 180/3.1415926
#define DEG_45_COS 0.707107
#define carVel2RPM 194.883603786f
#define CHASSIS_MAX_VELOCITY 5.f
#define CHASSIS_MAX_ANGULAR_VELOCITY 300
typedef struct Wheel
{
    float V_set;
    float Vx_set;
    float Vy_set;
    float AngleSet_Deg;
    float AngleSet_Rad;
    float cosPhaseAngle;
    float sinPhaseAngle;
}Wheel;
typedef struct chassisPostureSet
{
    float Setvx,Setvy,Setv,SetW;
}chassisPostureSet;

typedef struct chassisPostureReal
{
    float angle,speed;
}chassisPostureReal;
typedef struct Chassis
{
    Wheel wheel[4];
    chassisPostureSet chassispostureset;
    chassisPostureReal chassisposturereal;
}Chassis;

void Chassis_Init(Chassis *chassis);
#endif /* __SPEED_CAL_H__ */
