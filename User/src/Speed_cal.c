#include "Speed_cal.h"
#include "mathFunc.h"
#include "main.h"
extern Chassis chassis;
void Chassis_Init(Chassis *chassis)
{
    *chassis = (Chassis){0};

    chassis->wheel[FL].cosPhaseAngle = -(DEG_45_COS);
    chassis->wheel[FL].sinPhaseAngle = -(DEG_45_COS);

    chassis->wheel[FR].cosPhaseAngle = -(DEG_45_COS);
    chassis->wheel[FR].sinPhaseAngle =  (DEG_45_COS);

    chassis->wheel[BL].cosPhaseAngle =  (DEG_45_COS);
    chassis->wheel[BL].sinPhaseAngle = -(DEG_45_COS);

    chassis->wheel[BL].cosPhaseAngle =  (DEG_45_COS);
    chassis->wheel[BL].sinPhaseAngle =  (DEG_45_COS);

}
int WheelTurnMin(Wheel *wheel, float TargetAngle)
{
    float deltaAngle = TargetAngle * Deg2Rad - wheel->AngleSet_Rad;
    int temp = floor(deltaAngle / Pi * 0.5);
    wheel->AngleSet_Rad += deltaAngle - temp *Pi;

    wheel->AngleSet_Deg = wheel->AngleSet_Rad * Rad2Deg;
    return powf(-1.f,temp);
}
void Single_WheelDeal(Wheel *wheel,float Car_vx,float Car_vy,float Car_w)
{
    wheel->Vx_set = (Car_vx + Deg2Rad * Car_w * Wheel2Center * wheel->cosPhaseAngle) * carVel2RPM;
    wheel->Vy_set = (Car_vy + Deg2Rad * Car_w * Wheel2Center * wheel->sinPhaseAngle) * carVel2RPM;
    wheel->V_set = sqrtf(wheel->Vx_set * wheel->Vx_set + wheel->Vy_set * wheel->Vy_set);
    if(wheel->V_set == 0) return;
    float TargetAngle = atan2(wheel->Vy_set,wheel->Vx_set) * Rad2Deg;
    wheel->V_set *= WheelTurnMin(wheel,TargetAngle);
}
void Speed2Wheel(Chassis *chassis)
{
    chassis->chassispostureset.Setv = sqrt(chassis->chassispostureset.Setvx*chassis->chassispostureset.Setvx+
                                           chassis->chassispostureset.Setvy*chassis->chassispostureset.Setvy                    
                                          );
    if(ABS(chassis->chassispostureset.Setv > CHASSIS_MAX_VELOCITY))
    {
        chassis->chassispostureset.Setvx *= (chassis->chassispostureset.Setv / CHASSIS_MAX_VELOCITY);
        chassis->chassispostureset.Setvy *= (chassis->chassispostureset.Setv / CHASSIS_MAX_VELOCITY);
        chassis->chassispostureset.Setv = CHASSIS_MAX_VELOCITY;
    }
    if(ABS(chassis->chassispostureset.SetW) > CHASSIS_MAX_ANGULAR_VELOCITY)
    {
        chassis->chassispostureset.SetW = GetSign(chassis->chassispostureset.SetW) * CHASSIS_MAX_ANGULAR_VELOCITY;
    }
    float chassisRealAngle = chassis->chassisposturereal.angle * Deg2Rad;
    float Car_vx = chassis->chassispostureset.Setvx * cosf(chassisRealAngle) + chassis->chassispostureset.Setvy * sinf(chassisRealAngle);
    float Car_vy = chassis->chassispostureset.Setvy * cosf(chassisRealAngle) - chassis->chassispostureset.Setvx * sinf(chassisRealAngle);
    for(int i = 0;i < 4 ;i++)
    {
        Single_WheelDeal(chassis->wheel + i,Car_vx,Car_vy,chassis->chassispostureset.SetW);
    }
}
void Crosslock(Chassis *chassis)
{
    WheelTurnMin(&chassis->wheel[FL],-45);
    WheelTurnMin(&chassis->wheel[FR],45);
    wheelTurnmin(&chassis->wheel[BL],45);
    wheelTurnmin(&chassis->wheel[BL],-45);
}
