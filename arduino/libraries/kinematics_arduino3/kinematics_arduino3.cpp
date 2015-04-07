#include "Arduino.h"
#include "kinematics_arduino3.h"

//Constructor to set the pointers to the variables
kinematics_arduino3::kinematics_arduino3(double* evx, double* evy, double* evth,
                                       double* eFL, double* eFR, double* eRR,
                                       double* eRL, double* linscale, double* angscale)
{
    vx = evx;
    vy = evy;
    vth = evth;
    FL = eFL;
    FR = eFR;
    RR = eRR;
    RL = eRL;
    lastvx = 0.0;
    lastvy = 0.0;
    lastvth = 0.0;
    lastTime = millis();
    l1 = 0.16;
    l2 = 0.1665;
    dd = 0.245;
    sm = 0.035;
    radius_wheel = 0.052;
    linearscale = linscale;
    angularscale = angscale;
}

double kinematics_arduino3::timemsec()
{
    return lastTime;
}
void kinematics_arduino3::stop_speed()
{
        *FL = 0.0;
        *FR = 0.0;
        *RL = 0.0;
        *RR = 0.0;
}
bool kinematics_arduino3::compute()
{
    lastTime = millis();
    //if(*vx != lastvx || *vy != lastvy || *vth != lastvth )
    //{
        //This function calculates the speeds of the wheels using the inverse kinematics
        //The speed will be given in rad/sec
        //FL = ((linear.x) + (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ;
        //FR = ((linear.x) - (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ;
        //RL = ((linear.x) - (linear.y) - ((l1+l2) * angular.z)) / radius_wheel ;
        //RR = ((linear.x) + (linear.y) + ((l1+l2) * angular.z)) / radius_wheel ;
        *FL = ((*vx / *linearscale) - (*vy / *linearscale) - ((l1+l2) * (*vth / *angularscale))) / radius_wheel ;
        *FR = ((*vx / *linearscale) + (*vy / *linearscale) + ((l1+l2) * (*vth / *angularscale))) / radius_wheel ;
        /*Change*/
        *RL = ((*vy / *linearscale) - (*vx / *linearscale) - ((dd-sm) * (*vth /*/ *angularscale*/))) / radius_wheel ;
        *RR = ((*vx / *linearscale) - (*vy / *linearscale) + ((l1+l2) * (*vth / *angularscale))) / radius_wheel ;

        //The control node will receive the desired speeds in RPM , so it is neccesary to
        //do a units conversion, from rad/sec to RPM , multiplying by a factor of 60 / 2 pi = 9.5496
        *FL = *FL * 9.5496;
        *FR = *FR * 9.5496;
        *RL = *RL * 9.5496;
        *RR = *RR * 9.5496;
        lastvx = *vx;
        lastvy = *vy;
        lastvth = *vth;
        return true;
    //}
   // else return false;
}
