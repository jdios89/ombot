#ifndef kinematics_arduino_H
#define kinematics_arduino_H

#include "Arduino.h"

class kinematics_arduino
{
public:
    double timemsec();
    void stop_speed();
    kinematics_arduino(double*, double*, double*,double*, double*, double*, double*, double*, double*);
    bool compute();
private:
    double secs();
    double *vx;
    double *vy;
    double *vth;
    double *FL;
    double *FR;
    double *RR;
    double *RL;
    double lastvx;
    double lastvy;
    double lastvth;
    unsigned long lastTime;
    //robot dimensions
    double l1;// = 0.16;   //m
    double l2;// = 0.1665; // 0.16637; //m
    double radius_wheel;// = 0.052; //0.05146; //m
    double *linearscale;
    double *angularscale;

};


#endif // KINEMATICS_ARDUINO_H
