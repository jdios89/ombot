#ifndef odometer_H
#define odometer_H

class Odometer
{
public:
    Odometer(double*, double*, double*, double*, float*, double*, double*);

    void update();
    void useImu();
    void noImu();
    double x;
    double y;
    double th;
    double vx;
    double vy;
    double w;
    bool use_imu;
private:
    double *rFL;
    double *rFR;
    double *rRR;
    double *rRL;
    float  *yaw; //Is reversed in my particular case
    double offset;
    unsigned long lastTime;
    double secs;
    double l1;// = 0.16;   //m
    double l2;// = 0.1665; // 0.16637; //m
    double radius_wheel;// = 0.052; //0.05146; //m
    double quarter_radiuswheel;
    double lsum;
    double _th; // last th value
    double alpha; //low pass filter value
    double *linearscale;
    double *angularscale;
};

#endif // ODOMETER_H
