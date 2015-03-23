/****************PI control for each wheel
*************************with different gains
*************the units used in this node are 
************in revolutions per minute (rpm) *****
*******************************/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <string>

double FLinput = 0, FRinput = 0, RRinput =0, RLinput=0;
double FLsetpoint =0, FRsetpoint =0, RRsetpoint =0, RLsetpoint =0;
double ckpfl = 0.4 , ckifl = 0.1 ;
double akpfl = ckpfl , akifl = 3.9578 ; //5.5 times ckifl
double ckpfr = 0.4 , ckifr = 0.5546 ;
double akpfr = ckpfr , akifr = 3.0503;
double ckprl = 0.4 , ckirl = 0.9095 ;
double akprl = ckprl , akirl = 5.00225;
double ckprr = 0.4 , ckirr = 0.9035 ;
double akprr = ckprr , akirr = 4.96925;
double outputfl = 0, outputfr =0, outputrr =0, outputrl=0;
double error1, error2, error3, error4;
double ItermFR=0, ItermFL=0, ItermRL=0, ItermRR=0;
double kpfl, kpfr, kprr, kprl;
double kifl, kifr, kirr, kirl;
double outputfll=0;
double outputrrl=0;
double outputrll=0;
double outputfrl=0;
double time_diff = 0;
double last_time = 0;
int signo = 0;
double timea = 0.7;   //variable of acceleration limits
double max_acc =250/timea;//250 / timea 190/timea;//140/timea //68 / timea; //25 rpm difference in timea seconds
double rec_time = 0; //last time of received commands
double outmax;
double outmin;

char output[4];
std::string str, str2;
int freqq = 30;
float tfreqq = 1 / ((float)freqq);

void getinspeeds(const std_msgs::String::ConstPtr& msg)
{
	int temp;
	str = msg->data.c_str();
	temp = ((int)((unsigned char)str.at(0)));
	RRinput = (float)temp - 120;   //RR
	temp = ((int)((unsigned char)str.at(1)));
	RLinput = (float)temp - 120;   //RL
	temp = ((int)((unsigned char)str.at(2)));
	FRinput = (float)temp - 120;    //FR
	temp = ((int)((unsigned char)str.at(3)));
	FLinput = (float)temp - 120;  //FL

	if(abs(FLinput) <=2)
	{
		FLinput = 0;
	}
	if(abs(FRinput) <=2)
	{
		FRinput = 0;
	}
	if(abs(RRinput) <=2)
	{
		RRinput = 0;
	}
	if(abs(RLinput) <=2)
	{
		RLinput = 0;
	}

}

void getindesiredSpeeds(const std_msgs::StringConstPtr& msg2)
{
	int temp2= 0;
	str2 = msg2->data.c_str();
	temp2 = ((int)((unsigned char)str2.at(0)));
	FLsetpoint = (float)temp2 ;
	temp2 = ((int)((unsigned char)str2.at(1)));
	FRsetpoint = (float)temp2;
	temp2 = ((int)((unsigned char)str2.at(2)));
	RRsetpoint = (float)temp2;
	temp2 = ((int)((unsigned char)str2.at(3)));
	RLsetpoint = (float)temp2;

	FLsetpoint = FLsetpoint - 101; 
	FRsetpoint = FRsetpoint - 101;
	RRsetpoint = RRsetpoint - 101;
	RLsetpoint = RLsetpoint - 101;
	rec_time = ros::Time::now().toSec();
	
}
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SpeedControl");
	ros::NodeHandle n;
        ros::NodeHandle nh("~");
                
        n.param("max_output", outmax, 75.0);
        n.param("min_output", outmin, -75.0); 
        
	ros::Subscriber sub = n.subscribe("speeds", 10, getinspeeds);    //Get the current speeds
	ros::Subscriber sub2 = n.subscribe("desSpeed", 10, getindesiredSpeeds); //Get the desired speeds
        ros::Publisher speeds = n.advertise<std_msgs::String>("cmd_motors", 5); //Publish the commands for the motors
	ros::Publisher speeds2 = n.advertise<geometry_msgs::Twist>("input_speeds", 1);
        ros::Publisher speeds3 = n.advertise<geometry_msgs::Twist>("setpoint_speeds",1);
        ros::Publisher speeds4 = n.advertise<geometry_msgs::Twist>("output_speeds",1);
	ros::Rate loop_rate(freqq);
	while (ros::ok())
	{
                n.param("max_output", outmax, 75.0);
                n.param("min_output", outmin, -75.0);	
		if(ros::Time::now().toSec() > rec_time + 1)
		{
			FLsetpoint = 0;
			FRsetpoint = 0;
			RRsetpoint = 0;
			RLsetpoint = 0;
		}
		//Proportional values of speed 
		outputfl = FLsetpoint * 90 / 105;
		outputfr = FRsetpoint * 90 / 105;
		outputrr = RRsetpoint * 90 / 105;
		outputrl = RLsetpoint * 90 / 105;
				
		/////calculate max acc allowed/////
                time_diff = ros::Time::now().toSec() - last_time ;
                if( abs(outputfr) < 40){} 
		else if( abs( (outputfr-outputfrl) / time_diff) > max_acc)
		{
			if( outputfr == outputfrl)
			{
				signo = 0;
			}
			else if (outputfr > outputfrl)
			{
				signo = 1;
			}
			else 
			{
				signo = -1;
			}	
				
			outputfr = outputfrl + signo * max_acc * time_diff;
		}
                if( abs(outputfl) < 40){}
		else if( abs( (outputfl-outputfll) / time_diff) > max_acc)
		{
			if( outputfl == outputfll)
			{
				signo = 0;
			}
			else if (outputfl > outputfll)
			{
				signo = 1;
			}
			else 
			{
				signo = -1;
			}
			outputfl = outputfll + signo * max_acc * time_diff;
		}
                if( abs(outputrr) < 40){} 
		else if( abs( (outputrr-outputrrl) / time_diff) > max_acc)
		{
			if( outputrr == outputrrl)
			{
				signo = 0;
			}
			else if (outputrr > outputrrl)
			{
				signo = 1;
			}
			else 
			{
				signo = -1;
			}
			outputrr = outputrrl + signo * max_acc * time_diff;
		}
                if( abs(outputrl) < 40){}
		else if( abs( (outputrl-outputrll) / time_diff) > max_acc)
		{
			if( outputrl == outputrll)
			{
				signo = 0;
			}
			else if (outputrl > outputrll)
			{
				signo = 1;
			}
			else 
			{
				signo = -1;
			}
			outputrl = outputrll + signo * max_acc * time_diff;
		}
		

		//setting maximum outputs
		if(outputrr > outmax) outputrr = outmax;
		else if(outputrr < outmin) outputrr = outmin;
		
		if(outputfr > outmax) outputfr = outmax;
		else if(outputfr < outmin) outputfr = outmin;
		
		if(outputfl > outmax) outputfl = outmax;
		else if(outputfl < outmin) outputfl = outmin;

		if(outputrl > outmax) outputrl = outmax;
		else if(outputrl < outmin) outputrl = outmin;
		//Setting zeros
		if(FLsetpoint == 0)
		{
			outputfl = 0;
		}
		if(FRsetpoint == 0)
		{
			outputfr = 0;
		}
		if(RRsetpoint == 0)
		{
			outputrr = 0;
		}
		if(RLsetpoint == 0)
		{
			outputrl = 0;
		}
		/////////////////////////////////////////////
		/////////////Applying offset/////////////////
		geometry_msgs::Twist mytwist,mytwist2,mytwist3;
		mytwist.linear.x = FLsetpoint;
		mytwist.linear.y = FRsetpoint;
		mytwist.linear.z = RLsetpoint;
		mytwist.angular.x = RRsetpoint;
                mytwist2.linear.x = outputfl;
                mytwist2.linear.y = outputfr;
                mytwist2.linear.z = outputrl;
                mytwist2.angular.x = outputrr;
                mytwist3.linear.x = FLinput;
                mytwist3.linear.y = FRinput;
                mytwist3.linear.z = RLinput;
                mytwist3.angular.x = RRinput;
		outputfll = outputfl;
		outputfrl = outputfr;		
		outputrll = outputrl;
		outputrrl = outputrr;
		
		outputfl = outputfl + 91; //avoid writing the null character
		outputfr = outputfr + 91;
		outputrl = outputrl + 91;
		outputrr = outputrr + 91;
		////////Set into ccspeeds ///////////////////
		output[0] = (char)((int)outputfl);
		output[1] = (char)((int)outputfr);
		output[2] = (char)((int)outputrl);
		output[3] = (char)((int)outputrr);
		/////////////////////////////////////////////
		std_msgs::String ccspeeds;
		
		ccspeeds.data = output;
		speeds.publish(ccspeeds);
		speeds2.publish(mytwist3); //Input speeds
                speeds3.publish(mytwist); //Setpoint speeds
                speeds4.publish(mytwist2); //output for debugging
		last_time = ros::Time::now().toSec();
		ros::spinOnce();
		loop_rate.sleep();
	}	

	return 0;
}
