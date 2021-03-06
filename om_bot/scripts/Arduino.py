#!/usr/bin/env python
'''
Heavily borrowed from Dr. Hessmer
'''

#import roslib

import rospy
import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Range
from my_repository.msg import speed_wheel
from my_repository.srv import *

from SerialDataGateway import SerialDataGateway

class Arduino(object):
        '''
        Helper class for communicating with an Arduino board over serial port
        '''
        def _HandleReceivedLine(self,  line):
                self._Counter = self._Counter + 1
                self._speedsimupub.publish(String(str(self._Counter) + " " + line))

                if (len(line) > 0):
                        lineParts = line.split('\t')
                        if (lineParts[0] == 'u'):
                                self._ulsensor(lineParts)
                                return
                        if (lineParts[0] == 'o'):
                                self._BroadcastOdometry(lineParts)
                                return
                        if (lineParts[0] == "InitializeBaseController"):
                                # controller requesting initialization
                                self._InitializeBase()
                                return
                        if (lineParts[0] == "Active"):
                                # controller requesting initialization
                                self._IsActive()
                                return
                        if (lineParts[0] == 'p'):
                                self._printThis(lineParts)
                                return
                        if (lineParts[0] == 'w'):
                                self._sendspeeds(lineParts)
                                return

        def _ulsensor(self, lineParts):
                partsCount = len(lineParts)
                if (partsCount < 2):
                        pass
                try:
                    drange = float(lineParts[1])

                    rangesensor = Range()
                    drange = drange / 100;
                    rangesensor.range = drange
                    rangesensor.header.frame_id = "ul_range"
                    rangesensor.field_of_view = 0.4
                    rangesensor.min_range = 0.01
                    rangesensor.max_range = 1
                    rosNow = rospy.Time.now()
                    rangesensor.header.stamp = rosNow
                    self._RangePublisher.publish(rangesensor)
                except:
                    rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))

        def _printThis(self, lineParts): #Function to receive information from Arduino
                partsCount = len(lineParts)
                for x in range(1, partsCount):
                        rospy.loginfo("Printing: " + lineParts[x])

 
        def _BroadcastOdometry(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 6):
                        pass

                try:
                        x = float(lineParts[1])

                        y = float(lineParts[2])
                        theta = float(lineParts[3])

                        vx = float(lineParts[4]) 
                        vy = float(lineParts[5]) 
                        omega = float(lineParts[6]) 

                        #quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
                        quaternion = Quaternion()
                        quaternion.x = 0.0
                        quaternion.y = 0.0
                        quaternion.z = sin(theta/2.0)
                        quaternion.w = cos(theta/2.0)


                        rosNow = rospy.Time.now()

                        # First, we'll publish the transform from frame odom to frame base_link over tf
                        # Note that sendTransform requires that 'to' is passed in before 'from' while
                        # the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
                        self._OdometryTransformBroadcaster.sendTransform(
                                (x, y, 0),
                                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                                rosNow,
                                "base_link",
                                "odom"
                                )

                        # next, we'll publish the odometry message over ROS
                        odometry = Odometry()
                        odometry.header.frame_id = "odom"
                        odometry.header.stamp = rosNow
                        odometry.pose.pose.position.x = x
                        odometry.pose.pose.position.y = y
                        odometry.pose.pose.position.z = 0.0812779 #just for debugging
                        odometry.pose.pose.orientation = quaternion

                        odometry.child_frame_id = "base_link"
                        odometry.twist.twist.linear.x = vx
                        odometry.twist.twist.linear.y = vy
                        odometry.twist.twist.angular.z = omega

                        self._OdometryPublisher.publish(odometry)

                        #rospy.loginfo(odometry)

                except:
                        rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))

       

        def _sendspeeds(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 3):
                        pass

                try:
                       
			FL = float(lineParts[1])
			#rospy.loginfo("Todo bien")
                        RL = float(lineParts[2])
                        FR = float(lineParts[3]) 
                        rosNow = rospy.Time.now()
                        wheelspeds = speed_wheel()
                        
			wheelspeds.header.frame_id = "base_link"
                        wheelspeds.header.stamp = rosNow
                        
                        wheelspeds.FL = FL
                        wheelspeds.RL = RL
                        wheelspeds.FR = FR
                        


                        
                        
                        
                        self._SpeedsPublisher.publish(wheelspeds)

                        #rospy.loginfo(odometry)

                except:
                        rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))

       

        def _WriteSerial(self, message):
                #self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
                self._SerialDataGateway.Write(message)

        def __init__(self, port="/dev/ttyACM0", baudrate=57600):
                '''
                Initializes the receiver class. 
                port: The serial port to listen to.
                baudrate: Baud rate for the serial communication
                '''
                
                self._Counter = 0

                rospy.init_node('arduino')
              
                port = rospy.get_param("~port", "/dev/ttyACM0")
                baudRate = int(rospy.get_param("~baudRate", 57600))
           

                rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
                self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
                

                # Service for IMU use
                self._ImuService = rospy.Service('Use_Imu', imuactive, self._sendImu)
                # Service for angular and linear correction values
                self._angularservice = rospy.Service('Angular_float_value', angularscale, self._sendang)
                self._linearservice = rospy.Service('Linear_float_value', linearscale, self._sendlin)
                # subscriptions
                rospy.Subscriber("cmd_vel", Twist, self._GetSpeeds, queue_size=10)

                self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)
                self._SpeedsPublisher = rospy.Publisher("wheel_speeds", speed_wheel, queue_size=10)
                self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
                self._RangePublisher = rospy.Publisher("ulrange", Range, queue_size=10)
                self._SerialPublisher = rospy.Publisher('serial', String, queue_size=10)
                self._speedsimupub = rospy.Publisher('speeds', String, queue_size=10)
               

        def Start(self):
                rospy.logdebug("Starting")
                self._SerialDataGateway.Start()
 
        def Stop(self):
                rospy.logdebug("Stopping")
                self._SerialDataGateway.Stop()
                
        def _GetSpeeds(self, twist_command):
                """ Handle movement requests. """
                vx = twist_command.linear.x        # m/s
                vy = twist_command.linear.y        # m/s
                omega = twist_command.angular.z      # rad/s
                #rospy.loginfo("Sending twist command: " + str(vx) + "," + str(vy) + "," + str(omega))

                message = 's %d %d %d %d %d %d\r' % self._GetBaseAndExponents((vx, vy, omega))
                rospy.logdebug("Sending speed command message: " + message)
                self._WriteSerial(message)
                #rospy.loginfo("Sent: " + message)
        def _sendImu(self, request):
                """ Service to enable or disable imu for odometry"""

                rospy.set_param("~imuenable", request.imu_active)
                if request.imu_active:
                   message = 'imu\r'
                   rospy.loginfo("Imu enabled")
                else:
                   message = 'noimu\r'
                   rospy.loginfo("Imu disabled")

                self._WriteSerial(message)
                return imuactiveResponse()

        def _sendang(self, request):
                rospy.set_param("~angular_correction", request.avaluescale)
                message = 'ascale %d %d\r' % self._GetBaseAndExponent(request.avaluescale)
                rospy.loginfo("Sending correction value: " + message)
                self._WriteSerial(message)
                return angularscaleResponse()

        def _sendlin(self, request):
                rospy.set_param("~linear_correction", request.lvaluescale)
                message = 'lscale %d %d\r' % self._GetBaseAndExponent(request.lvaluescale)
                rospy.loginfo("Sending correction value: " + message)
                self._WriteSerial(message)
                return linearscaleResponse()

        def _InitializeBase(self):
                """ Writes an initializing string to start the Base """
                imuen = rospy.get_param("~imuenable", "True")
                if imuen:
                     message = 'Startimu\r'
                else:
                     message = 'Startnoimu\r'

                rospy.loginfo("Initializing Base " + message)
                self._WriteSerial(message)
                
                lincorrection = rospy.get_param("~linear_correction", 1.0)
                angcorrection = rospy.get_param("~angular_correction", 0.984)
                message = 'ascale %d %d\r' % self._GetBaseAndExponent(angcorrection)
                rospy.loginfo("Sending correction value: " + message)
                self._WriteSerial(message)
                message = 'lscale %d %d\r' % self._GetBaseAndExponent(lincorrection)
                rospy.loginfo("Sending correction value: " + message)
                self._WriteSerial(message)

        def _GetBaseAndExponent(self, floatValue, resolution=4):
                '''
                Converts a float into a tuple holding two integers:
                The base, an integer with the number of digits equaling resolution.
                The exponent indicating what the base needs to multiplied with to get
                back the original float value with the specified resolution.
                '''

                if (floatValue == 0.0):
                        return (0, 0)
                else:
                        exponent = int(1.0 + math.log10(abs(floatValue)))
                        multiplier = math.pow(10, resolution - exponent)
                        base = int(floatValue * multiplier)

                        return(base, exponent - resolution)

        def _GetBaseAndExponents(self, floatValues, resolution=4):
                '''
                Converts a list or tuple of floats into a tuple holding two integers for each float:
                The base, an integer with the number of digits equaling resolution.
                The exponent indicating what the base needs to multiplied with to get
                back the original float value with the specified resolution.
                '''

                baseAndExponents = []
                for floatValue in floatValues:
                      baseAndExponent = self._GetBaseAndExponent(floatValue)
                      baseAndExponents.append(baseAndExponent[0])
                      baseAndExponents.append(baseAndExponent[1])

                return tuple(baseAndExponents)

        def _IsActive(self):
                rospy.loginfo("Is Active")


if __name__ == '__main__':
        arduino = Arduino()
        try:
                arduino.Start()
                rospy.spin()

        except rospy.ROSInterruptException:
                arduino.Stop()
