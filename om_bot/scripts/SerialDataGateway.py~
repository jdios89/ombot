#!/usr/bin/env python
'''
Created on November 20, 2010

@author: Dr. Rainer Hessmer
'''
import threading
import serial
from cStringIO import StringIO
import time
import rospy

def _OnLineReceived(line):
        print(line)


class SerialDataGateway(object):
        '''
        Helper class for receiving lines from a serial port
        '''

        def __init__(self, port="/dev/ttyACM0", baudrate=57600, lineHandler = _OnLineReceived):
                '''
                Initializes the receiver class. 
                port: The serial port to listen to.
                receivedLineHandler: The function to call when a line was received.
                '''
                self._Port = port
                self._Baudrate = baudrate
                self.ReceivedLineHandler = lineHandler
                self._KeepRunning = False
                self._Counter = 11

        def Start(self):
                self._Serial = serial.Serial(port = self._Port, baudrate = self._Baudrate, timeout = 1)

                self._KeepRunning = True
                self._ReceiverThread = threading.Thread(target=self._Listen)
                self._ReceiverThread.setDaemon(True)
                self._ReceiverThread.start()

        def Stop(self):
                rospy.loginfo("Stopping serial gateway")
                self._KeepRunning = False
                time.sleep(.1)
                self._Serial.close()

        def _Listen(self):
                stringIO = StringIO()
                while self._KeepRunning:
                        data = self._Serial.read()
                        if data == '\r' : #carriage /r
                                pass # self._Counter = self._Counter + 1                       
                        if data == '\n' : #new line /n
                                self.ReceivedLineHandler(stringIO.getvalue())
                                stringIO.close()
                                stringIO = StringIO()
                               # info = "Received " + str(self._Counter)
                               # rospy.loginfo(info)
                               # self._Counter = 0
                        else:
                                stringIO.write(data)
                               # self._Counter = self._Counter +1
                                

        def Write(self, data):
               # info = "Writing to serial port: %s" %data
               # rospy.loginfo(info)
                self._Serial.write(data)

        if __name__ == '__main__':
                dataReceiver = SerialDataGateway("/dev/ttyACM0",  57600)
                dataReceiver.Start()

                raw_input("Hit <Enter> to end.")
                dataReceiver.Stop()
