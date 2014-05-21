#!/usr/bin/env python
import sys
import rospy
import logging
from roskomodo.msg import RegistrationLogger
from roskomodo.msg import LaunchLogger
import registration_logger

#Custom logging handler to publish topic/node registration/unregistration
class registration_handler(logging.Handler):
    
    def __init__(self):
        logging.Handler.__init__(self)
        self.parser = registration_parser()
        self.loggingList = list()
        self.samples_seen = 0
    def registerNode(self, logType):
        if logType == 'rosmaster':
            self.pub = rospy.Publisher('registration_logger', RegistrationLogger)
        if logType == 'roslaunch':
            self.pub = rospy.Publisher('launch_logger', LaunchLogger)
        rospy.init_node('reg_logger', log_level=rospy.DEBUG, disable_rosout=True, disable_signals=True, anonymous=True)     
    def emit(self, record):
        line = self.format(record)
        rl = self.parser.parse_logging(line, record)
        if rl is not None:
            self.samples_seen += 1
            if self.pub.get_num_connections() == 0:
                #If there are no subscribers yet, store data for later publication to roskomodo. 
                #If ~10 samples have been stored, it is safe to say roskomodo will not be ran, stop storing to save memory.
                if self.samples_seen < 10:
                    self.loggingList.append(rl)
                return
            self.pub.publish(rl)

            #If there is data to send, send it now, since we now know there is something subscribed. This is to help avoid the problem of
            #nodes not having their subscriptions registered by the time registration_handler starts publishing.     
            if len(self.loggingList) != 0:
                for ele in self.loggingList:
                    self.pub.publish(ele)
                self.loggingList = []
                return

#Global addLogger access. registration_logger's probably isn't needed. TODO.
def addLogger(loggerName):
        h = registration_handler()
        h.registerNode(loggerName)
        rospy.logdebug("ADDING LOGGER: " + loggerName)
        logging.getLogger(loggerName).addHandler(h)


class registration_parser(object):
    def __init__(self):
        self.processToNodeName = dict()
        self.options = {"-PUB" : self.unregisterPublisher,
        "+PUB" : self.registerPublisher,
        "-SUB" : self.unregisterSubscriber,
        "+SUB" : self.registerSubscriber,
        "-SERVICE" : self.unregisterService,
        "+SERVICE" : self.registerService,
        "create_node_process" : self.launchNode,
        "env[" : self.parseEnvironment,
        "... successfully launched" : self.successLaunch,
        "process has died" : self.crashedProcess,
        "killing os process" : self.killingProcess}

    def unregisterPublisher(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Publisher"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 0
        return msg

    def registerPublisher(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Publisher"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 1
        return msg

    def unregisterSubscriber(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Subscriber"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 0
        return msg

    def registerSubscriber(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Subscriber"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 1
        return msg

    def unregisterService(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Service"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 0
        return msg

    def registerService(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Service"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 1
        return msg

    def launchNode(self, record):
        self.logMsg = LaunchLogger()
        self.logMsg.package = record.args[0]
        self.logMsg.node_name = record.args[1]
        #If you're launching a python file, you're python, otherwise you're C++. TODO: Do this more intelligently.
        if '.py' in self.logMsg.node_name:
            self.logMsg.node_name = self.logMsg.node_name.replace('.py','')
            self.logMsg.python = True
        else:
            self.logMsg.python = False

        #If you're launching from the local machine, you're local. Otherwise you're not. TODO: Do this more intelligently.
        if 'localhost' in record.args[3]:
            self.logMsg.master_uri = 'local'
        else:
            self.logMsg.master_uri = 'remote'

        self.logMsg.register = 1
        self.current_node_name = self.logMsg.node_name

    def parseEnvironment(self, record):
        self.logMsg.process_name = record.args[0].split('-',1)[0]

        #envDict contains a lot of information about the environment the user is using. Including the os (DESKTOP_SESSION) and ROS_DISTRO.
        envDict = record.args[1]
        self.logMsg.stamp = rospy.Time.now()

        #DESKTOP_SESSION doesn't exist when running from a remote computer.
        if 'DESKTOP_SESSION' in envDict:
            self.logMsg.desktop_session = envDict['DESKTOP_SESSION']
        self.logMsg.ros_distro = envDict['ROS_DISTRO']
        self.current_process_name = self.logMsg.process_name

    def successLaunch(self, record):
        rospy.logdebug('SUCCESSFULLY LAUNCHED: ' + self.logMsg.node_name)
        if self.logMsg.process_name is None:
            return
        self.processToNodeName[self.logMsg.process_name] = self.logMsg.node_name
        return self.logMsg
    
    def crashedProcess(self, record):
        if(record.args[0].split('-',1)[0] == 'master'):
            msg = LaunchLogger()
            msg.stamp = rospy.Time.now()
            msg.process_name = record.args[0].split('-',1)[0]
            msg.node_name = 'master'
            msg.register = 0
            return msg
        msg = LaunchLogger()
        msg.stamp = rospy.Time.now()
        msg.process_name = record.args[0].split('-',1)[0]
        msg.node_name = self.processToNodeName[msg.process_name]
        msg.register = 0
        return msg

    def killingProcess(self, record):
        if(record.args[0].split('-',1)[0] == 'master'):
            msg = LaunchLogger()
            msg.stamp = rospy.Time.now()
            msg.process_name = record.args[0].split('-',1)[0]
            msg.node_name = 'master'
            msg.register = 0
            return msg
        msg = LaunchLogger()
        msg.stamp = rospy.Time.now()
        msg.process_name = record.args[0].split('-',1)[0]
        msg.node_name = self.processToNodeName[msg.process_name]
        msg.register = 0
        return msg

    def parse_logging(self,line, record):
        #This line can cause errors. Use only for debugging.
        #rospy.logerr(line)
        for ind in self.options:
            if ind in line:
                msg = self.options[ind](record)
                return msg
        return None