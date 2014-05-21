#!/usr/bin/env python
import sys
import rospy
import logging
from roskomodo.msg import RegistrationLogger
from roskomodo.msg import LaunchLogger
import registration_logger

class RegistrationHandler(logging.Handler):
    """
    Custom logging handler class for publshing node and topic registrations.
    """
    def __init__(self):
        """
        Constructor   
        """
        logging.Handler.__init__(self)
        self.parser = RegistrationParser()
        self.loggingList = list()
        self.samples_seen = 0
    def register_node(self, logType):
        """
        Launch a node and create a publisher relative to the logType. This should only be called once.     
        @param logType: Type of log to be a handler of. i.e. 'rosmaster' for topics or 'roslaunch' for nodes.
        """
        if logType == 'rosmaster':
            self.pub = rospy.Publisher('registration_logger', RegistrationLogger)
        if logType == 'roslaunch':
            self.pub = rospy.Publisher('launch_logger', LaunchLogger)
        rospy.init_node('reg_logger', log_level=rospy.DEBUG, disable_rosout=True, disable_signals=True, anonymous=True)     
    def emit(self, record):
        """
        Overloading the emit function of the logging.Handler class. This is called whenever a logger is sent information to log.   
        @param record: Contains all the information pertinent to the event being logged.
        @type record: logging.LogRecord
        """
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
            #nodes not having their subscriptions registered by the time RegistrationHandler starts publishing.     
            if len(self.loggingList) != 0:
                for ele in self.loggingList:
                    self.pub.publish(ele)
                self.loggingList = []
                return

def add_logger(loggerName):
    """
    Global add_logger function access. In this way, any code in ros can add the RegistrationHandler to an existing logger.
    However, only rosmaster and roslaunch is really needed.
    """
    h = RegistrationHandler()
    h.register_node(loggerName)
    rospy.logdebug("ADDING LOGGER: " + loggerName)
    logging.getLogger(loggerName).addHandler(h)


class RegistrationParser(object):
    """
    Parser class for extracting metadata from logged records emitted by the RegistrationHandler.
    """
    def __init__(self):
        """
        Constructor builds the mapping from pattern to parser function.
        """

        self.processToNodeName = dict()
        self.options = {"-PUB" : self.unregister_publisher,
        "+PUB" : self.register_publisher,
        "-SUB" : self.unregister_subscriber,
        "+SUB" : self.register_subscriber,
        "-SERVICE" : self.unregister_service,
        "+SERVICE" : self.register_service,
        "create_node_process" : self.launch_node,
        "env[" : self.parse_environment,
        "... successfully launched" : self.success_launch,
        "process has died" : self.crashed_process,
        "killing os process" : self.killing_process}

    def unregister_publisher(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Publisher"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 0
        return msg

    def register_publisher(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Publisher"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 1
        return msg

    def unregister_subscriber(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Subscriber"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 0
        return msg

    def register_subscriber(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Subscriber"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 1
        return msg

    def unregister_service(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Service"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 0
        return msg

    def register_service(self,record):
        msg = RegistrationLogger()
        msg.name = record.args[0]
        msg.msg_type = "Service"
        msg.process_name = record.args[1]
        msg.stamp = rospy.Time.now()
        msg.register = 1
        return msg

    def launch_node(self, record):
        """
        First half of parsing when launching a node.
        Goes through the process variables and parses out variables associated with the executable.
        Does not return a msg, instead fills the logMsg with half of the data. The second half of the parsing is always logged next.
        """
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

    def parse_environment(self, record):
        """
        Second half of parsing when launching a node.
        Goes through the environment variables and parses out the variables associated with the process and environment.
        Does not return a msg, instead fills the logMsg with the second half of the data. The notification of a successful launch is always logged next.
        """
        self.logMsg.process_name = record.args[0].split('-',1)[0]

        #envDict contains a lot of information about the environment the user is using. Including the os (DESKTOP_SESSION) and ROS_DISTRO.
        envDict = record.args[1]
        self.logMsg.stamp = rospy.Time.now()

        #DESKTOP_SESSION doesn't exist when running from a remote computer.
        if 'DESKTOP_SESSION' in envDict:
            self.logMsg.desktop_session = envDict['DESKTOP_SESSION']
        self.logMsg.ros_distro = envDict['ROS_DISTRO']

    def success_launch(self, record):
        """
        Associates the process_name (The name given by the user and OS) to the node_name (The executable name). 
        This is used later when unregistering a node, so that the published message contains both the node and process name.
        Lastly returns the filled launch message to be published.
        """
        rospy.logdebug('SUCCESSFULLY LAUNCHED: ' + self.logMsg.node_name)
        if self.logMsg.process_name is None:
            return
        self.processToNodeName[self.logMsg.process_name] = self.logMsg.node_name
        return self.logMsg
    
    def crashed_process(self, record):
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

    def killing_process(self, record):
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
        """
        Cycles through a list of known patterns and calls the associated parser function. Returns either the parsed metadata or None.
        @param line: Text emitted by the logger pre-formatted.
        @type: string
        @param: record: Contains all the information pertinent to the event being logged.
        @type: logging.LogRecord
        """
        #This line can cause errors. Use only for debugging.
        #rospy.logerr(line)
        for ind in self.options:
            if ind in line:
                msg = self.options[ind](record)
                return msg
        return None