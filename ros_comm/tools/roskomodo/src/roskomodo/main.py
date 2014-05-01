#!/usr/bin/env python
import rospy
from roskomodo.msg import RegistrationLogger
from roskomodo.msg import LaunchLogger
from roslaunch.core import *
import rosservice
from xml.dom.minidom import Document
import rosnode
import rospkg
import time

class roskomodo(object):
    def __init__(self):
        rospy.init_node('roskomodo', log_level=rospy.DEBUG)
        self.sub_registration = rospy.Subscriber('/registration_logger', RegistrationLogger, self.reg_callback)
        self.sub_launch = rospy.Subscriber('/launch_logger', LaunchLogger, self.launch_callback)
        self.registeredList = list()
        self.launchList = list()

        #I don't think there is a function to acquire this information.
        self.processNameToNode = {'main': 'main', 'rosout':'rosout'}


        #For services/topics that exist when roskomodo begins. Assume they started
        #approx. at the same time roskomodo did, since roskomodo is in roscore
        master_uri = os.environ.get(rosgraph.ROS_MASTER_URI, None)
        self.master = xmlrpclib.ServerProxy(master_uri)
        s = self.master.getSystemState('/roskomodo')
        tt = self.master.getTopicTypes('/roskomodo')
        topicTypes = tt[2]
        publishers = s[2][0]
        subscribers = s[2][1]
        services = s[2][2]
        for p in publishers:
            if 'reg_logger' in p[1][0]:
                continue
            msg = RegistrationLogger()
            msg.msg_type = 'Publisher'
            msg.name = p[0]
            msg.process_name = p[1][0]
            msg.stamp = rospy.Time.now()
            msg.register = 1

            for t in topicTypes:
                if t[0] == msg.name:
                    msg.topic_type = t[1]

            self.reg_callback(msg)

        for p in subscribers:
            if 'reg_logger' in p[1][0]:
                continue
            msg = RegistrationLogger()
            msg.msg_type = 'Subscriber'
            msg.name = p[0]
            msg.process_name = p[1][0]
            msg.stamp = rospy.Time.now()
            msg.register = 1

            for t in topicTypes:
                if t[0] == msg.name:
                    msg.topic_type = t[1]

            self.reg_callback(msg)

        for p in services:
            if 'reg_logger' in p[1][0]:
                continue
            msg = RegistrationLogger()
            msg.msg_type = 'Service'
            msg.name = p[0]
            msg.process_name = p[1][0]
            msg.stamp = rospy.Time.now()
            msg.register = 1
            self.reg_callback(msg)


    def launch_callback(self, msg):
        #Connects process names to node names. Needed to connect topics to nodes launching them.
        self.processNameToNode[msg.process_name] = msg.node_name

        #If unregistering, find the corresponding node and compute duration.
        if(msg.register == 0):
            for regEle in self.launchList:
                if regEle.process_name == msg.process_name and regEle.duration == 0:
                    rospy.logdebug('Unregistering Node: ' + msg.node_name)
                    regEle.duration = msg.stamp.to_sec() - regEle.stamp.to_sec() 
                    return


        if(msg.register == 0):
            #If the msg trying to unregister is roskomodo, everything is being shut down. Print and quit.
            if msg.process_name == 'roskomodo' and msg.node_name == 'main':
                self.output_xml()
                return
            rospy.logdebug('Node Error: ' + msg.node_name + ' did not find match when unregistering node')
            return

        rospy.logdebug('Registering Node: ' + msg.node_name)
        self.launchList.append(msg)


    def reg_callback(self, msg):
        #If unregistering, find the corresponding topic/service and compute duration.
        if(msg.register == 0):
            for regEle in self.registeredList:
                if regEle.name == msg.name and regEle.duration == 0 and regEle.process_name == msg.process_name:
                    rospy.logdebug('Unregistering: ' + msg.name)
                    regEle.duration = msg.stamp.to_sec() - regEle.stamp.to_sec()
                    return
        
        #If you are unregistering a topic/service that wasn't registered, 
        #something went wrong.            
        if(msg.register == 0):
            rospy.logdebug('Topic Error: ' + msg.name + ' ' + msg.process_name + ' did not find match when unregistering')
            return
        
        rospy.logdebug('Registering: ' + msg.name)
        self.registeredList.append(msg)


    def preprocess_xml(self):
        tt = self.master.getTopicTypes('/roskomodo')
        topicTypes = tt[2]

        for msg in self.registeredList:
            #If process_name -> node_name mapping doesn't exist, delete it.
            if msg.process_name.replace('/','', 1) not in self.processNameToNode:
                rospy.logdebug(msg.process_name.replace('/','', 1) + " not in processNameToNode, deleting")
                self.registeredList.remove(msg)
                continue

            node_name = self.processNameToNode[msg.process_name.replace('/','',1)]
            #Fix duration if node shuts down *after* roskomodo. Otherwise it's an error and delete it
            if msg.duration == 0:
                node_name = self.processNameToNode[msg.process_name.replace('/','',1)]
                if "roskomodo" in node_name or "main" in node_name or "rosout" in node_name:
                    msg.duration = rospy.Time.now().to_sec() - msg.stamp.to_sec()
            
            for t in topicTypes:
                if t[0] == msg.name:
                    msg.topic_type = t[1]
                    break



    def output_xml(self):
        
        self.preprocess_xml()

        doc = Document()
        root = doc.createElement('root')
        doc.appendChild(root)
        msgs = doc.createElement('msgs')
        root.appendChild(msgs)

        for msg in self.registeredList:

            rospy.logdebug(msg.process_name)
            indvidual_msg = doc.createElement('msg')
            name = doc.createElement('name')
            name_content = doc.createTextNode(msg.name)
            name.appendChild(name_content)
            indvidual_msg.appendChild(name)

            msg_type = doc.createElement('msg_type')
            msg_type_content = doc.createTextNode(msg.msg_type)
            msg_type.appendChild(msg_type_content)
            indvidual_msg.appendChild(msg_type)

            topic_type = doc.createElement('topic_type')
            topic_type_content = doc.createTextNode(msg.topic_type)
            topic_type.appendChild(topic_type_content)
            indvidual_msg.appendChild(topic_type)

            node_name = doc.createElement('node_name')
            node_name_content = doc.createTextNode(self.processNameToNode[msg.process_name.replace('/','',1)])
            node_name.appendChild(node_name_content)
            indvidual_msg.appendChild(node_name)

            duration = doc.createElement('duration')
            duration_content = doc.createTextNode(str(msg.duration))
            duration.appendChild(duration_content)
            indvidual_msg.appendChild(duration)

            msgs.appendChild(indvidual_msg)


        launches = doc.createElement('launches')
        root.appendChild(launches)

        for msg in self.launchList:
            indvidual_msg = doc.createElement('launch')
            package = doc.createElement('package')
            package_content = doc.createTextNode(msg.package)
            package.appendChild(package_content)
            indvidual_msg.appendChild(package)

            node_name = doc.createElement('node_name')
            node_name_content = doc.createTextNode(msg.node_name)
            node_name.appendChild(node_name_content)
            indvidual_msg.appendChild(node_name)

            process_name = doc.createElement('process_name')
            process_name_content = doc.createTextNode(msg.process_name)
            process_name.appendChild(process_name_content)
            indvidual_msg.appendChild(process_name)

            desktop_session = doc.createElement('desktop_session')
            desktop_session_content = doc.createTextNode(msg.desktop_session)
            desktop_session.appendChild(desktop_session_content)
            indvidual_msg.appendChild(desktop_session)
            
            master_uri = doc.createElement('master_uri')
            master_uri_content = doc.createTextNode(msg.master_uri)
            master_uri.appendChild(master_uri_content)
            indvidual_msg.appendChild(master_uri)

            ros_distro = doc.createElement('ros_distro')
            ros_distro_content = doc.createTextNode(msg.ros_distro)
            ros_distro.appendChild(ros_distro_content)
            indvidual_msg.appendChild(ros_distro)

            duration = doc.createElement('duration')
            duration_content = doc.createTextNode(str(msg.duration))
            duration.appendChild(duration_content)
            indvidual_msg.appendChild(duration)

            launches.appendChild(indvidual_msg)

        log_dir = rospkg.get_log_dir()
        date_time = time.strftime("%j-%H-%S")
        file_dir = log_dir + "/roskomodo-" + date_time + ".xml"
        f = open(file_dir,'a')
        f.write(doc.toprettyxml(indent="    ", encoding="utf-8"))

if __name__ == "__main__":
    a = roskomodo()
    rospy.spin()
