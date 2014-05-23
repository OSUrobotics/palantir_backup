roskomodo
=========

Description:
ROSKomodo is a replacement for ros_comm that monitors what packages users launch, and records the topics, services, environment, and packages associated with that launch. These packages are stored in /.ros/log and periodically sent to the wiki web server to be uploaded to a database.

Details:
ROSKomodo adds the files rosmaster.registration_logger and the roskomodo package to the ros_comm infrastructure. The rosmaster.registration_logger class adds a handler to the logger associated with the argument given, and overloads the emit function. This emit function parses the log and publishes data about what had occured, such as registering a publisher/subscriber or launching a node. Roskomodo subscribes to these publishers and stores the data. When the core is killed, this data is then saved in an xml file in .ros/log. The user can opt out at any time by modifying the roscore.xml file and removing the roskomodo package. In this way, rosmaster.registration_logger publishes interesting data, but roskomodo does not store it, it is simply for the users benefit.

How to use: Clone the github repository and add it to your catkin workspace. Once you perform a catkin_make and source, these packages overwrite the built-in ros_comm. Also clone the rosbash_komodo github repository for rosrun support.

Git hub: https://github.com/OSUrobotics/roskomodo	 https://github.com/OSUrobotics/rosbash_komodo

Bugs and Ideas/Enhancement reporting: https://github.com/OSUrobotics/roskomodo/issues
