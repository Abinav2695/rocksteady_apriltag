#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import tf2_ros
import tf_conversions
import sys



class reroute_handler():

    passThrough = True
    circularMotionEndTime =0
    lastCircularMotionCommandExecutionTime=0
    circularMotionDuration = rospy.Duration(15)  ##15 secs
    exploreLiteCommandPassThroughDuration = rospy.Duration(25)  ##once in every 20 secs stop the bot and start custom rotation 

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/cmd_vel_reroute', Twist, self.move_base_command_callback, queue_size=1)


    def move_base_command_callback(self,vel_msg):

        if not self.passThrough:
            return
        self.cmd_pub.publish(vel_msg)

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)
    
    def get_current_secs(self):
        currTime = rospy.Time.now()
        currTime = currTime.secs
        return currTime

    def get_current_nsecs(self):
        currTime = rospy.Time.now()
        currTime = currTime.nsecs
        return currTime

    def handle_whirl_movement(self):
        ## Perform one rotation in 30 secs
        ## angle covered = 2*pi
        ## Total time = 20 secs
        ## velocity required  = 2*pi/15 = 0.41887902047
        vel_cmd = Twist()
        currTime = self.get_current_timestamp()
        if(currTime - self.lastCircularMotionCommandExecutionTime > self.exploreLiteCommandPassThroughDuration.to_sec()):
            self.passThrough = False  ##This will block the commands from explore lite in callback function
            vel_cmd.linear.x = 0
            vel_cmd.linear.y = 0
            vel_cmd.linear.z = 0
            vel_cmd.angular.z = 0
            self.cmd_pub.publish(vel_cmd)  ##stop the bot

            vel_cmd.angular.z = 0.41887902047
            self.circularMotionEndTime = self.get_current_timestamp() + self.circularMotionDuration.to_sec()
            print('[INFO]: Executing whirl motion')
            while (self.get_current_timestamp() <= self.circularMotionEndTime):
               
                self.cmd_pub.publish(vel_cmd)
                rospy.sleep(0.1)
            vel_cmd.angular.z = 0
            self.cmd_pub.publish(vel_cmd)
            self.lastCircularMotionCommandExecutionTime = self.get_current_timestamp()
        self.passThrough = True




def main(args):

    rospy.init_node('cmd_velocity_reroute_node')
    
    rh = reroute_handler()
    rate = rospy.Rate(hz=30)
    try:
        while not rospy.is_shutdown():
            rh.handle_whirl_movement()
            rate.sleep()
    except rospy.ROSInterruptException:
        print("Shutting down")



if __name__ == '__main__':
    main(sys.argv)
