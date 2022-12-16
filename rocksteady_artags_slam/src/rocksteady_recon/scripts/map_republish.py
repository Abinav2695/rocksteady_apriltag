#!/usr/bin/env python3  
import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import numpy as np
from nav_msgs.msg import OccupancyGrid
import sys


class map_handler():
    def __init__(self):
        self.map_sub = rospy.Subscriber('/rocksteady_cmap',OccupancyGrid,self.map_callback)
        self.map_pub = rospy.Publisher('/map',OccupancyGrid,queue_size=3)

    def map_callback(self,map_data):

        # re_map = OccupancyGrid()
        # re_map.header = map_data.header
        # re_map.info = map_data.info
        # re_map

        
        temp_list = list(map_data.data)
        M = 70
        N = 50
        for x in range(map_data.info.width):
            for y in range(map_data.info.height):
                i = x + (map_data.info.height - 1 - y) * map_data.info.width
                if temp_list[i] >= M:  
                    temp_list[i] = 100
                elif (temp_list[i] >= 0) and (temp_list[i] < N):  # free
                    temp_list[i] = 0
                else:                                                # unknown
                    temp_list[i] = -1
        
        map_data.data = tuple(temp_list)
        self.map_pub.publish(map_data) 

        # M = 70
        # N = 50
        # temp_list = []
        # for i in range (len(map_data.data)):
        #     temp = map_data.data[i]
        #     if(temp>=M):
        #         temp_list.append(100)  ## Occupied if probability of occupany is more than M
        #     elif(temp<=N and temp>=0):
        #         temp_list.append(0)  ## Unoccupied if prob is between N and 0 
        #     else:
        #         temp_list.append(-1)  ## for all -1 values append -1 because it is unknown space
        # map_data.data = tuple(temp_list)
        # self.map_pub.publish(map_data)

def main(args):
    rospy.init_node('map_republish_node', anonymous=False)
    mh = map_handler()
#   rate = rospy.Rate(hz=60)
    try:
    # while not rospy.is_shutdown():
    #     ic.get_frame()
    #     rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)