#!/usr/bin/env python3  
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.spatial.transform import Rotation as R



class artag_detections_handler():

    def __init__(self) -> None:
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tag_sub = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_callback)

        self.detected_tags = {}
        self.camera_frame = 'camera'
        self.base_frame = 'base_footprint'
        self.map_frame = 'map'
        self.TMC = None  ##camera to map transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.closed_tags={}
        self.camera_max_detect_distance = 3.5
        # self.open_tags=[]
        
        
    def tag_callback(self,tags):
        
        num_tags = len(tags.detections)
        if num_tags == 0:
            return
        
        for i in range(num_tags):
            tag_id= int(tags.detections[i].id[0])
            x = tags.detections[i].pose.pose.pose.position.x
            y = tags.detections[i].pose.pose.pose.position.y
            z = tags.detections[i].pose.pose.pose.position.z
            q0 = tags.detections[i].pose.pose.pose.orientation.w
            q1 = tags.detections[i].pose.pose.pose.orientation.x
            q2 = tags.detections[i].pose.pose.pose.orientation.y
            q3 =  tags.detections[i].pose.pose.pose.orientation.z

            if z<self.camera_max_detect_distance:
                translation_vector = np.matrix([[x],
                                                [y],
                                                [z],
                                                [1]])

                rotation_matrix = self.get_rotation_from_quat([q0,q1,q2,q3])                              
                rotation_matrix = np.vstack((rotation_matrix,np.matrix([0,0,0])))
                TCA = np.concatenate((rotation_matrix,translation_vector),axis=1)

                if self.TMC is None:
                    print('[WARNING]: Map to camera transform not available')
                    return

                ##Pose of Apriltag in map frame
                TMA = np.dot(self.TMC,TCA)


                if tag_id in self.closed_tags.keys():
                    print('UPDATING TAG: ', tag_id)
                    # update using learning rate.
                    # - use L=0 to throw away old data in favor of new.
                    L = 0.95
                    self.closed_tags[tag_id] = np.add(L * self.closed_tags[tag_id], (1-L) * TMA)
                else: 
                    print('FOUND NEW TAG: ', tag_id)
                    # create a new entry for this tag.
                    self.closed_tags[tag_id] = TMA
    
    def broadcast_tag_frames(self):

        for keys in self.closed_tags.keys():

            TMA = self.closed_tags[keys]
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()

            rotation_matrix_tam = TMA[0:3,0:3]
            r_tma = R.from_matrix(rotation_matrix_tam)
            q_tma = r_tma.as_quat()

            t.header.frame_id = self.map_frame
            t.child_frame_id = 'TAG_'+ str(keys)
            t.transform.translation.x = TMA[0,3]
            t.transform.translation.y = TMA[1,3]
            t.transform.translation.z = TMA[2,3]
            t.transform.rotation.x = q_tma[0]
            t.transform.rotation.y = q_tma[1]
            t.transform.rotation.z = q_tma[2]
            t.transform.rotation.w = q_tma[3]


            
            self.tf_broadcaster.sendTransform(t)
        
        self.save_tags_to_file()
    

    def save_tags_to_file(self):
        
        num_tags = len(self.closed_tags.keys())
        if not num_tags:
            return
        data_for_file = []
        for keys in self.closed_tags.keys():
            data_for_file.append("id: " + str(keys))
            for row in self.closed_tags[keys]:
                data_for_file.append(list(row))
            data_for_file.append("---------------------------------------")
        np.savetxt('Tags_detected.txt', data_for_file, fmt="%s", delimiter=",")
                

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
    
    def update_camera_to_map_transform(self):

        try:
            # get most recent relative pose from the tf service.
            camera_map_pose = self.tfBuffer.lookup_transform(self.map_frame, self.camera_frame,rospy.Time(0), rospy.Duration(4))
            
            translation_vector = np.matrix([[camera_map_pose.transform.translation.x],
                                            [camera_map_pose.transform.translation.y],
                                            [camera_map_pose.transform.translation.z],
                                            [1]])

            rotation_matrix = self.get_rotation_from_quat([camera_map_pose.transform.rotation.w,
                                                            camera_map_pose.transform.rotation.x,
                                                            camera_map_pose.transform.rotation.y,
                                                            camera_map_pose.transform.rotation.z])
            rotation_matrix = np.vstack((rotation_matrix,np.matrix([0,0,0])))
            self.TMC = np.concatenate((rotation_matrix,translation_vector),axis=1)

        except Exception as e:
            # requested transform was not found.
            print("Transform from MAP to Camera not found")
            print("Exception: ", e)
            return None



    def get_rotation_from_quat(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)/(w,x,y,z) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix orientation
        rot_matrix = np.matrix([[r00, r01, r02],
                                [r10, r11, r12],
                                [r20, r21, r22]])
                                
        return rot_matrix




def main():

    rospy.init_node('tag_pose_publisher',anonymous=False)
    

    adh = artag_detections_handler()
    rate = rospy.Rate(hz=60)
    
    lastUpdateTime = 0
    while not rospy.is_shutdown():
        
        curr_time = adh.get_current_timestamp()
        
        if((curr_time - lastUpdateTime) > 0.1):
            
            adh.update_camera_to_map_transform()
            adh.broadcast_tag_frames()
            lastUpdateTime = curr_time
        rate.sleep()





if __name__=='__main__':

    main()