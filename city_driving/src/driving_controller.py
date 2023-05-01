#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tf

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

class DrivingController():
    """
    A controller for driving to lookahead distance on trajectory.
    Listens for a point to drive to and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic", "/pf/pose/odom")
        STOP_TOPIC = "/stop_sign"
        #new subscriber for the stop signd detector at topic /stop_sign
        rospy.Subscriber("/intersection", PointStamped,
            self.lookahead_callback)
        #make state machine that changes from this stop_sign topic to look_ahead topic
        rospy.Subscriber(STOP_TOPIC, PointStamped,
            self.stop_callback)

        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback, queue_size=1)

        self.robot_pose = None

        self.parking_distance = 0 #.9 # goal distance from target (between .75 and 1 meter)
        self.stopping_distance = .9 #in front of stop sign
        self.relative_x = 0
        self.relative_y = 0

        self.prev_angle = 0
        self.prev_time = rospy.get_time()

        #working for v=1: p=1, d=.13,
        #working for v=2: p=1.5, d=.135
        self.steering_kp = 7
        self.steering_kd = 0

        self.velocity_kp = 1.1
        self.velocity_max = 5

        self.already_stopped = False
        self.stopping = False

    def pose_callback(self, odom):
        self.robot_pose = odom.pose.pose

    def lookahead_callback(self, msg):
        self.relative_x, self.relative_y = self.robot_to_world(msg.point.x, msg.point.y)
        drive_cmd = AckermannDriveStamped()

        #################################

        # Goal: Drive to a specified location of self.parking_distance away from cone
        # Also, make sure to face the point
        # Goal is to move the x_error and y_error to 0, where x_error = self.relative_x - self.parking_distance
        # and y_error = self.relative_y
        
        #while there isn't a stop sign near
        if not self.stopping:

            y_error = self.relative_y
            x_error = self.relative_x - self.parking_distance
            angle = np.arctan2(self.relative_y, self.relative_x)
            overall_error = np.linalg.norm([x_error, y_error])

            current_time = rospy.get_time()

            angle_derivative = (angle - self.prev_angle)/(current_time - self.prev_time)

            steering_angle = self.steering_kp * angle + self.steering_kd*angle_derivative

            # Set the velocity
            if x_error >= 0:
                velocity = self.velocity_kp * overall_error #error_to_use
            else:
                steering_angle = angle
                velocity = .2 * overall_error

            print('y', y_error)

    #        if y_error > 0.8:
    #            velocity = 0
    #            print('stopping', y_error)
    #        elif y_error > 0.2:
    #            steering_angle *= 2

            drive_cmd.drive.steering_angle = steering_angle
            drive_cmd.drive.speed = np.min([velocity, self.velocity_max])
            # rospy.loginfo("steering" + str(steering_angle))

            #################################

            self.drive_pub.publish(drive_cmd)
            self.prev_angle = angle
            self.prev_time = current_time
        # self.error_publisher()

    def stop_callback(self, msg):
        """
        Activates when a stop sign is detected. Will trigger stop sign drive which 
        involves slowing down to a stop 'stop_distance' meters away from the stop sign.
        Releases control after comming to a stop for .5 seconds 
        """
        #stop sign location
        #add an if to see if x and y are 0 (or -1) or another indication that there isn't 
        #a visible stop sign near so we do not use the stop PID
        if not self.stopping and not self.already_stopped:
            #sign detected, enter stopping state
            self.stopping = True 
        if self.stopping:
            self.relative_x, self.relative_y = self.robot_to_world(msg.point.x, msg.point.y)

            drive_cmd = AckermannDriveStamped()

            y_error = self.relative_y
            x_error = self.relative_x - self.stopping_distance
            angle = np.arctan2(self.relative_y, self.relative_x)
            overall_error = np.linalg.norm([x_error, y_error])
            rospy.loginfo("angle = " + str(angle * 180 / np.pi))
            # we should not go in circles, stop sign guaranteed to be in front of car
            
            steering_angle = self.steering_kp * angle


            # Set the velocity
            if x_error >= 0:
                velocity = self.velocity_kp * overall_error #error_to_use
            elif x_error > -.05:
                velocity = self.velocity_kp * y_error
            else:
                velocity = self.velocity_kp * x_error

            if velocity < .05 :
                #we are almost fully stopped so wait for 1 second?
                #do some waiting ... maybe while(100)?

                self.stopping = False

                #now give up control and go back to lookahead callback
                self.already_stopped = True

            drive_cmd.drive.steering_angle = steering_angle
            drive_cmd.drive.speed = np.min([velocity, self.velocity_max])

            #################################

            self.drive_pub.publish(drive_cmd)
            self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        #################################

        # YOUR CODE HERE
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x**2 + self.relative_y**2)

        #################################
        
        self.error_pub.publish(error_msg)

    def robot_to_world(self, x, y):
        angle = self.euler_from_quaternion(self.robot_pose.orientation)[2]
        rotation_matrix = np.array([[np.cos(angle), np.sin(angle), 0], 
                                    [-np.sin(angle), np.cos(angle), 0], 
                                    [0, 0, 1]])
        rotated_coord =  np.matmul(rotation_matrix, np.array([x - self.robot_pose.position.x, y - self.robot_pose.position.y, 0]))
        
        return (rotated_coord[0], rotated_coord[1])

    def euler_from_quaternion(self, quaternion):
        return tf.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

# if __name__=="__main__":
#     rospy.init_node("driving_controller")
#     dc = DrivingController()
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         rospy.init_node('ParkingController', anonymous=True)
#         ParkingController()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
