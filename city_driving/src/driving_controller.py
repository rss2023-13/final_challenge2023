#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tf
import math

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
        STOP_TOPIC = "/clicked_point"
        #new subscriber for the stop signd detector at topic /stop_sign
        rospy.Subscriber("/intersection", PointStamped, self.lookahead_callback)
        #make state machine that changes from this stop_sign topic to look_ahead topic
        rospy.Subscriber(STOP_TOPIC, PointStamped, self.stop_callback)

        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback, queue_size=1)

        rospy.Subscriber("/odom", Odometry, self.vel_callback, queue_size=1)

        self.robot_pose = None

        self.parking_distance = 0 #.9 # goal distance from target (between .75 and 1 meter)
        self.stopping_distance = .9 #in front of stop sign
        self.relative_x = 0
        self.relative_y = 0
        self.stop_x = None
        self.stop_y = None

        self.curr_velocity = 0
        self.stop_time = None
        self.just_stopped = False

        self.prev_angle = 0
        self.prev_time = rospy.get_time()

        #working for v=1: p=1, d=.13,
        #working for v=2: p=1.5, d=.135
        self.steering_kp = 7
        self.steering_kd = 0

        self.velocity_kp = 0.8
        self.velocity_max = 5

        self.already_stopped = False
        self.stopping = False

    def vel_callback(self, odom):
        self.curr_velocity = odom.twist.twist.linear.x

    def pose_callback(self, odom):
        self.robot_pose = odom.pose.pose

    def lookahead_callback(self, msg):
        self.relative_x, self.relative_y = self.world_to_robot(msg.point.x, msg.point.y) #where lookahead is relative to robot
        drive_cmd = AckermannDriveStamped()

        #################################

        # Goal: Drive to a specified location of self.parking_distance away from cone
        # Also, make sure to face the point
        # Goal is to move the x_error and y_error to 0, where x_error = self.relative_x - self.parking_distance
        # and y_error = self.relative_y
        
        y_error = self.relative_y
        x_error = self.relative_x - self.parking_distance
        angle = np.arctan2(self.relative_y, self.relative_x)
        print(angle)
        overall_error = np.linalg.norm([x_error, y_error])

        current_time = rospy.get_time()

        angle_derivative = (angle - self.prev_angle)/(current_time - self.prev_time)

        steering_angle = self.steering_kp * angle + self.steering_kd*angle_derivative

        if abs(angle) > math.pi/2:
            velocity = -self.velocity_kp
            steering_angle = 0
        else:

            # Set the velocity
            if x_error >= 0:
                velocity = self.velocity_kp * overall_error #error_to_use
            else:
                steering_angle = angle
                velocity = .2 * overall_error

        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = np.min([velocity, self.velocity_max])
        # rospy.loginfo("steering" + str(steering_angle))

        #################################
        if self.stopping: #have seen stop sign

            if self.curr_velocity == 0:
                # stopped at stop sign - velocity is 0
                if self.just_stopped: 
                    print('just stopped')
                    self.just_stopped = False
                    self.stop_time = rospy.get_time() # start timer if we just reach v=0
                if rospy.get_time() - self.stop_time > 0.8: #have stopped for 2sec (tune this)
                    # remove stop sign coords because we have now stopped at it
                    print('DONE')
                    self.stop_x = None
                    self.stop_y = None
                    self.stopping = False
                else:
                    drive_cmd.drive.speed = 0 #else continue to override drive commands

            elif self.world_to_robot(self.stop_x, self.stop_y)[0] < 1.2:
                # within 1.2 meters of stop sign - issue stop command
                print('setting to 0')
                drive_cmd.drive.speed = 0

        self.drive_pub.publish(drive_cmd)
        self.prev_angle = angle
        self.prev_time = current_time

    def stop_callback(self, msg):
        """
        Activates when a stop sign is detected. Will trigger stop sign drive which 
        involves slowing down to a stop 'stop_distance' meters away from the stop sign.
        Releases control after comming to a stop for .5 seconds 
        """
        #stop sign location
        #add an if to see if x and y are 0 (or -1) or another indication that there isn't 
        #a visible stop sign near so we do not use the stop PID

        #homography points
        self.stop_x, self.stop_y = msg.point.x, msg.point.y
        self.just_stopped = True
        self.stopping = True

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

    def world_to_robot(self, x, y):
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
