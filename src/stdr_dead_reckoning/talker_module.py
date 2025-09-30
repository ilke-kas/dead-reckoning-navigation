#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import math

class Talker(object):
    def rotate_robot(self,pub,rate,velocity_command, angular_speed, target_angle):
        """
        Rotate the robot by a specified angle (in radians) at a given angular speed.
        :param angular_speed: Angular speed in radians/sec.
        :param target_angle: Target angle to rotate in radians.
        """
        rospy.loginfo(f"Rotating robot by {target_angle} radians at {angular_speed} rad/s...")
        velocity_command.linear.x = 0.0
        # needed to set the angular z velocity
        velocity_command.angular.z = angular_speed
        # get the start time for movement
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            #calculate the elapsed time whether to check the total movement time is reached, break
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time >= target_angle / abs(angular_speed):
                rospy.loginfo("Rotation complete.")
                break
            pub.publish(velocity_command)
            rate.sleep()
        # set angular velocity.z back to 0, so the robot stops 
        velocity_command.angular.z = 0.0
        pub.publish(velocity_command)

        rospy.sleep(1) 

    def move_forward(self,pub,rate,velocity_command, linear_speed, distance):
        """
        Move the robot forward for a specified distance at a given linear speed.
        :param linear_speed: Linear speed in meters/sec.
        :param distance: Distance to move in meters.
        """
        rospy.loginfo(f"Moving forward by {distance} meters at {linear_speed} m/s...")
        velocity_command.linear.x = linear_speed
        # needed to set the angular z velocity
        velocity_command.angular.z = 0.0
        # get the start time for movement
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            #calculate the elapsed time whether to check the total movement time is reached, break
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            distance_covered = elapsed_time * linear_speed
            if distance_covered >= distance:
                rospy.loginfo("Target distance reached. Stopping robot.")
                # set linear velocity.x back to 0, so the robot stops 
                velocity_command.linear.x = 0.0
                pub.publish(velocity_command)
                break
            pub.publish(velocity_command)
            rate.sleep()
        rospy.sleep(3) 

    def talker(self):
    
        rospy.init_node('dead_reckoning_talker', anonymous=True)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.sleep(2)
        rate = rospy.Rate(10) 
        velocity_command = Twist()

        # set of move forward and rotate robot instructions to move robot from (1,2,0) to (1,14,0)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=0.5, target_angle=1.570796)
        self.move_forward(pub,rate,velocity_command,linear_speed=0.5, distance=3.0)
        rospy.sleep(2)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=-0.5, target_angle=1.570796)
        self.move_forward(pub,rate,velocity_command,linear_speed=0.5, distance=6.5)
        rospy.sleep(2)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=0.5, target_angle=1.570796)
        self.move_forward(pub,rate,velocity_command,linear_speed=0.5, distance=2.2)
        rospy.sleep(2)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=0.5, target_angle= 1.570796)
        self.move_forward(pub,rate,velocity_command,linear_speed=0.5, distance=3.5)
        rospy.sleep(2)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=-0.5, target_angle= 1.570796)
        self.move_forward(pub,rate,velocity_command,linear_speed=0.5, distance=6.9)
        rospy.sleep(2)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=0.5, target_angle= 1.570796)
        self.move_forward(pub,rate,velocity_command,linear_speed=0.5, distance=2.5)
        # try to go back to orientation (0,0,0,1)
        self.rotate_robot(pub,rate,velocity_command,angular_speed=-0.5, target_angle= 3.141592)

        rospy.loginfo("Dead reckoning navigation completed.")


if __name__ == '__main__':
    try:
        talker_instance = Talker()
        talker_instance.talker()
    except rospy.ROSInterruptException:
        pass