#!/usr/bin/env python

# Copyright 2020 by Spyridon Karachalios.
# All rights reserved for the turtlebot_rotate node.
# This file is part of an interview assessment.

import rospy
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3Stamped
from tf.transformations import euler_from_quaternion


def clamp(value, low, high):
    """Clamps a value between a low and a high value.
    low <= value <= high

    Args:
        value (float): The input value to clamp
        low (float): The lower end of range
        high (float): The higher end of the range

    Returns:
        float: The value clamped between low and high
    """
    return min(max(low, value), high)


class PIDController(object):
    """Simple PID Controller to control the movement 
    of the turtlebot.

    Args:
        object (): New style object inheritance for backwards compatibility
    """
    def __init__(self, kP, kI, kD, dt):
        # The gains for P,I,D
        self._kP = kP
        self._kI = kI
        self._kD = kD

        # Delta time since last PID calculation
        self._dt = dt

        # Integral of errors over time
        self._integral = 0

        # The error from the previous calculation
        self._prevError = 0

    def calculatePID(self, error):
        """Simple calculation of the PID output
        based on the input error.

        Args:
            error (float32): The error between the desired 
            and the actual value.

        Returns:
            float32: The total PID control output
        """
        P = self._kP * (error)
        self._integral += error * self._dt
        I = self._kI * self._integral
        D = self._kD * (error - self._prevError) / self._dt
        total = P + I + D
        self._prevError = error
        return total


class TurtlebotRotator(object):
    """The turtlebot rotator receives updates from the 
    Odometry and the Magnetometer and rotates the bot
    to face the north.

    Args:
        object ([type]): [description]
    """
    def __init__(self, pub):
        """Intialise the class

        Args:
            pub (cmd_vel publisher): The publisher to the /cmd_vel topic
        """
        # Get the publisher and initialise the command to send later
        self._pub = pub
        self._command = Twist()
        
        # Initialise the PID Controller
        self._PID = PIDController(2.5, 0.02, 0.6, 1/4.0)

        # No current measurement of yaw
        self._yaw = None
        
    def updateFromOdometry(self,msg):
        """Receives an update from the Odometry at the 
        /odom topic.

        Args:
            msg (Odometry): The message received from the /odom
        """
        # Just for debuggind purposes for now we calculate the yaw in
        # the local frame.
        orientationMsg = msg.pose.pose.orientation
        orientationQuat = [orientationMsg.x, orientationMsg.y, orientationMsg.z, orientationMsg.w]
        yaw = euler_from_quaternion(orientationQuat)[2]
        self._yaw = yaw
        
    def updateFromMagnetometer(self, msg):
        """Receives an update from the Magnetometer at the 
        /mag topic.

        Args:
            msg (Vector3Stamped): The message received from the /mag
        """
        # Calculate the desired heading based on the magnetometer
        xVec = msg.vector.x
        yVec = msg.vector.y
        errorRad = math.atan2(yVec, xVec)
        
        # Calculate the PID and turn the turtlebot towards the north
        commandAngularVel = self._PID.calculatePID(errorRad)
        commandAngularVel = clamp(commandAngularVel, -1.5, 1.5)
        self._command.angular.z = commandAngularVel
        self._pub.publish(self._command)
        rospy.loginfo("heading error: {},      current yaw: {}".format(errorRad * 180/math.pi, self._yaw * 180/math.pi))

def main():
    """Main function that creates the publisher and subscribers.
    It controls the turtlebot to point always to the north based
    on a magnetometer.
    """
    rospy.init_node('turtlebot_rotator')

    # Create a publisher to control the turtlebot
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Initialise the turtlebot rotator
    rotator = TurtlebotRotator(pub)

    # Subscribe the turtlebot rotator to the Odometry and Magnetometer
    rospy.Subscriber("/odom", Odometry, rotator.updateFromOdometry)
    rospy.Subscriber("/mag", Vector3Stamped, rotator.updateFromMagnetometer)

    # Do not exit the function and run in the background
    rospy.spin()

if __name__ == "__main__":
    main()