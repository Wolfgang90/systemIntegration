#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        rospy.loginfo('Start initialization of DBWNode')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)


        self.linear_velocity_future = None
        self.angular_velocity_future = None
        self.linear_velocity_current = None
        self.angular_velocity_current = None
        self.acceleration_current = None
        self.dbw_enabled = None


        self.rate = 10 # Rate in Hz

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(
            vehicle_mass = vehicle_mass,
            fuel_capacity = fuel_capacity,
            brake_deadband = brake_deadband,
            decel_limit = decel_limit,
            accel_limit = accel_limit,
            wheel_radius = wheel_radius,
            rate = self.rate
            )

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twistcmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cur_vel_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)


        rospy.loginfo('DBWNode: Subscribed to relevant topics')

        self.loop()

    def loop(self):
        rospy.loginfo('DBWNode: Started looping')
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            if(self.linear_velocity_current == None or self.linear_velocity_future == None or not self.dbw_enabled):
                continue
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            throttle, brake, steer = self.controller.control(
                linear_velocity_future = self.linear_velocity_future, 
                angular_velocity_future = self.angular_velocity_future, 
                linear_velocity_current = self.linear_velocity_current, 
                angular_velocity_current = self.angular_velocity_current,
                acceleration_current = self.acceleration_current)

            rospy.loginfo('DBWNode: Controller output: throttle -> %.2f     brake -> %.2f     steer -> %.2f', throttle, brake, steer)

            self.publish(throttle, brake, steer)


            rate.sleep()

    def publish(self, throttle, brake, steer):
        rospy.loginfo('DBWNode: Publisher started with: throttle -> %.2f     brake -> %.2f     steer -> %.2f', throttle, brake, steer)
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


    # TODO: Implement
    def twistcmd_cb(self, msg):
        self.linear_velocity_future = msg.twist.linear.x
        self.angular_velocity_future = msg.twist.angular.z
        #rospy.loginfo('DBWNode: Updated twist')


    # TODO: Implement
    def cur_vel_cb(self, msg):
        if (self.linear_velocity_current is not None):
            self.acceleration_current = self.rate * (self.linear_velocity_current - msg.twist.linear.x)
        self.linear_velocity_current = msg.twist.linear.x
        self.angular_velocity_current = msg.twist.angular.z
        #rospy.loginfo('DBWNode: Updated velocity')


    # TODO: Implement
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo('DBWNode: Updated dbw_enabled with %d', self.dbw_enabled)

        if (not self.dbw_enabled):
            self.controller.reset()
        
        


if __name__ == '__main__':
    DBWNode()
