#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_VELOCITY = 10.0 # In miles per hour


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # TODO: Uncomment when traffic light detection node and/or obstacle detection node is implemented
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)



        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Storage for current pose received via subscribtion to '/current_pose'
        self.pose = None

        # Storage for current base waypoints received via subscribtion to '/base_waypoints'
        self.base_waypoints = None

        rospy.spin()


    def pose_cb(self, msg):
        # TODO: Implement
        """
        Step 1: Update of self.pose
        """
        self.pose = msg.pose
        rospy.loginfo('WaypointUpdater: Updated with current pose')
        
        """
        Step 2: Publication of the waypoints ahead, which is supposed to be in this function, as a new publication is required every time we receive an update of the car's pose
        """
        # Only publish waypoints ahead if we already received base_waypoints
        if self.base_waypoints:

            # Define container for waypoints ahead
            wps_ahead = Lane()
            
            # Determine length of input waypoints
            waypoints_len = len(self.base_waypoints)

            # Determine target velocity
            target_velocity = MAX_VELOCITY
            #TODO: Implement statements to adjust target velocity in case we are approaching a red traffic light 

            # Determine first waypoint ahead of the car
            idx_wp_closest = get_idx_closest_waypoint()
            idx_wp_ahead = get_idx_ahead_waypoint(idx_wp_closest)

            # Determine waypoints ahead to be published
            idx_cur = idx_wp_ahead
            for i in range (LOOKAHEAD_WPS):
                wp = self.base_waypoints[idx_cur]
                next_wp = Waypoint()
                next_wp.pose = wp.pose
                next_wp.twist.twist.linear.x = target_velocity
                wps_ahead.waypoints.append(next_wp)
                idx_cur = (idx_cur + 1) % waypoints_len
            
            # Publish waypoints ahead
            self.final_waypoint_pub.publish(wps_ahead)


    def waypoints_cb(self, waypoints):
        """
        Stores the waypoints initially received by subscription in the class variable 'base_waypoints'
        <- waypoints: waypoints subscribed in the 'Lane'-datatype
        """
        # TODO: Implement
        self.base_waypoints = waypoints
        rospy.loginfo('WaypointUpdater: Updated with current waypoints')


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def get_idx_closest_waypoint(self):
        # Only feasible if there are base_waypoints and the position of the car available
        if (self.base_waypoints and self.pose):
            min_wp_dist = 1000000
            idx_wp_closest = None
            for i in range(len(self.base_waypoints)):
                dist = self.get_eucl_distance(self.base_waypoints[i].pose.pose.postition.x, self.base_waypoints[i].pose.pose.position.y,self.pose.position.x,self.pose.position.y)
                if dist < min_wp_dist:
                    min_wp_dist = dist
                    idx_wp_closest = i

            return idx_wp_closest

        else:
            return None


    def get_idx_ahead_waypoint(self, idx_wp_closest):
        wp_x_local,_,_ = self.transform_wp_to_local(self.base_waypoints[idx_wp_closest])
        if wp_x_local > 0.0:
            return idx_wp_closest
        else:
            return idx_wp_closest + 1
        
    def transform_wp_to_local(self, wp)
        wx = wp.pose.pose.position.x
        wy = wp.pose.pose.position.y
        # Get yaw
        _,_,yaw = tf.transformation.euler_from quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])

        dx = wx - self.px
        dy = wy - self.py
        wx_local = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        wy_local = math.sin(-yaw) * dx - math.cos(-yaw) * dy
        return wx_local, wy_local, math.atan2(local_wy, local_wx)


    def get_eucl_distance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)




if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
