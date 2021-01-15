#!/usr/bin/env python
import rospy
 
import sys  # command line arguments argv
import math  # atan2
 
# TODO: Import the messages we need
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
 
 
class TurtleWaipoint(object):
    """Class to guide the turtle to the specified waypoint."""
 
    def __init__(self, waypoint_x=None, waypoint_y=None):
        """Class constructor."""
        # Init all variables
        # Current turtle position
        self.x = None
        self.y = None
        self.theta = None
        # Tolerance to reach waypoint
        self.tolerance = 0.1
        # A position was received
        self.got_position = False
        # Reached position
        self.finished = False
 
        # ROS init
        rospy.init_node('turtle_waypoint')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        
        self.vel_msg = Twist()

        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y

        if self.waypoint_x is None or self.waypoint_y is None:
            if rospy.get_param("/default_x") is True and rospy.get_param("/default_y") is True:
                print("Waypoint found in param server")
                self.waypoint_x = rospy.get_param("/default_x")
                self.waypoint_y = rospy.get_param("/default_y")
            else:
                print("No waypoint found in param server")
                exit(1)
 
        print('Heading to: {:.2f}, {:.2f}'.format(self.waypoint_x,
                                                  self.waypoint_y))
 
    def callback(self, msg):
        """Saves the tutle position when a message is received."""
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.got_position = True
 
    def iterate(self):
        """Keeps sending control commands to turtle until waypoint reached."""
        if self.finished:
            print('Waypoint reached, final position X: ',self.x,'  Y: ', self.y)
            exit(0)
        else:
            # We know where we are
            #self.got_position = True #for debugging
            if self.got_position:

                self.x_displacement = self.waypoint_x - self.x
                self.y_displacement = self.waypoint_y - self.y
                self.ang_displacement = math.atan2(self.y_displacement, self.x_displacement)
                self.distance = math.sqrt(self.x_displacement**2+self.y_displacement**2)

                if self.distance < self.tolerance:
                    self.finished = True

                else:
 
                    self.angular_Pgain = 1
                    self.linear_Pgain = 0.5
 
                    if (abs(self.theta - math.atan2(self.y_displacement, self.x_displacement)) > 0.01):

                        self.vel_msg.angular.z = self.angular_Pgain * (math.atan2(self.y_displacement, self.x_displacement)-self.theta)

                    if (abs(self.theta - math.atan2(self.y_displacement, self.x_displacement)) < 0.4):

                        self.vel_msg.linear.x = self.linear_Pgain*self.distance

                    self.pub.publish(self.vel_msg)

                    print('self angle: ', self.theta,' atan2 angle: ', math.atan2(self.y_displacement, self.x_displacement),' X: ',self.x,'  Y: ', self.y, 'distance: ', self.distance)

if __name__ == '__main__':
    # Check commandline inputs
    if not len(sys.argv) == 3:
        # No input waypoint specified
        print('No waypoint specified in commandline')
        node = TurtleWaipoint()
    else:
        node = TurtleWaipoint(float(sys.argv[1]), float(sys.argv[2]))
    # Run forever
    while not rospy.is_shutdown():
        node.iterate()
        rospy.sleep(0.3)
    print('\nROS shutdown')