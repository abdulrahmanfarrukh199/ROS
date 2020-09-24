#!/usr/bin/env python


import rospy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point, Twist

from sensor_msgs.msg import LaserScan



from tf.transformations import euler_from_quaternion

from math import atan2, sqrt, fabs

import time

global count_state

count_state = 0

global count
count = 0

# global constants
angle_eps = 0.2
dis_eps = 0.01

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None


class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        



state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle'
}

  

def distance (p1,p2):

    distance = sqrt((p1.y - p2.y)**2 + (p1.x - p2.x)**2)

    return distance

def main():

    global pub

    # define initial scenario
    state = 0

    # initialize ROS node
    rospy.init_node("bug_2")

    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(20)

    # Set the goal point 
    goal = Point()
    goal.x = 1.0
    goal.y = 0.0

    
    initial_pt = Point()
    initial_pt.x = 0
    initial_pt.y = 0

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 

    # Variable that stores the coordinate of hit point when you encounter obstacle for the first time
    hit_point = Point()
    hit_point.x = 1
    hit_point.y = 1

    # Trigger distance

    D = 0.25


    while not rospy.is_shutdown():
    	global count_state
    	global count
        

       
        point = Point()
        point.x = odom.x
        point.y = odom.y

    
        
        inc_x = goal.x - odom.x 
        inc_y = goal.y - odom.y
     	
       
         
        incc_x = closest_point.x - goal.x
        incc_y = closest_point.y - goal.y

       
         
        incc1_x = odom.x - closest_point.x
        incc1_y = odom.y - closest_point.y

      
         
        incc2_x = odom.x - hit_point.x
        incc2_y = odom.y - hit_point.y

         
         
        incc3_x = initial_pt.x - goal.x
        incc3_y = initial_pt.y - goal.y

     	

        angle_to_goal = atan2(inc_y, inc_x)
        
       

        angle_diff = angle_to_goal - odom.theta 

        

        dist_diff = sqrt((inc_x)**2+ (inc_y)**2) 

        distance_goal = sqrt((incc_x)**2+ (incc_y)**2) 

        distance_close = sqrt((incc1_x)**2+ (incc1_y)**2) 

        distance_hit = sqrt((incc2_x)**2+ (incc2_y)**2) 
    
        distance_close2 = sqrt((incc3_x)**2+ (incc3_y)**2) 
 
        distancedifference = fabs((goal.y - initial_pt.y) * odom.x - (goal.x - initial_pt.x) * odom.y + (goal.x * initial_pt.y) - (goal.y * initial_pt.x))

        distance1 = distancedifference/distance_close2

        

        # TODO:

        
        
        if state == 0:
            
                 
            
            if abs(angle_to_goal - odom.theta) > 0.20:

	       speed.linear.x = 0.0 
	       speed.angular.z = -0.2 
           
            else: 

               speed.linear.x = 0.2
               speed.angular.z = 0

               if dist_diff <= D:

                  speed.linear.x = 0
                  speed.angular.z = 0
            
	    if scan.region['front'] < D:
               closest_point = point               
               hit_point = point
               state = 1

            print "current state: ", state_dict[state]

            print "closest point: ", closest_point

            print "hit point: ", hit_point

            print "Current to goal: ",dist_diff

            print "Closest point to goal: ",distance_goal

        elif state == 1:
            

            if count_state > 40 and distance < D:
               state = (0)

            if scan.region['front'] < D: 

               speed.linear.x = 0.0
               speed.angular.z = -0.2 
    

            if scan.region['front'] > D and scan.region['fleft'] > D and scan.region['fright'] < D:
             
                 speed.linear.x = 0.2
                 speed.angular.z = 0
                
             
            if scan.region['front'] > D and scan.region['fleft'] > D and scan.region ['fright'] > D:             
                 speed.linear.x = 0
                 speed.angular.z= -0.2

            print "current state: ", state_dict[state]

            print "closest point: ", closest_point

            print "hit point: ", hit_point

            print "Current to goal: ",dist_diff

            print "Closest point to goal: ",distance_goal

 

        count = count + 1
        if count == 20:
            count_state = count_state + 1
            count = 0

        print scan.region
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
