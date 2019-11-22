#!/usr/bin/env python

import csv
import rospy
import math
import smach
import pdb

import smach_ros

from smach import CBState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PolygonStamped
from waypoint_pit_planner.srv import waypoints


GLOBAL_RADIUS = 1
nav2pit_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)


#check the requirement of using self with userdata, it is a smach thing so it is possible that you do not need to do that
#CLASS 3
class nav2PIT(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_2_pit','counter_wp_2_pit','illumination_start_time'],
						output_keys=['counter_wp_2_pit','illumination_start_time'],
						outcomes=['reached_pit','mission_ongoing','failed'])
		self.success_flag = False
		self.mission_flag = False



	def position_cb(self,msg,argc):

		x_pose = msg.polygon.points[0].x
		y_pose = msg.polygon.points[0].y


		userdata = argc[0]
		if(userdata.counter_wp_2_pit != -1):
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
		else:
			error = 0;


		if(error<GLOBAL_RADIUS or userdata.counter_wp_2_pit == -1):
			# print("UserData position cb counter {0}".format(userdata.counter_wp_2_pit))
			userdata.counter_wp_2_pit += 1
			# print("self.mission_flag-----pdb.set_trace()error ", self.mission_flag, error)
			if(userdata.counter_wp_2_pit == len(userdata.wp_2_pit)):
				self.mission_flag = True
				return
			self.wp = self.global_wp_nav(userdata,nav2pit_pub)

	def global_wp_nav(self,userdata,nav2pit_pub):
		#publish points from csv and get x,y
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_2_pit[userdata.counter_wp_2_pit][0]#x
		msg.pose.position.y = userdata.wp_2_pit[userdata.counter_wp_2_pit][1]#y
		msg.pose.orientation.w = 1
		msg.header.frame_id = 'map'
		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		# print("UserData execute counter {0}".format(userdata.counter_wp_2_pit))
		rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		rate = rospy.Rate(5)
		rate.sleep()

		if self.mission_flag:
			userdata.illumination_start_time = rospy.get_rostime()
			return 'reached_pit'
		return 'mission_ongoing'

#CLASS 2
class circum_wp_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['counter_wp_around_pit','wp_around_pit','illumination_start_time'],
					output_keys=['counter_wp_around_pit','illumination_start_time'],
					outcomes=['reached_vantage_zone','mission_ongoing','failed','MISSION_COMPLETE'])
		self.success_flag = False
		self.mission_flag = False

	def position_cb(self, msg, argc):

		x_pose = msg.polygon.points[0].x
		y_pose = msg.polygon.points[0].y
		userdata = argc[0]
		if(userdata.counter_wp_2_pit != -1):
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
		else:
			error = 0

		if(error<GLOBAL_RADIUS or userdata.counter_wp_2_pit == -1):
			if (userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 1 ):
				if not (current_time - userdata.illumination_start_time >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]):
					self.success_flag = True
					return
			userdata.counter_wp_2_pit += 1
			if(userdata.counter_wp_2_pit == len(userdata.wp_2_pit)):
				self.mission_flag = True
				return
			self.wp = self.global_wp_nav(userdata,nav2pit_pub)


	def global_wp_nav(self,nav2pit_pub, userdata):
		#get waypoint
		msg = Odometry()
		rospy.loginfo("Publishing")
		current_time = rospy.get_rostime()
		while(current_time - userdata.illumination_start_time >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]):
			userdata.counter_wp_around_pit+=1

		msg.pose.pose.position.x = userdata.wp_around_pit[userdata.counter_wp_around_pit][1]#x
		msg.pose.pose.position.y = userdata.wp_around_pit[userdata.counter_wp_around_pit][2]#y
		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata))
		rate = rospy.Rate(5)
		rate.sleep()
		if self.mission_flag:
			return 'MISSION_COMPLETE'
		elif self.success_flag:
			self.success_flag = False
			return 'reached_vantage_zone'
		return 'mission_ongoing'

		# use mission flag to find out if all the things

#CLASS 3
class reach_edge_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=[ ],
					output_keys=[ ],
					outcomes=['reached_edge','mission_ongoing','failed'])
		self.gen_first_flag = True
		self.success_flag = False
		self.mission_flag = False
		self.mission_failure = False


	def position_cb(self,msg, argc):
 	 	x_pose = msg.polygon.points[0].x
		y_pose = msg.polygon.points[0].y
		userdata = argc[0]
		if self.gen_first_flag:
			print("here bro")
			error = 0
		else:
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
		if(error<GLOBAL_RADIUS):
			self.success_flag = True
			rospy.wait_for_service('gen_wp2pit')
			nav2pit_serv = rospy.ServiceProxy('gen_wp2pit', waypoints)
	 	 	try:
	  			self.wp_gen = nav2pit_serv()
	  			self.mission_flag = self.wp_gen.mission_flag
	  			self.mission_failure  = not self.wp_gen.wp_received
	  			print('self.wp_gen.mission_flag', self.wp_gen.mission_flag)#( self.wp_gen.mission_flag or self.wp_gen.wp_received)
	  			if (self.wp_gen.wp_received and not self.mission_flag):
						self.wp = self.global_wp_nav(nav2pit_pub,self.wp_gen)
						self.gen_first_flag = False
			except rospy.ServiceException as exc:
	  			print("Service did not process request: " + str(exc))


			# if last flag is reached set mission flag to true


	def global_wp_nav(self,nav2pit_pub,wp_gen):
		#service call to ayush's function to get way point
		msg = PoseStamped()

		msg.pose.position.x = wp_gen.x#x
		msg.pose.position.y = wp_gen.y#y
		print("Gen Points:", wp_gen.x, wp_gen.y)
		msg.pose.orientation.w = 1
		msg.header.frame_id = 'map'
		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		# rospy.loginfo('Generating Waypoints and Navigating to the pit')

		# self.success_flag = False
		# if (not self.gen_first_flag ):
		rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		rate = rospy.Rate(5)
		rate.sleep()

		# if (self.gen_first_flag ):

 		if self.mission_flag:
 			self.gen_first_flag = True
			return 'reached_edge'
		if self.mission_failure:
			return 'failed'
		return 'mission_ongoing'


def read_csv(filename):
	wp = []
	offset = 2.5
	resolution = 5
	with open(filename, 'rb') as f:
		reader = csv.reader(f, delimiter=',')
		for row in reader:
			print(row)
			tmp = []
			for elem in row:
				tmp.append(int(elem) * resolution + offset)
			wp.append(tmp)
	return wp

def read_csv_with_time(filename):
	wp = []
	map_offset = 2.5
	map_resolution = 5
	time_resolution = 700
	with open(filename, 'rb') as f:
		reader = csv.reader(f, delimiter=',')
		header = next(reader)
		for row in reader:
			tmp = []
			i = 0
			for elem in row:
				if (i == 0):
					tmp.append(int(elem) * time_resolution + time_offset)
				tmp.append(int(elem) * map_resolution + map_offset)
				i+=1
			wp.append(tmp)

	return wp


def main():
	rospy.init_node('smach_nodelet')
	sm = smach.StateMachine(outcomes=['Mission_completed_succesfully','Mission_aborted'])
	sm.userdata.q = 5 # size of the array given

	sm.userdata.counter_wp_2_pit = -1
	sm.userdata.counter_wp_around_pit = 0
	sm.userdata.wp_2_pit = read_csv('/home/ayush/mrsd_ws/src/smach_pit_exp/src/waypoints.csv')
	sm.userdata.current_wp = sm.userdata.wp_2_pit[0]
	# sm.userdata.wp_around_pit = read_csv('')
	sm.userdata.illumination_start_time = 0

	with sm:

		# smach.StateMachine.add('nav2PIT', nav2PIT(),#BState(nav2pit_cb),
		# 						 transitions = {'reached_pit':'Mission_aborted','mission_ongoing':'nav2PIT' ,'failed':'Mission_aborted'})

		# smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(nav2pit_cb),
		# 				 transitions = {'reached_vantage_zone':'nav2EDGE', 'mission_ongoing':'navAROUNDPIT', 'failed':'Mission_aborted',
		# 				 'MISSION_COMPLETE':'Mission_completed_succesfully'})

		smach.StateMachine.add('nav2EDGE', reach_edge_cb(), #BState(nav2pit_cb),
				 transitions = {'reached_edge':'Mission_aborted', 'mission_ongoing':'nav2EDGE','failed':'Mission_aborted'})

		# smach.StateMachine.add('nav2EDGE', reach_edge_cb(), #BState(nav2pit_cb),
		# 		 transitions = {'reached_edge':'navAROUNDPIT', 'mission_ongoing':'nav2EDGE','failed':'navAROUNDPIT'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__== "__main__":
	main()
