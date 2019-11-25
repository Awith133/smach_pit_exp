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


GLOBAL_RADIUS = 3
GLOBAL_RADIUS2 = 0.5
TIME_OUT = 600
file_to_pit = rospy.get_param("file_to_pit")
FILE_TO_PIT = file_to_pit
file_around_pit = rospy.get_param("file_around_pit")
FILE_AROUND_PIT = file_around_pit

nav2pit_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
pit_edge_dist_pub = rospy.Publisher('/robot_at_edge_position', Odometry, queue_size = 1)


#CLASS 1
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
			error = 0
		if(error<GLOBAL_RADIUS or userdata.counter_wp_2_pit == -1):
			userdata.counter_wp_2_pit += 1
			if(userdata.counter_wp_2_pit == len(userdata.wp_2_pit)):
				self.mission_flag = True
				return
			self.wp = self.global_wp_nav(userdata,nav2pit_pub)

	def global_wp_nav(self,userdata,nav2pit_pub):
		#publish points from csv and get x,y
		msg = PoseStamped()
		msg.pose.position.x = userdata.wp_2_pit[userdata.counter_wp_2_pit][1]#x
		msg.pose.position.y = userdata.wp_2_pit[userdata.counter_wp_2_pit][0]#y
		msg.pose.orientation.w = 1
		msg.header.frame_id = 'map'
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y )

		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		rate = rospy.Rate(5)
		rate.sleep()
		if self.mission_flag:
			sm.userdata.illumination_start_time = rospy.get_rostime().secs
			return 'reached_pit'
		return 'mission_ongoing'

#CLASS 2
class circum_wp_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['counter_wp_around_pit','wp_around_pit','illumination_start_time','towards_edge_time_start'],
					output_keys=['counter_wp_around_pit','illumination_start_time','towards_edge_time_start'],
					outcomes=['reached_vantage_zone','mission_ongoing','failed','MISSION_COMPLETE'])
		self.success_flag = False
		self.mission_flag = False

	def position_cb(self, msg, argc):
		x_pose = msg.polygon.points[0].x
		y_pose = msg.polygon.points[0].y
		userdata = argc[0]
		if(userdata.counter_wp_around_pit != -1):
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
		else:
			error = 0

		if(error<GLOBAL_RADIUS or userdata.counter_wp_around_pit == -1):
			if (userdata.counter_wp_around_pit>-1):
				if(userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 2):
					self.mission_flag = True
					return
				if(userdata.wp_around_pit[userdata.counter_wp_around_pit][0] - rospy.get_rostime().secs>0):
					return
				if (userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == 1 ):
					if not (current_time - userdata.illumination_start_time >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]): 
						self.success_flag = True
						return
			userdata.counter_wp_around_pit += 1
			while((rospy.get_rostime().secs - userdata.illumination_start_time) >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]):
				userdata.counter_wp_around_pit += 1
			while(userdata.wp_around_pit[userdata.counter_wp_around_pit][3] == -1):
				if(userdata.wp_around_pit[userdata.counter_wp_around_pit][0] - rospy.get_rostime().secs<0):
					userdata.counter_wp_around_pit += 1
			#last way 
			if(userdata.counter_wp_around_pit >= len(userdata.wp_around_pit)):
				self.mission_flag = True
				return
			self.wp = self.global_wp_nav(nav2pit_pub,userdata)


	def global_wp_nav(self,nav2pit_pub, userdata):
		msg = PoseStamped()
		rospy.loginfo("Publishing")
		current_time = rospy.get_rostime().secs

		# while((current_time - userdata.illumination_start_time) >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]):
		# 	userdata.counter_wp_around_pit+=1
		# 	return
		msg.pose.position.x = userdata.wp_around_pit[userdata.counter_wp_around_pit][2]
		msg.pose.position.y = userdata.wp_around_pit[userdata.counter_wp_around_pit][1]
		msg.pose.orientation.w = 1
		msg.header.frame_id = 'map'
		print('Publishing wp', msg.pose.position.x , msg.pose.position.y )
		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.success_flag))
		rate = rospy.Rate(10)
		rate.sleep()
		if self.mission_flag:
			return 'MISSION_COMPLETE'
		elif self.success_flag:
			self.success_flag = False
			sm.userdata.towards_edge_time_start = rospy.get_rostime().secs
			return 'reached_vantage_zone'
		return 'mission_ongoing'


#CLASS 3
class reach_edge_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['towards_edge_time_start' ],
					output_keys=[ ],
					outcomes=['reached_edge','mission_ongoing','failed'])
		self.gen_first_flag = True
		self.mission_flag = False
		self.mission_failure = False


	def position_cb(self,msg, argc):
		x_pose = msg.polygon.points[0].x
		y_pose = msg.polygon.points[0].y
		userdata = argc[0]
		if self.gen_first_flag:
			error = 0
		else:
			error = math.sqrt((x_pose - self.wp.pose.position.x)**2 + (y_pose - self.wp.pose.position.y)**2)
			rospy.wait_for_service('gen_wp2pit')
			nav2pit_serv = rospy.ServiceProxy('gen_wp2pit', waypoints)
	 	 	try:
	  			self.wp_gen = nav2pit_serv()
	  			self.mission_flag = self.wp_gen.mission_flag
	  			if (self.mission_flag):
	  				msg_odom = Odometry()
					msg_odom.header.frame_id = 'map'
					msg_odom.pose.pose.position.x = -1*y_pose
					msg_odom.pose.pose.position.y = x_pose
					pit_edge_dist_pub.publish(msg_odom)

				self.mission_failure  = not self.wp_gen.wp_received
	  			print('self.wp_gen.mission_flag', self.wp_gen.mission_flag)#( self.wp_gen.mission_flag or self.wp_gen.wp_received)
	  			if (self.wp_gen.wp_received):# and not self.mission_flag):
						self.wp = self.global_wp_nav(nav2pit_pub,self.wp_gen)
						self.gen_first_flag = False
			except rospy.ServiceException as exc:
	  			print("Service did not process request: " + str(exc))

					

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
		rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.position_cb, (userdata,self.gen_first_flag))
		rate = rospy.Rate(5)
		rate.sleep()
		current_time = rospy.get_rostime().secs
		if(userdata.towards_edge_time_start - current_time > TIME_OUT):
			return 'failed'
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
			i = 0
			for elem in row:
				if(i == 0):
					tmp.append(-1*int(elem) * resolution + offset)
				else:
					tmp.append(int(elem) * resolution + offset)
				i+=1
			wp.append(tmp)
	return wp

def read_csv_with_time(filename):
	wp = []
	map_offset = 2.5
	map_resolution = 5
	time_resolution = 700
	time_offset = 0
	with open(filename, 'rb') as f:
		reader = csv.reader(f, delimiter=',')
		header = next(reader)
		for row in reader:
			tmp = []
			i = 0
			for elem in row:
				if (i == 0):
					tmp.append(int(elem) * time_resolution + time_offset)
				elif (i==1):
					tmp.append(-1*int(elem) * map_resolution + map_offset)
				elif (i == 2):
					tmp.append(int(elem) * map_resolution + map_offset)
				else:
					tmp.append(int(elem))
				i+=1
			wp.append(tmp)

	return wp


def main():
	rospy.init_node('smach_nodelet')
	sm = smach.StateMachine(outcomes=['Mission_completed_succesfully','Mission_aborted'])
	sm.userdata.q = 5 # size of the array given

	sm.userdata.counter_wp_2_pit = -1
	sm.userdata.counter_wp_around_pit = -1
	sm.userdata.wp_2_pit = read_csv(FILE_TO_PIT)
	sm.userdata.current_wp = sm.userdata.wp_2_pit[0]
	sm.userdata.wp_around_pit = read_csv_with_time(FILE_AROUND_PIT)
	sm.userdata.illumination_start_time = 0
	sm.userdata.illumination_start_time = rospy.get_rostime().secs
	sm.userdata.towards_edge_time_start = 0
	# print("sm.userdata.illumination_start_time ", sm.userdata.illumination_start_time )
	# print()
	with sm:

		smach.StateMachine.add('nav2PIT', nav2PIT(),#BState(nav2pit_cb),
								 transitions = {'reached_pit':'navAROUNDPIT','mission_ongoing':'nav2PIT' ,'failed':'Mission_aborted'})

		smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(nav2pit_cb),
						 transitions = {'reached_vantage_zone':'nav2EDGE', 'mission_ongoing':'navAROUNDPIT', 'failed':'Mission_aborted',
						 'MISSION_COMPLETE':'Mission_completed_succesfully'})
		
		smach.StateMachine.add('nav2EDGE', reach_edge_cb(), #BState(nav2pit_cb),
				 transitions = {'reached_edge':'nav2EDGE', 'mission_ongoing':'nav2EDGE','failed':'nav2EDGE'})




		# smach.StateMachine.add('nav2EDGE', reach_edge_cb(), #BState(nav2pit_cb),
		# 		 transitions = {'reached_edge':'Mission_aborted', 'mission_ongoing':'nav2EDGE','failed':'Mission_aborted'})

		# smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(nav2pit_cb),
		# 				 transitions = {'reached_vantage_zone':'Mission_aborted', 'mission_ongoing':'navAROUNDPIT', 'failed':'Mission_aborted',
		# 				 'MISSION_COMPLETE':'Mission_completed_succesfully'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__== "__main__":
	main()
