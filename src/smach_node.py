#!/usr/bin/env python

import rospy
import smach
from smach import CBState
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import math
import smach_ros

GLOBAL_RADIUS = 1
nav2pit_pub = rospy.Publisher('/smachine/nav2wp', Odometry, queue_size = 1)


#check the requirement of using self with userdata, it is a smach thing so it is possible that you do not need to do that 	
class nav2PIT(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['wp_2_pit','counter_wp_2_pit','illumination_start_time'],
						output_keys=['counter_wp_2_pit','illumination_start_time'],
						outcomes=['reached_pit','mission_ongoing','failed'])	
		self.success_flag = False
		self.mission_flag = False
		  

	def position_cb(self,msg,userdata,argc):
 	 	wp = argc[0]
 	 	error = math.sqrt((msg.pose.pose.position.x - wp.pose.pose.position.x)**2 + (msg.pose.pose.position.y - wp.pose.pose.position.y)**2)
		# rospy.loginfo("the error is this bro-------- %d", error)
		if(error<GLOBAL_RADIUS):
			self.success_flag = True
			userdata.counter_wp_2_pit += 1
			if(userdata.counter_wp_2_pit == len(userdata.wp_2_pit))
				self.mission_flag = True
			# if last flag is reached set mission flag to true
 			 

	def global_wp_nav(self,userdata,nav2pit_pub):
		#publisg points from csv and get x,y
		msg = Odometry()
		rospy.loginfo("Publishing")
		msg.pose.pose.position.x = userdata.wp_2_pit[userdata.counter_wp_2_pit][0]#x
		msg.pose.pose.position.y = userdata.wp_2_pit[userdata.counter_wp_2_pit][1]#y
 		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		rospy.loginfo('Generating Waypoints and Navigating to the pit') 
		wp = self.global_wp_nav(nav2pit_pub)
		self.success_flag = False
 	 	nav2pit_sub = rospy.Subscriber("/smachine/nav2wp", Odometry, self.position_cb, (wp,self.success_flag))
 	 	rate = rospy.Rate(1)
		rate.sleep()
		if self.mission_flag:
			userdata.illumination_start_time = rospy.get_rostime()
			return 'reached_pit'
		return 'mission_ongoing'



class circum_wp_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=['counter_wp_around_pit','wp_around_pit','illumination_start_time'],
					output_keys=['counter_wp_around_pit','illumination_start_time'],
					outcomes=['reached_vantage_zone','mission_ongoing','failed','MISSION_COMPLETE'])
		self.success_flag = False
		self.mission_flag = False


	def position_cb(self, msg, argc):
 	 	wp = argc[0]
 	 	error = math.sqrt((msg.pose.pose.position.x - wp.pose.pose.position.x)**2 + (msg.pose.pose.position.y - wp.pose.pose.position.y)**2)
		# rospy.loginfo("the error is this bro-------- %d", error)
		if(error<GLOBAL_RADIUS):
			self.success_flag = True
			#userdata.wp_around_pit += 1
			if(userdata.counter_wp_around_pit == len(userdata.wp_around_pit))
				self.mission_flag = True
			# if last flag is reached set mission flag to true
 			 

	def global_wp_nav(self,nav2pit_pub, userdata):
		#get waypoint
		msg = Odometry()
		rospy.loginfo("Publishing")
		current_time = rospy.get_rostime()
		while(current_time - userdata.illumination_start_time >= userdata.wp_around_pit[userdata.counter_wp_around_pit][0]){
			userdata.counter_wp_around_pit+=1;
		}
		msg.pose.pose.position.x = userdata.wp_around_pit[userdata.counter_wp_around_pit][1]#x
		msg.pose.pose.position.y = userdata.wp_around_pit[userdata.counter_wp_around_pit][2]#y
 		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		rospy.loginfo('Generating Waypoints and Navigating to the pit') 
		wp = self.global_wp_nav(,)
		self.success_flag = False
 	 	nav2pit_sub = rospy.Subscriber("/smachine/nav2wp", Odometry, self.position_cb, (wp,self.success_flag))
 	 	rate = rospy.Rate(1)
		rate.sleep()
		if self.mission_flag:
			return 'MISSION_COMPLETE'
		else if self.success_flag:
			return 'reached_vantage_zone'

		return 'mission_ongoing'

		# use mission flag to find out if all the things 


class reach_edge_cb(smach.State):

	def __init__(self):
		smach.State.__init__(self,input_keys=[ ],
					output_keys=[ ],
					outcomes=['reached_edge','mission_ongoing','failed'])
		self.success_flag = False
		self.mission_flag = False
		self.mission_failure = False


	def position_cb(self,msg, argc):
 	 	wp = argc[0]
 	 	error = math.sqrt((msg.pose.pose.position.x - wp.pose.pose.position.x)**2 + (msg.pose.pose.position.y - wp.pose.pose.position.y)**2)
		# rospy.loginfo("the error is this bro-------- %d", error)
		if(error<GLOBAL_RADIUS):
			self.success_flag = True
			# if last flag is reached set mission flag to true
 			 	

	def global_wp_nav(self,nav2pit_pub,wp_gen):
		#service call to ayush's function to get way point
		msg = Odometry()
		rospy.loginfo("Publishing")
		msg.pose.pose.position.x = wp_gen.x#x
		msg.pose.pose.position.y = wp_gen.y#y
		#msg.pose.pose.position.z = 3#z
 		nav2pit_pub.publish(msg)
		return msg


	def execute(self,userdata):
		rospy.loginfo('Generating Waypoints and Navigating to the pit') 
		
		self.success_flag = False
		rospy.wait_for_service('some name for the thing')
		nav2pit_serv = rospy.ServiceProxy('some name for the thing', waypoints)
 	 	try:
  			wp_gen = nav2pit_serv()
		except rospy.ServiceException as exc:
  			print("Service did not process request: " + str(exc))
 	 	#rate = rospy.Rate(1)
		#rate.sleep()
		wp = self.global_wp_nav(nav2pit_pub,wp_gen)
 		if self.mission_flag:
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
		header = next(reader)
		for row in reader:
			tmp = []
			for elem in row:
				tmp.append(int(elem) * resolution + offset)
			wp.append(tmp)
	
	csvFile.close()
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
	
	csvFile.close()
	return wp


def main():
	rospy.init_node('smach_nodelet')
	sm = smach.StateMachine(outcomes=['Mission_completed_succesfully','Mission_aborted'])
	sm.userdata.q = 5 # size of the array given

	sm.userdata.counter_wp_2_pit = 0
	sm.userdata.counter_wp_around_pit = 0
	sm.userdata.wp_2_pit = read_csv(filename)
	sm.userdata.wp_around_pit = read_csv(filename2)
	sm.userdata.illumination_start_time = 0

	with sm:

		smach.StateMachine.add('nav2PIT', nav2PIT(),#BState(nav2pit_cb),
								 transitions = {'reached_pit':'navAROUNDPIT','mission_ongoing':'nav2PIT' ,'failed':'Mission_aborted'})

		smach.StateMachine.add('navAROUNDPIT', circum_wp_cb(),#BState(nav2pit_cb),
						 transitions = {'reached_vantage_zone':'nav2EDGE', 'mission_ongoing':'navAROUNDPIT', 'failed':'Mission_aborted',
						 'MISSION_COMPLETE':'Mission_completed_succesfully'})

		smach.StateMachine.add('nav2EDGE', reach_edge_cb(),#BState(nav2pit_cb),
				 transitions = {'reached_edge':'navAROUNDPIT', 'mission_ongoing':'nav2EDGE','failed':'navAROUNDPIT'})

			 
	 
	 
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()



if __name__== "__main__":
	main()





 