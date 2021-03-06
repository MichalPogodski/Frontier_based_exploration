#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from tf.transformations import quaternion_from_euler
from copy import copy

map = OccupancyGrid()
goal = PoseStamped()
collected = False
exploring = False
frontiers_groups = []
status = None
map_height = 0
map_width = 0

# def get_marker_xy(pos):
#     mul = 1. / 0.05
#     return int(pos * mul)


def newFrontier(col, row, map):
    print('1')
    global frontiers_groups
    isNew = False                                       #########################################################
    if map.data[col + row * map.info.width] == 0:
        # if map.data[col + row * map.info.width] not in frontiers_groups:
        for i in range(-1, 2):
            for j in range(-1, 2):
                if (0 <= col + i < map.info.width) and (0 <= row + j < map.info.height):
                    if map.data[i + j * map.info.width] == -1:
                        isNew = True                                ####################################################
        return isNew                                                                ###########################


def findFrontiers(map):
    print('2')
    global frontiers_groups, exploring

    for col in range(map.info.width):
        for row in range(map.info.height):
            if newFrontier(col, row, map):
                # checkSets(col, row, map)
                if [col, row] not in frontiers_groups:
                    frontiers_groups.append([col, row])
                    print(len(frontiers_groups))
    return True


def calculateNewGoal():
    print('3')
    global goal,  frontiers_groups, exploring, map_width, map_height
    if len(frontiers_groups) == 0:
        print('EXPLORATION FINISHED!')
    else:
        goal.pose.position.x = -10.0 + (frontiers_groups[0][0] * 0.05) - 0.05/2
        goal.pose.position.y = -10.0 + (frontiers_groups[0][1] * 0.05) - 0.05/2
        goal.pose.orientation.w = 1.0
        # quaternion = quaternion_from_euler(1.0, 1.0, 1.0)
        # goal.pose.orientation.w = quaternion[3]

        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        print('#########################', frontiers_groups[0])

        del frontiers_groups[:]
        exploring = True
        pub.publish(goal)
        trigger_sender()



def map_callback(data):
    print('4')
    global map, collected, map_width, map_height,exploring

    map = copy(data)
    map.data = list(map.data)
    map_width, map_height = map.info.width, map.info.height
    if not exploring:
        print('4.2')
        collected = findFrontiers(map)
        print('4.3')
        if collected:
            print('4.4')
            calculateNewGoal()
            print('4.5')
            collected = False
    else:
        print('4.6')
        pass




def check_pose(cur_pose):
    print('5')
    #znacznie okrojona funkcja z moving_real
    global status, exploring
    status = cur_pose
    print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%', cur_pose.status_list[-1].status)
    if (cur_pose.status_list[-1].status == 3 or cur_pose.status_list[-1].status == 4):
        exploring = False
        print('10.0')



# def dest_achieved(arrived):
#     global exploring
#     if arrived:
#         exploring = False



print('6')
rospy.init_node('frontier_exploration', anonymous=True)
print('6.1')
# rospy.wait_for_service('/moveDemo/new_goal')
print('6.2')
trigger_sender = rospy.ServiceProxy('/moveDemo/new_goal', Trigger)
print('6.3')
rospy.Subscriber('map', OccupancyGrid, map_callback)
print('6.4')
rospy.Subscriber("/move_base/status", GoalStatusArray, check_pose)
print('6.5')
# rospy.Subscriber('/frontier_exploration/goal', PoseStamped, dest_achieved)
print('7')

pub = rospy.Publisher('frontier_exploration/new_goal', PoseStamped, queue_size=10)
rate = rospy.Rate(1)
print('8')
while not rospy.is_shutdown():
    rospy.spin()










