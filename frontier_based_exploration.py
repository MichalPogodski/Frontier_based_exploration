#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from copy import copy

map_cpy = OccupancyGrid()

def callback(data):
    map = copy(data)
    map.data = list(map.data)

    for n in map.data:
        if n != -1:
            print(n.par)


def mapListener():
   rospy.init_node('map_listener', anonymous=True)
   rospy.Subscriber('map', OccupancyGrid, callback)
   pub = rospy.Publisher('map_copy', OccupancyGrid, queue_size=10)

   rate = rospy.Rate(1)  # 1hz
   while not rospy.is_shutdown():
       pub.publish(map_cpy)
       rate.sleep()


if __name__ == '__main__':
   mapListener()



