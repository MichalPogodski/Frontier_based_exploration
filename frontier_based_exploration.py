#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from copy import copy

map_cpy = OccupancyGrid()
frontiers_groups = []
# free 0, obstacle 100, unknown -1


def get_pos_xy(marker, col, row):
    mul = 1. / map.info.resolution
    x = int(col * mul)
    y = int(row * mul)
    return x, y



def newFrontier(col, row, map):
    global frontiers_groups
    if map.data[col + row * map.info.width] == 0 and map.data[col + row * map.info.width] not in frontiers_groups:
        for i in range(-1, 2):
            for j in range(-1, 2):
                if (0 <= col + i < map.info.width) and (0 <= row + j < map.info.height):
                    if map.data[i + j * map.info.width] == -1:
                        return True

    return False



def checkSets(col, row, map):
    global frontiers_groups
    neig_pos = [(-1, 0), (0, -1), (1, 0), (0, 1)]

    for i, n in enumerate(neig_pos):
        n_row,n_col = row + n[0], col + n[1]
        existed_frontiers = []
        for j, group in enumerate(frontiers_groups):
            if map.data[n_col + n_row * map.info.width] in group:
                existed_frontiers.append(j)
        if len(existed_frontiers) == 1:
            frontiers_groups[existed_frontiers[0]].append(frontiers_groups[existed_frontiers[1:]])
        elif len(existed_frontiers) != 0:
            frontiers_groups[existed_frontiers[0]].append([col, row])
            frontiers_groups.delete[existed_frontiers[1:]]
        else:
            frontiers_groups.append([[col, row]])



def findFrontiers(map):

    for col in range(map.info.width):
        for row in range(map.info.height):
            if newFrontier(col, row, map):
                checkSets(col, row, map)
    return True


def calculateNewGoal():
    if len(frontiers_groups) == 0:
        print('EXPLORATION FINISHED!')
    else:
        toExplore = max(frontiers_groups, key=len)
        for loc in toExplore:
            print(loc)
        # jezeli instnieja to znajdz najdluzszy
        # wybierz jego element najblizszy robotowi
        # wystaw ten punkt
        # wyczysc frontiersy ?
        
# ZADAJ DOJAZD



def callback(data):
    map = copy(data)
    map.data = list(map.data)

    finished = findFrontiers(map)
    if finished: calculateNewGoal()



def mapListener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)
    pub = rospy.Publisher('map_copy', OccupancyGrid, queue_size=10)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(map_cpy)
        rate.sleep()


if __name__ == '__main__':
    mapListener()
