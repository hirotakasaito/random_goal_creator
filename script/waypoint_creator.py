#!/usr/bin/python3

import math
import json
import random
import rospy
import copy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped,PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from node_astar import *

class LocalGoalCreator:
    def __init__(self):
        rospy.init_node('WaypointCreator', anonymous=True)

        # param
        self.HZ = rospy.get_param("~HZ", 10)
        self.WORLD_FRAME = rospy.get_param("~WORLD_FRAME", 'map')
        self.ROBOT_FRAME = rospy.get_param("~ROBOT_FRAME", 'base_link')
        # self.GOAL_DIS_TOLERANCE = rospy.get_param("~GOAL_DIS_TOLERANCE", 0.3)
        # self.GOAL_YAW_TOLERANCE = rospy.get_param("~GOAL_YAW_TOLERANCE", 1.0)
        # self.TIMEOUT = rospy.get_param("~TIMEOUT", 180)

        self.estimated_pose_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.estimated_pose_call_back)
        # self.global_goal_sub = rospy.Subscriber('/goal_reach',Bool,self.goal_reach_call_back)

        WAYPOINTS_PATH = rospy.get_param("~WAYPOINTS_PATH",'/home/amsl/catkin_ws/src/global_path_creator/waypoints/waypoints.json')
        with open(WAYPOINTS_PATH) as f:
            waypoints_data = json.load(f)
        self.waypoints = []
        for wp in waypoints_data["WAYPOINTS"]:
            self.waypoints.append([wp["x"], wp["y"], wp["yaw"]])
        self.idx = 0

        # publisher
        self.waypoint_pub = rospy.Publisher('/waypoint', Path, queue_size=10)

        self.start_time = rospy.Time.now()
        self.goal_reach = True
        self.estimated_pose = PoseWithCovarianceStamped()
        self.global_goal_x = 0
        self.global_goal_y = 0

        self.maze = [[0, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0],
                    [0, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0],
                    [0, 0, 0, 0, 0]]
        self.split = 10
        self.set_node()
        self.global_path = Path()
    def set_node(self):
        # 218
        # 307
        # 456
        id0 = (2,2)
        id1 = (2,4)
        id2 = (0,4)
        id3 = (0,2)
        id4 = (0,0)
        id5 = (2,0)
        id6 = (0,4)
        id7 = (4,2)
        id8 = (4,4)
        self.id_list = [id0,id1,id2,id3,id4,id5,id6,id7,id8]

    def goal_callabck(self, data):
        self.goal = data
        print("next goal: ")
        print(self.goal)
        self.start_time = rospy.Time.now()

    # def goal_reach_call_back(self,msg):
        # self.goal_reach = msg.data

    def estimated_pose_call_back(self,msg):

        self.estimated_pose = msg

    def next_waypoint(self):
        next_waypoint = PoseStamped()
        next_waypoint.header.frame_id = self.WORLD_FRAME
        ex = self.estimated_pose.pose.pose.position.x
        ey = self.estimated_pose.pose.pose.position.y
        waypoint_dis = 0
        next_waypoint_id = 0
        min_waypoint_id = 0
        next_waypoint_list = []
        max_dis = 1e5
        min_waypoint = 0
        path_id = []
        waypoint_size = len(self.waypoints)

        for i in range(waypoint_size):
            waypoint_dis = math.sqrt((ex - self.waypoints[i][0]) ** 2 + (ey - self.waypoints[i][1]) ** 2)
            if waypoint_dis > 15.0:
                next_waypoint_list.append(i)
            if waypoint_dis < max_dis:
                max_dis = waypoint_dis
                next_waypoint_id = i
            if waypoint_dis > min_waypoint:
                min_waypoint = waypoint_dis
                min_waypoint_id = i

        if len(next_waypoint_list) == 0:
            next_waypoint.pose.position.x = self.waypoints[next_waypoint_id][0]
            next_waypoint.pose.position.y = self.waypoints[next_waypoint_id][1]
        else:
            next_waypoihnt_id = random.choice(next_waypoint_list)
            next_waypoint.pose.position.x = self.waypoints[next_waypoihnt_id][0]
            next_waypoint.pose.position.y = self.waypoints[next_waypoihnt_id][1]

        path = astar(self.maze,self.id_list[min_waypoint_id],self.id_list[next_waypoint_id])
        # print(path)
        for i in path:
            for idx,id in enumerate(self.id_list):
                if id == i:
                    path_id.append(idx)
        # if(self.reach_goal):
        self.local_goal_creator(path_id)

    def local_goal_creator(self,path_id):
        self.global_path = Path()
        path_id.reverse()
        path_point = PoseStamped()
        path_point.header.frame_id = "map"
        for i in range(len(path_id)-1):
            x1 = self.waypoints[path_id[i]][0]
            x2 = self.waypoints[path_id[i+1]][0]
            y1 = self.waypoints[path_id[i]][1]
            y2 = self.waypoints[path_id[i+1]][1]

            if (x2 - x1) != 0:
                a = (y2 - y1) / (x2 - x1)

            # else:
                # a = x1
            b = y1 - a*x1
            split = (x2 - x1)/self.split
            path_point.pose.position.x = self.waypoints[path_id[i]][0]
            path_point.pose.position.y = self.waypoints[path_id[i]][1]
            self.global_path.poses.append(copy.deepcopy(path_point))
            add_split = split
            for i in range(self.split):
                y = a * (add_split + x1) + b
                x = add_split + x1
                path_point.pose.position.x = x
                path_point.pose.position.y = y
                self.global_path.poses.append(copy.deepcopy(path_point))
                add_split += split
        path_point.pose.position.x = self.waypoints[path_id[-1]][0]
        path_point.pose.position.y = self.waypoints[path_id[-1]][1]
        self.global_path.poses.append(copy.deepcopy(path_point))
        self.global_path.header.frame_id = "map"
        # print(global_path.poses)
        # self.waypoint_pub.publish(global_path)
        self.global_goal_x = path_point.pose.position.x
        self.global_goal_y = path_point.pose.position.y

    def check_goal_reach(self):
        cx = self.estimated_pose.pose.pose.position.x
        cy = self.estimated_pose.pose.pose.position.y
        gx = self.global_goal_x
        gy = self.global_goal_y
        dis = math.sqrt((gx - cx) ** 2 + (gy -cy) ** 2)
        # print(dis)
        if dis < 1.0:
            self.goal_reach = True
        else:
            self.goal_reach = False

    def process(self):
        r = rospy.Rate(self.HZ)
        while not rospy.is_shutdown():
            if self.goal_reach:
                self.next_waypoint()
            self.check_goal_reach()
            self.waypoint_pub.publish(self.global_path)
            r.sleep()

if __name__ == '__main__':
    local_goal_creator = LocalGoalCreator()
    try:
        local_goal_creator.process()
    except rospy.ROSInterruptException:
        pass
