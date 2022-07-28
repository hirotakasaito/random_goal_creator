#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


class LocalGoalCreator{
    public:
        LocalGoalCreator();
        void process();
    private:
        void global_path_callback(const nav_msgs::Path::ConstPtr&);
        void current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
        void calc_dis(const float& x1,const float& y1,const float& x2,const float& y2);
        void set_next_goal();

        int hz;
        float goal_dis;
        int add_local_goal_index;
        float distance = 0;
        float goal_number = 0;
        int min_index = 0.0;
        int count = 0.0;
        float max_distance = 0.0;
        bool get_global_path = false;
        bool get_current_pose = false;
        bool reach_final_goal = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Publisher pub_local_goal;
        ros::Publisher pub_final_goal;
        ros::Subscriber sub_global_path;
        ros::Subscriber sub_current_pose;
        nav_msgs::Path global_path;
        geometry_msgs::PoseWithCovarianceStamped current_pose;
        geometry_msgs::PoseStamped local_goal;
        geometry_msgs::PoseStamped final_goal;
};
#endif
