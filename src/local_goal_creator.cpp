#include "local_goal_creator/local_goal_creator.h"
#include <iostream>
LocalGoalCreator::LocalGoalCreator():private_nh("~"){
    private_nh.param("hz",hz,{10});
    private_nh.param("goal_dis",goal_dis,{2.0});
    private_nh.param("add_local_goal_index",add_local_goal_index,{2});
    sub_global_path = nh.subscribe("waypoint",10,&LocalGoalCreator::global_path_callback,this);
    sub_current_pose = nh.subscribe("amcl_pose",10,&LocalGoalCreator::current_pose_callback,this);
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("check_point",1);
    pub_final_goal = nh.advertise<geometry_msgs::PoseStamped>("final_goal",1);
}

void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    /* if(!get_global_path){ */
        global_path = *msg;
        if(global_path.poses.size() !=0)
        {
            local_goal = global_path.poses[goal_number];
            get_global_path = true;
        }
    /* } */
}

void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgs)
{
    current_pose = *msgs;
}

void LocalGoalCreator::calc_dis(const float& x1, const float& y1, const float& x2, const float& y2)
{
    distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void LocalGoalCreator::set_next_goal()
{
    float cx = current_pose.pose.pose.position.x;
    float cy = current_pose.pose.pose.position.y;
    count = 0;
    max_distance = 1e5;

    geometry_msgs::PoseStamped temporay_goal;
    temporay_goal.pose.position.x = 0.0;
    temporay_goal.pose.position.y = 0.0;

    for(auto& point : global_path.poses)
    {
        float x = point.pose.position.x;
        float y = point.pose.position.y;

        calc_dis(x, y, cx, cy);
        if(max_distance > distance)
        {
            max_distance = distance;
            min_index = count;
        }
        std::cout << min_index << std::endl;

    }

    if(min_index + add_local_goal_index > global_path.poses.size() -1)
    {
        reach_final_goal = true;
        local_goal = global_path.poses[global_path.poses.size() -1];
        final_goal = global_path.poses[global_path.poses.size() -1];
    }
    else
    {
        local_goal = global_path.poses[min_index + add_local_goal_index];
        final_goal = temporay_goal;
    }

    local_goal.header.frame_id = "map";
    if(!reach_final_goal) pub_local_goal.publish(local_goal);
    else
    {
        local_goal = global_path.poses[global_path.poses.size() -1];
        pub_local_goal.publish(local_goal);
        pub_final_goal.publish(final_goal);
    }
}

void LocalGoalCreator::process(){
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(get_global_path){
            set_next_goal();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator LocalGoalCreator;
    LocalGoalCreator.process();
}

