#ifndef __LOCAL_GOAL_CREATOR_H__
#define __LOCAL_GOAL_CREATOR_H__

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        // callbacks
        void checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void node_edge_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr& msg);
        void current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void stop_node_id_list_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);

        // other functions
        void get_node2node_poses(int node0_id, int node1_id, std::vector<geometry_msgs::PoseStamped>& node2node_poses);
        bool reached_checkpoint(int current_checkpoint_id_, int next_checkpoint_id, int one_more_ahead_checkpoint_id, geometry_msgs::PoseStamped current_pose);
        geometry_msgs::PoseStamped get_local_goal(std::vector<geometry_msgs::PoseStamped> &node2node_poses, int &poses_index, geometry_msgs::PoseStamped current_pose);
        bool reached_goal(int goal_node_id, geometry_msgs::PoseStamped current_pose);
        bool reached_stop_node(int next_node_id, std::vector<int> &stop_node_id_list, geometry_msgs::PoseStamped current_pose);

        // private params
        int hz_;
        int start_node_;
        int goal_node_;
        double local_goal_interval_;
        double local_goal_dist_;
        double stop_radius_min_;
        std::string local_goal_frame_id_;

        // other params
        bool checkpoint_received_;
        bool stop_node_id_list_received_;
        bool node_edge_map_received_;
        bool current_pose_updated_;
        std_msgs::Int32MultiArray checkpoint_;
        amsl_navigation_msgs::NodeEdgeMap node_edge_map_;
        geometry_msgs::PoseStamped current_pose_;
        int current_checkpoint_id_;
        int next_checkpoint_id_;
        int one_more_ahead_checkpoint_id_;
        std::vector<int> node_id_list_;
        std::vector<geometry_msgs::PoseStamped> local_goal_poses_;
        int local_goal_index_;
        geometry_msgs::PoseStamped local_goal_;
        geometry_msgs::PoseStamped local_goal_base_link_;
        std::vector<int> stop_node_id_list_;

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber checkpoint_sub_;
        ros::Subscriber node_edge_sub_;
        ros::Subscriber current_pose_sub_;
        ros::Subscriber stop_node_id_list_sub_;
        ros::Publisher local_goal_pub_;
        ros::Publisher current_checkpoint_id_pub_;

        // tf
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener *tf_listener_;
};

#endif // __LOCAL_GOAL_CREATOR_H__