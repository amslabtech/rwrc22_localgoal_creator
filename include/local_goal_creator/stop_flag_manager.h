#ifndef __STOP_FLAG_MANAGER_H__
#define __STOP_FLAG_MANAGER_H__

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>

class StopFlagManager
{
    public:
        StopFlagManager();
        void process();

    private:
        // callbacks
        void current_node_callback(const std_msgs::Int32::ConstPtr& msg);
        void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

        // other functions
        void get_stop_node_id_list(XmlRpc::XmlRpcValue &stop_list, std::vector<int> &stop_node_id_list);
        bool is_stop_node(int node_id);
        bool get_go_signal(sensor_msgs::Joy &joy);

        // private params
        int hz_;
        XmlRpc::XmlRpcValue stop_list_;

        // other params
        bool current_node_received_;
        bool joy_received_;
        std::vector<int> stop_node_id_list_;
        int current_node_;
        sensor_msgs::Joy joy_;
        bool stop_flag_;

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber current_node_sub_;
        ros::Subscriber joy_sub_;
        ros::Publisher stop_flag_pub_;
        ros::Publisher stop_node_id_list_pub_;
};

#endif // __STOP_FLAG_MANAGER_H__