#include "local_goal_creator/stop_flag_manager.h"

StopFlagManager::StopFlagManager() : nh_(), private_nh_("~")
{
    private_nh_.param("hz", hz_, 10);
    private_nh_.getParam("stop_list", stop_list_);

    current_node_sub_ = nh_.subscribe("/current_node", 1, &StopFlagManager::current_node_callback, this);
    joy_sub_ = nh_.subscribe("/joy", 1, &StopFlagManager::joy_callback, this);
    stop_flag_pub_ = nh_.advertise<std_msgs::Bool>("/stop_flag", 1);
    stop_node_id_list_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/stop_node_id_list", 1);

    get_stop_node_id_list(stop_list_, stop_node_id_list_);
    current_node_received_ = false;
    joy_received_ = false;
}

void StopFlagManager::current_node_callback(const std_msgs::Int32::ConstPtr &msg)
{
    current_node_ = (int)msg->data;
    current_node_received_ = true;
}

void StopFlagManager::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
    joy_ = *msg;
    joy_received_ = true;
}

void StopFlagManager::get_stop_node_id_list(XmlRpc::XmlRpcValue &stop_list, std::vector<int> &stop_node_id_list_)
{
    stop_node_id_list_.clear();
    for (int i = 0; i < stop_list.size(); i++)
    {
        int stop_node_id = stop_list[i]["node_id"];
        stop_node_id_list_.push_back(stop_node_id);
        ROS_INFO("stop_node_id: %d", stop_node_id);
    }
    ROS_WARN("stop_node_id_list_: %d", (int)stop_node_id_list_.size());
}

bool StopFlagManager::is_stop_node(int node_id)
{
    current_node_received_ = false;
    if (stop_node_id_list_.size() == 0) return false;
    // if (find(stop_node_id_list_.begin(), stop_node_id_list_.end(), node_id) != stop_node_id_list_.end())
    if(stop_node_id_list_[0] == node_id)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool StopFlagManager::get_go_signal(sensor_msgs::Joy &joy)
{
    if(joy_received_ == false)
    {
        return false;
    }

    if (joy.buttons[11] == 1 && joy.buttons[12] == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void StopFlagManager::process()
{
    ros::Rate loop_rate(hz_);
    while (ros::ok())
    {
        if (current_node_received_)
        {
            bool stop_node_flag = is_stop_node(current_node_);
            bool go_signal_flag = get_go_signal(joy_);
            if (stop_node_flag)
            {
                if (go_signal_flag)
                {
                    // std::vector<int>::iterator it = find(stop_node_id_list_.begin(), stop_node_id_list_.end(), current_node_);
                    // stop_node_id_list_.erase(it);
                    stop_node_id_list_.erase(stop_node_id_list_.begin());
                    for (int i = 0; i < stop_node_id_list_.size(); i++)
                        ROS_INFO("stop_node_id: %d", stop_node_id_list_[i]);
                    ROS_WARN("stop_node_id_list_: %d", (int)stop_node_id_list_.size());

                    std_msgs::Bool stop_flag_msg;
                    stop_flag_msg.data = false;
                    stop_flag_pub_.publish(stop_flag_msg);
                }
                else
                {
                    std_msgs::Bool stop_flag_msg;
                    stop_flag_msg.data = true;
                    stop_flag_pub_.publish(stop_flag_msg);
                }
            }
            current_node_received_ = false;
            joy_received_ = false;
        }
        std_msgs::Int32MultiArray stop_node_id_list_msg;
        for(auto &node_id : stop_node_id_list_) stop_node_id_list_msg.data.push_back(node_id);
        stop_node_id_list_pub_.publish(stop_node_id_list_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
