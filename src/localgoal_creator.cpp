#include "localgoal_creator/localgoal_creator.h"

LocalGoalCreator::LocalGoalCreator() : nh_(),
                                       private_nh_("~")
{
    private_nh_.param("hz", hz_, 10);
    private_nh_.param("start_node", start_node_, 0);
    private_nh_.param("goal_node", goal_node_, 1);
    private_nh_.param("local_goal_interval", local_goal_interval_, 1.0);
    private_nh_.param("local_goal_dist", local_goal_dist_, 5.0);
    private_nh_.param("stop_radius_min", stop_radius_min_, 0.75);
    private_nh_.param("local_goal_frame_id", local_goal_frame_id_, std::string("base_link"));

    checkpoint_sub_ = nh_.subscribe("/checkpoint", 1, &LocalGoalCreator::checkpoint_callback, this);
    node_edge_sub_ = nh_.subscribe("/node_edge_map", 1, &LocalGoalCreator::node_edge_callback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &LocalGoalCreator::current_pose_callback, this);
    is_stop_node_flag_sub_ = nh_.subscribe("/is_stop_node_flag", 1, &LocalGoalCreator::is_stop_node_flag_callback, this);

    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
    current_checkpoint_id_pub_ = nh_.advertise<std_msgs::Int32>("/current_checkpoint", 1);

    checkpoint_received_ = false;
    node_edge_map_received_ = false;
    current_pose_updated_ = false;
    current_checkpoint_id_ = start_node_;
    local_goal_index_ = 0;
    checkpoint_update_threshold_ = local_goal_dist_;
    update_angle_threshold_ = M_PI / 2.0;
    is_stop_node_ = false;

    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}

void LocalGoalCreator::checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    // ROS_INFO("checkpoint_callback");
    if (checkpoint_received_ == false)
    {
        checkpoint_ = *msg;
        if (checkpoint_.data.size() > 2)
        {
            checkpoint_received_ = true;
            next_checkpoint_id_ = checkpoint_.data[0];
            // if (next_checkpoint_id_ == start_node_)
            //     next_checkpoint_id_ = checkpoint_.data[1];
            ROS_INFO("checkpoint size: %d", (int)checkpoint_.data.size());
            while (current_checkpoint_id_ == next_checkpoint_id_)
            {
                checkpoint_.data.erase(checkpoint_.data.begin());
                next_checkpoint_id_ = checkpoint_.data.size() > 0 ? checkpoint_.data[0] : -1;
                next2_checkpoint_id_ = checkpoint_.data.size() > 1 ? checkpoint_.data[1] : -1;
                ROS_INFO("current_checkpoint_id_ : %d", current_checkpoint_id_);
                ROS_INFO("next_checkpoint_id_ : %d", next_checkpoint_id_);
                ROS_INFO("checkpoint_.data.size() : %d", (int)checkpoint_.data.size());
            }
        }
    }
}

void LocalGoalCreator::node_edge_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr &msg)
{
    // ROS_INFO("node_edge_callback");
    if (node_edge_map_received_ == false)
    {
        node_edge_map_ = *msg;
        if (node_edge_map_.nodes.size() != 0 && node_edge_map_.edges.size() != 0)
        {
            node_edge_map_received_ = true;
            for (auto &node : node_edge_map_.nodes)
                node_id_list_.push_back(node.id);
        }
    }
}

void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // ROS_INFO("current_pose_callback");
    if (current_pose_updated_ == false)
    {
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        current_pose_updated_ = true;
    }
}

void LocalGoalCreator::is_stop_node_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
    is_stop_node_ = msg->data;
}

void LocalGoalCreator::get_node2node_poses(int node0_id, int node1_id, std::vector<geometry_msgs::PoseStamped> &node2node_poses)
{
    // ROS_INFO("------------------------------------");
    // ROS_INFO("get_node2node_poses");
    int node0_idx = find(node_id_list_.begin(), node_id_list_.end(), node0_id) - node_id_list_.begin();
    int node1_idx = find(node_id_list_.begin(), node_id_list_.end(), node1_id) - node_id_list_.begin();
    geometry_msgs::Point node0_pos = node_edge_map_.nodes[node0_idx].point;
    geometry_msgs::Point node1_pos = node_edge_map_.nodes[node1_idx].point;
    node2node_poses.clear();
    double direction = atan2(node1_pos.y - node0_pos.y, node1_pos.x - node0_pos.x);

    // ROS_INFO("node0_id: %d (%f, %f)", node0_id, node0_pos.x, node0_pos.y);
    // ROS_INFO("node1_id: %d (%f, %f)", node1_id, node1_pos.x, node1_pos.y);
    // ROS_INFO("direction: %f", direction);

    while (true)
    {
        geometry_msgs::PoseStamped node2node_pose;
        node2node_pose.header.frame_id = "map";
        node2node_pose.pose.position.x = node0_pos.x + local_goal_interval_ * cos(direction);
        node2node_pose.pose.position.y = node0_pos.y + local_goal_interval_ * sin(direction);
        node2node_pose.pose.position.z = 0;
        node2node_pose.pose.orientation = tf::createQuaternionMsgFromYaw(direction);
        node2node_poses.push_back(node2node_pose);

        ROS_INFO("node2node_pose: %f, %f", node2node_pose.pose.position.x, node2node_pose.pose.position.y);

        if (node2node_poses.size() > 1e6)
            ROS_ERROR("node2node_poses.size() > 1e6");

        if (sqrt(pow(node2node_pose.pose.position.x - node1_pos.x, 2) + pow(node2node_pose.pose.position.y - node1_pos.y, 2)) < local_goal_interval_)
            break;
        node0_pos = node2node_pose.pose.position;
    }
}

void LocalGoalCreator::calc_checkpoint_update_threshold(int current_id, int next_id, int next2_id, double &threshold)
{
    if (is_stop_node_ == true)
    {
        threshold = stop_radius_min_;
    }
    else
    {
        int current_idx = find(node_id_list_.begin(), node_id_list_.end(), current_id) - node_id_list_.begin();
        int next_idx = find(node_id_list_.begin(), node_id_list_.end(), next_id) - node_id_list_.begin();
        int next2_idx = find(node_id_list_.begin(), node_id_list_.end(), next2_id) - node_id_list_.begin();
        double current_x = node_edge_map_.nodes[current_idx].point.x;
        double current_y = node_edge_map_.nodes[current_idx].point.y;
        double next_x = node_edge_map_.nodes[next_idx].point.x;
        double next_y = node_edge_map_.nodes[next_idx].point.y;
        double next2_x = node_edge_map_.nodes[next2_idx].point.x;
        double next2_y = node_edge_map_.nodes[next2_idx].point.y;

        double current2next = sqrt(pow(current_x - next_x, 2) + pow(current_y - next_y, 2));
        double next2next2 = sqrt(pow(next_x - next2_x, 2) + pow(next_y - next2_y, 2));
        double product = (next_x - current_x) * (next2_x - next_x) + (next_y - current_y) * (next2_y - next_y);
        double cos_theta = product / (current2next * next2next2);

        threshold = cos_theta > 0.5 ? local_goal_dist_ : stop_radius_min_;
    }
}

void LocalGoalCreator::calc_checkpoint_update_angle_threshold(int current_id, int next_id, int next2_id, double &threshold)
{
    if (is_stop_node_ == true)
    {
        threshold = M_PI;
    }
    else
    {
        int current_idx = find(node_id_list_.begin(), node_id_list_.end(), current_id) - node_id_list_.begin();
        int next_idx = find(node_id_list_.begin(), node_id_list_.end(), next_id) - node_id_list_.begin();
        int next2_idx = find(node_id_list_.begin(), node_id_list_.end(), next2_id) - node_id_list_.begin();
        double current_x = node_edge_map_.nodes[current_idx].point.x;
        double current_y = node_edge_map_.nodes[current_idx].point.y;
        double next_x = node_edge_map_.nodes[next_idx].point.x;
        double next_y = node_edge_map_.nodes[next_idx].point.y;
        double next2_x = node_edge_map_.nodes[next2_idx].point.x;
        double next2_y = node_edge_map_.nodes[next2_idx].point.y;

        double current2next = sqrt(pow(current_x - next_x, 2) + pow(current_y - next_y, 2));
        double next2next2 = sqrt(pow(next_x - next2_x, 2) + pow(next_y - next2_y, 2));
        double product = (next_x - current_x) * (next2_x - next_x) + (next_y - current_y) * (next2_y - next_y);
        double cos_theta = product / (current2next * next2next2);
        cos_theta = (1.0 - cos_theta) / 2.0; // 0.0(0 deg) ~ 0.5(90 deg) ~ 1.0(180 deg)

        threshold = (1.0 - cos_theta) * M_PI / 2.0;
    }
}

bool LocalGoalCreator::reached_checkpoint(int current_checkpoint_id, int next_checkpoint_id, geometry_msgs::PoseStamped current_pose)
{
    // ROS_INFO("------------------------------------");
    // ROS_INFO("reached_checkpoint");
    int next_checkpoint_idx = find(node_id_list_.begin(), node_id_list_.end(), next_checkpoint_id) - node_id_list_.begin();
    double next_checkpoint_x = node_edge_map_.nodes[next_checkpoint_idx].point.x;
    double next_checkpoint_y = node_edge_map_.nodes[next_checkpoint_idx].point.y;

    int current_checkpoint_idx = find(node_id_list_.begin(), node_id_list_.end(), current_checkpoint_id) - node_id_list_.begin();
    double current_checkpoint_x = node_edge_map_.nodes[current_checkpoint_idx].point.x;
    double current_checkpoint_y = node_edge_map_.nodes[current_checkpoint_idx].point.y;

    double current_pose_x = current_pose.pose.position.x;
    double current_pose_y = current_pose.pose.position.y;

    double angle_diff = atan2(current_pose_y - next_checkpoint_y, current_pose_x - next_checkpoint_x) - atan2(current_checkpoint_y - next_checkpoint_y, current_checkpoint_x - next_checkpoint_x);
    angle_diff = fabs(angle_diff);
    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;

    bool is_inside_dist = sqrt(pow(current_pose.pose.position.x - next_checkpoint_x, 2) + pow(current_pose.pose.position.y - next_checkpoint_y, 2)) < checkpoint_update_threshold_;
    bool is_outside_angle = angle_diff > update_angle_threshold_;
    return is_inside_dist || is_outside_angle;
}

geometry_msgs::PoseStamped LocalGoalCreator::get_local_goal(std::vector<geometry_msgs::PoseStamped> &node2node_poses, int &poses_index, geometry_msgs::PoseStamped current_pose)
{
    // ROS_INFO("get_local_goal");
    double current_local_goal_x = node2node_poses[poses_index].pose.position.x;
    double current_local_goal_y = node2node_poses[poses_index].pose.position.y;

    int selected_index = poses_index;
    for (int i = poses_index; i < node2node_poses.size(); i++)
    {
        double goal_pose_x = node2node_poses[i].pose.position.x;
        double goal_pose_y = node2node_poses[i].pose.position.y;
        if (sqrt(pow(current_pose.pose.position.x - goal_pose_x, 2) + pow(current_pose.pose.position.y - goal_pose_y, 2)) < local_goal_dist_)
            selected_index = i;
    }
    poses_index = selected_index;

    geometry_msgs::PoseStamped local_goal;
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = ros::Time::now();
    local_goal.pose.position.x = node2node_poses[poses_index].pose.position.x;
    local_goal.pose.position.y = node2node_poses[poses_index].pose.position.y;
    local_goal.pose.position.z = 0;
    local_goal.pose.orientation = node2node_poses[poses_index].pose.orientation;

    try
    {
        tf_buffer_.transform(local_goal, local_goal_base_link_, local_goal_frame_id_, ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    return local_goal;
}

bool LocalGoalCreator::reached_goal(int goal_node_id, geometry_msgs::PoseStamped current_pose)
{
    // ROS_INFO("------------------------------------");
    // ROS_INFO("reached_goal");
    int goal_idx = find(node_id_list_.begin(), node_id_list_.end(), goal_node_id) - node_id_list_.begin();
    double goal_x = node_edge_map_.nodes[goal_idx].point.x;
    double goal_y = node_edge_map_.nodes[goal_idx].point.y;
    if (sqrt(pow(current_pose.pose.position.x - goal_x, 2) + pow(current_pose.pose.position.y - goal_y, 2)) < local_goal_dist_)
        return true;
    else
        return false;
}

void LocalGoalCreator::process()
{
    ros::Rate rate(hz_);
    while (ros::ok())
    {
        if (checkpoint_received_ && node_edge_map_received_ && current_pose_updated_)
        {
            // ROS_INFO("========================================");
            // ROS_INFO("current_checkpoint_id: %d", current_checkpoint_id_);
            // ROS_INFO("next_checkpoint_id: %d", next_checkpoint_id_);
            // ROS_INFO("current_pose: (%f, %f)", current_pose_.pose.position.x, current_pose_.pose.position.y);
            // ROS_INFO("local_goal: (%f, %f)", local_goal_.pose.position.x, local_goal_.pose.position.y);
            // ROS_INFO("local_goal_index: %d", local_goal_index_);
            // ROS_INFO("local_goal_poses.size(): %d", (int)local_goal_poses_.size());

            if (reached_goal(goal_node_, current_pose_) && checkpoint_.data.size() == 0)
            {
                ROS_INFO("reached_goal");
                break;
            }

            if (local_goal_poses_.size() == 0)
            {
                get_node2node_poses(current_checkpoint_id_, next_checkpoint_id_, local_goal_poses_);
                calc_checkpoint_update_threshold(current_checkpoint_id_, next_checkpoint_id_, next2_checkpoint_id_, checkpoint_update_threshold_);
                calc_checkpoint_update_angle_threshold(current_checkpoint_id_, next_checkpoint_id_, next2_checkpoint_id_, update_angle_threshold_);
            }

            if (reached_checkpoint(current_checkpoint_id_, next_checkpoint_id_, current_pose_))
            {
                ROS_WARN("reached_checkpoint");
                // if (checkpoint_.data.size() == 0)
                // {
                //     ROS_WARN("Checkpoint is empty");
                // }
                local_goal_index_ = 0;
                current_checkpoint_id_ = next_checkpoint_id_;
                while (current_checkpoint_id_ == next_checkpoint_id_)
                {
                    checkpoint_.data.erase(checkpoint_.data.begin());
                    next_checkpoint_id_ = checkpoint_.data.size() > 0 ? checkpoint_.data[0] : goal_node_;
                    next2_checkpoint_id_ = checkpoint_.data.size() > 1 ? checkpoint_.data[1] : goal_node_;
                }
                get_node2node_poses(current_checkpoint_id_, next_checkpoint_id_, local_goal_poses_);
                local_goal_ = get_local_goal(local_goal_poses_, local_goal_index_, current_pose_);
                calc_checkpoint_update_threshold(current_checkpoint_id_, next_checkpoint_id_, next2_checkpoint_id_, checkpoint_update_threshold_);
                calc_checkpoint_update_angle_threshold(current_checkpoint_id_, next_checkpoint_id_, next2_checkpoint_id_, update_angle_threshold_);

                ROS_WARN("checkpoint updated");
                ROS_WARN("current_checkpoint_id: %d", current_checkpoint_id_);
                ROS_WARN("next_checkpoint_id: %d", next_checkpoint_id_);
            }
            else
            {
                local_goal_ = get_local_goal(local_goal_poses_, local_goal_index_, current_pose_);
            }
            // local_goal_pub_.publish(local_goal_); // frame_id: map
            local_goal_pub_.publish(local_goal_base_link_);
            current_pose_updated_ = false;

            std_msgs::Int32 current_checkpoint_id_msg;
            current_checkpoint_id_msg.data = current_checkpoint_id_;
            current_checkpoint_id_pub_.publish(current_checkpoint_id_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
