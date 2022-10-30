#include "local_goal_creator/stop_flag_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stop_flag_manager_node");
    StopFlagManager stop_flag_manager;
    stop_flag_manager.process();
    return 0;
}