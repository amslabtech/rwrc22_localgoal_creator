#include "localgoal_creator/localgoal_creator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localgoal_creator_node");
    LocalGoalCreator localgoal_creator;
    localgoal_creator.process();
    return 0;
}
