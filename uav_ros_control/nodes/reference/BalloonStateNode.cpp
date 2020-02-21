#include "uav_ros_control/reference/BalloonStateMachine.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "balloon_state_machine_node");
    ros::NodeHandle nh;

    std::shared_ptr<uav_reference::BalloonStateMachine> bsmObj{new uav_reference::VisualServoStateMachine(nh)};
    bsmObj->run();
}