#include <uav_ros_control/reference/WaitAndObserveStateMachine.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vs_pursuit_state_machine");
    ros::NodeHandle nh;

    std::shared_ptr<uav_reference::PursuitStateMachine> vssmObj{new uav_reference::PursuitStateMachine(nh)};
    vssmObj->run();
}