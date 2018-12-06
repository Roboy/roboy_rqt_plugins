#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "roboy_control_msgs/PerformMovementsAction.h"

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_client");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementsAction> ac("movements_server", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started.");

//    roboy_control_msgs::PerformMovemenstGoal goal;
//    goal.action = "tosides";
//    ac.sendGoal(goal);
//    ac.waitForResult();

    return 0;
}