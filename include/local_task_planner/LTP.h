#ifndef MANIPULATION_CONTROL_NODE
#define MANIPULATION_CONTROL_NODE

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <red_msgs/ManipulationObjects.h>
#include <red_msgs/DestAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <utility>
#include <string>


class ManipulationControlNode 
{
    public:
        ManipulationControlNode(ros::NodeHandle n);
        ~ManipulationControlNode();

        void start();

    private:
        bool pickAndPlaseFromTable(red_msgs::ManipulationObjects::Request & req, red_msgs::ManipulationObjects::Response & res);
        bool startCamera(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
        bool pickObjects(std::vector<std::string> & objects, std::vector<double> & heights);
        bool placeObjects(std::vector<std::string> & objects);
        size_t containerFilling(const std::vector<std::string> &);

        size_t numberOfContainers;
        double cameraOffsetX, cameraOffsetY, cameraOffsetZ;
        double distance;    // Distance from camera to table
        JointValues currentJointAngles;
        ArmKinematics solver;
        Rangefinder rf;
        double heightTrasholdKoeff;
        double openGripperWidth;

        ros::NodeHandle nh;

        ros::ServiceClient cameraTaskClient;

        ros::ServiceClient cameraStopClient;

        ros::ServiceClient graspObjectClient;

        ros::ServiceClient moveToPoseClient;

        ros::ServiceClient rangefinderClient;

        ros::ServiceServer manipulationTaskServer;

        ros::ServiceServer startCameraServer;

        std::pair<std::vector<std::string>, std::vector<Pose>> objectContainer;

        actionlib::SimpleActionClient<red_msgs::DestAction> naviAc;
};
#endif