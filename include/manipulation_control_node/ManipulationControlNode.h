#ifndef MANIPULATION_CONTROL_NODE
#define MANIPULATION_CONTROL_NODE

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <utility>

class ManipulationControlNode 
{
    public:
        ManipulationControlNode(ros::NodeHandle n);
        ~ManipulationControlNode();

        void start();

    private:
        bool pickAndPlaseFromTable(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
        bool startCamera(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
        bool switchCamera();
        size_t checkContainerContents(std::pair<std::vector<bool>, std::vector<Pose>> & container);

        double cameraOffsetX, cameraOffsetY, cameraOffsetZ;
        JointValues currentJointAngles;
        ArmKinematics solver;

        ros::NodeHandle nh;

        ros::ServiceClient getListObjectsClient;

        ros::ServiceClient graspObjectClient;

        ros::ServiceClient moveToPoseClient;

        ros::ServiceClient switchCameraClient;

        ros::ServiceServer tableServer;

        ros::ServiceServer startCameraServer;

        std::pair<std::vector<bool>, std::vector<Pose>> objectContainer;
};
#endif