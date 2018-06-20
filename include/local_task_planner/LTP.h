#ifndef LTP
#define LTP

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>

#include <red_msgs/Pose.h>
#include <red_msgs/CameraTask.h>
#include <red_msgs/ArmPoses.h>
#include <red_msgs/LTPTaskAction.h>
#include <red_msgs/DestAction.h>
#include <red_msgs/ManipulationObject.h>

// TF
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_srvs/Empty.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

struct Container
{
    Container()
    {}

    Container(size_t contSize) {
        size = contSize;
        ids.resize(size);
        poses.resize(size);
        std::fill(ids.begin(), ids.end(), -1);
    }

    bool isEmpty(int num) {
        if (ids[num] == -1)
            return true;
        return false;
    }

    red_msgs::Pose getFreeContainerPoseAndSetID(int id) {
        for (size_t i = 0; i < size; ++i) {
            if(isEmpty(i)) {
                ids[i] = id;
                return poses[i];
            }
        }
    }

    int getContainerByID(int id) {
        for (int i = 0; i < size; ++i) {
            if (ids[i] == id)
                return i;
        }
        return -1;
    }

    bool clearContainer(int id) {
        int num = getContainerByID(id);
        if (num == -1)
            return false;
        ids[num] = -1;
        return true;
    }

    void printIDS()
    {
        std::cout << "------------------ IDS ------------------" << std::endl;
        for (int i = 0; i < size; ++i) {
            std::cout << "id (" << i << ") ------------------------ " << ids[i] << std::endl;
        }
        std::cout << "/----------------- IDS -----------------/" << std::endl;
    }

    bool isFull()
    {
        for (int i = 0; i < size; ++i) {
            if (isEmpty(i))
                return false;
        }
        return true;
    }

    bool empty()
    {
        for (int i = 0; i < size; ++i) {
            if (!isEmpty(i))
                return false;
        }
        return true;
    }

    size_t size;

    // Object identificators
    std::vector<int> ids;

    // Poses in contaier
    std::vector<red_msgs::Pose> poses;
};


struct Configuration
{
    // Modes::
    // 1 |  Work with RoboCup RefBox

    int mode;
    // Computer vision service name
    std::string cvServiceName;
    // Manipulation service name
    std::vector<std::string> manipServiceName;
    // Navigation service name
    std::string navServiceName;
    // Global Task Planner service name
    std::string gtpServiceName;
};

class localTP
{
    public:
        localTP(ros::NodeHandle & n, Configuration conf);
        ~localTP();

    private:

        void localTaskCallback(const red_msgs::LTPTaskGoalConstPtr & goal);

        void callCamera(red_msgs::CameraTask & task);

        void recognizeAndPickObjects(std::vector<int> objectsForGrasping);

        int executePICK(std::vector<red_msgs::ManipulationObject> & objects);
        int executePLACE(std::vector<red_msgs::ManipulationObject> & objects);

        // Function for moving base to desired vector by action client
        int moveBase(geometry_msgs::Pose2D & Pose);

        // Go to initial position for 2...4 joints and switch off motors
        void goToInitialAndRelax();

        // Camera loock to table from 3 selected arm positions.
        // Function return all recognize object positions and their identificators
        bool researchTableByCamera(std::vector<red_msgs::Pose> & regonizedPoses, std::vector<long int> & objIdenifiers);

        // Move first joint of manipulator
        bool moveJoints(JointValues angle, std::vector<int> jointNum);

        bool getPadPlace(red_msgs::ArmPoses & p, red_msgs::ManipulationObject & object, std::vector<red_msgs::Pose> & tablePoses, std::vector<long int> & objIdenifiers);

        ros::NodeHandle nh;

        // Container for objects (table behind youbot)
        Container objectsContainer;

        // Kinematics class for transform camera point
        ArmKinematics kinematic;

        // Offset of camera from link 5
        red_msgs::Pose cameraOffset;

        red_msgs::Pose startPose;

        // Transformation of camera at cameraRecognizePosition
        tf2::Vector3 cameraTranslation;
        tf2::Matrix3x3 cameraRotMatrix;

        // Client for communication with
        // Computor Vision step
        ros::ServiceClient compVisionClient;

        // Client for communication with
        // Manipulation step
        // Move to point
        ros::ServiceClient manipulationPointClient;
        // Move to line
        ros::ServiceClient manipulationLineTrjClient;

        // Gripper service
        ros::ServiceClient gripperClient;

        // Publishers
        ros::Publisher armPublisher;

        // Errors
        int cameraError;

        // Actions
        actionlib::SimpleActionServer<red_msgs::LTPTaskAction> localTaskServer;
        actionlib::SimpleActionClient<red_msgs::DestAction> destNaviClient;

        // Initial Angle of joint 1 for researching tabole
        double initialResearchAngle;

        // Step of angle of joint 1 for researching table
        double cameraResearchAngleStep;

        // Base positions
        std::vector<geometry_msgs::Pose2D> positionsOfBase;
        std::vector<red_msgs::Pose> placingTablePoses;

        // Camera recognized joint values
        std::vector<JointValues> camJV;
};

#endif