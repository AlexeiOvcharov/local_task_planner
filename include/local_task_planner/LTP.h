#ifndef LTP
#define LTP

#include <ros/ros.h>
#include <arm_kinematics/ArmKinematics.h>

#include <red_msgs/Pose.h>
#include <red_msgs/CameraTask.h>
#include <red_msgs/ArmPoses.h>

#include <std_srvs/Empty.h>

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
        for (int i = 0; i < ids.size(); ++i) {
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
        for (int i = 0; i < ids.size(); ++i) {
            std::cout << "id (" << i << ") ------------------------ " << ids[i] << std::endl;
        }
        std::cout << "/----------------- IDS -----------------/" << std::endl;
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

        bool localTaskCallback(std_srvs::Empty::Request  & req,
                               std_srvs::Empty::Response & res);

        void callCamera(red_msgs::CameraTask & task);

        void recognizeAndPickObjects(std::vector<int> objectsForGrasping);

        ros::NodeHandle nh;

        // Container for objects (table behind youbot)
        Container objectsContainer;

        // Offset of camera from link 5
        red_msgs::Pose cameraOffset;

        // Client for communication with
        // Computor Vision step
        ros::ServiceClient compVisionClient;

        // Client for communication with
        // Manipulation step
        // Move to point
        ros::ServiceClient manipulationPointClient;
        // Move to line
        ros::ServiceClient manipulationLineTrjClient;

        // Kinematics class for transform camera point
        ArmKinematics kinematic;

        // Server for communication with
        // Global task planner
        ros::ServiceServer localTaskServer;

        // Publishers
        ros::Publisher armPublisher;

        // Errors
        int cameraError;
};

#endif
