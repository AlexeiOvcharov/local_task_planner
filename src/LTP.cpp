#include <local_task_planner/LTP.h>


localTP::localTP(ros::NodeHandle & n, Configuration conf)
{
    nh = n;
    if (conf.mode == 1) {
        ROS_INFO("Start RoboCup localTP");
        localTaskServer = nh.advertiseService(conf.gtpServiceName, &localTP::localTaskCallback, this);
        compVisionClient = nh.serviceClient<red_msgs::CameraTask>(conf.cvServiceName);
        manipulationPointClient = nh.serviceClient<red_msgs::ArmPoses>(conf.manipServiceName[0]);
        manipulationLineTrjClient = nh.serviceClient<red_msgs::ArmPoses>(conf.manipServiceName[1]);
    }
}

localTP::~localTP()
{}

bool localTP::localTaskCallback(std_srvs::Empty::Request  & req,
                            std_srvs::Empty::Response & res)
{
    ROS_INFO("[LTP] Start task execution");

    red_msgs::Pose pose;
    red_msgs::ArmPoses manipPoses;

    // Set initial position of manipulator for object recognition
    pose.x = 0.2; pose.y = 0; pose.z = 0.1;
    pose.theta = 3.1415; pose.psi = 0;
    ROS_INFO("[LTP] Go to first position.");
    ROS_INFO("[LTP] Pose: [%f, %f, %f, %f, %f]", pose.x, pose.y, pose.z, pose.theta, pose.psi);
    manipPoses.request.poses.push_back(pose);
    if (manipulationPointClient.call(manipPoses)) {
        std::cout << "\t Successfull." << std::endl;
    }

    // Communicate with camera
    ROS_INFO("[LTP] Set request to camera.");
    red_msgs::CameraTask cameraTask;
    cameraTask.request.mode = 1;
    if (compVisionClient.call(cameraTask)) {
        ROS_INFO("Successfull");
        int err = cameraTask.response.error;
        std::cout << "Error: " << err << std::endl;
        for (size_t i = 0; i < cameraTask.response.poses.size(); ++i) {
            std::cout << "Pose: " << i << std::endl;
            std::cout << "\t x:     \t" << cameraTask.response.poses[i].x << std::endl;
            std::cout << "\t y:     \t" << cameraTask.response.poses[i].y << std::endl;
            std::cout << "\t z:     \t" << cameraTask.response.poses[i].z << std::endl;
            std::cout << "\t phi:   \t" << cameraTask.response.poses[i].phi << std::endl;
            std::cout << "\t theta: \t" << cameraTask.response.poses[i].theta << std::endl;
            std::cout << "\t psi:   \t" << cameraTask.response.poses[i].psi << std::endl;

            std::cout << "Id: " << i << ": \t" << cameraTask.response.ids[i]<< std::endl;
            std::cout << "------------------------------------------" << std::endl;
        }
    }

    return true;
}