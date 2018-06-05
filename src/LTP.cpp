#include <local_task_planner/LTP.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>


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

    red_msgs::Pose recognPose;
    red_msgs::ArmPoses manipPoses;

    // Set initial position of manipulator for object recognition
    recognPose.x = 0.2; recognPose.y = 0; recognPose.z = 0.1;
    recognPose.theta = 3.1415; recognPose.psi = 0;
    ROS_INFO("[LTP] Go to first position.");
    ROS_INFO("[LTP] recognPose: [%f, %f, %f, %f, %f]", recognPose.x, recognPose.y, recognPose.z, recognPose.theta, recognPose.psi);
    manipPoses.request.poses.push_back(recognPose);
    if (manipulationPointClient.call(manipPoses)) {
        std::cout << "\t Successfull." << std::endl;
    }
    manipPoses.request.poses.clear();

    // Communicate with camera
    ROS_INFO("[LTP] Set request to camera.");
    red_msgs::CameraTask cameraTask;
    cameraTask.request.mode = 1;
    if (compVisionClient.call(cameraTask)) {
        ROS_INFO("Successfull");
        int err = cameraTask.response.error;
        std::cout << "Error: " << err << std::endl;
        for (size_t i = 0; i < cameraTask.response.poses.size(); ++i) {
            // Transform between arm_link_2 and arm_link_5
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

    // Activate tf for search transform
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Find manipulator configuration for optical axis of camera
    JointValues optq; red_msgs::Pose objectTransform;
    objectTransform.x = cameraTask.response.poses[0].x;
    objectTransform.y = cameraTask.response.poses[0].y;
    objectTransform.z = cameraTask.response.poses[0].z;
    optq(0) = atan2(recognPose.y, recognPose.x);

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("realsense_camera", "arm_link_2", ros::Time(1));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    optq(0) += atan2(objectTransform.y, objectTransform.x)
        - atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);

    ROS_INFO("Angle q1: %f", optq(0));


    return true;
}