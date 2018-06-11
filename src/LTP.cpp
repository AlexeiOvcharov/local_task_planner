//// ***************************
//// Camera modes:
//// 1 -- Clarify the position of the object
//// 2 -- Capture the object
//// ***************************

#define COLOR_NORMAL    "\033[0m"
#define COLOR_RED       "\033[31m"
#define COLOR_GREEN     "\033[32m"
#define COLOR_YELLOW    "\033[33m"


#include <local_task_planner/LTP.h>

// TF
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

// Other
#include <brics_actuator/JointPositions.h>
#include <std_srvs/Empty.h>


// TODO move to utils
brics_actuator::JointPositions createArmPositionMsg(const JointValues & jointAngles, std::vector<int> number)
{
    //ROS_INFO("createArmPositionMsg...");
    brics_actuator::JointPositions jointPositions;
    brics_actuator::JointValue jointValue;
    for (size_t i = 0; i < number.size(); ++i) {
        jointValue.timeStamp = ros::Time::now();
        std::stringstream jointName;
        jointName << "arm_joint_" << (number[i] + 1);
        jointValue.joint_uri = jointName.str();
        jointValue.unit = "rad";
        jointValue.value = jointAngles(number[i]);
        jointPositions.positions.push_back(jointValue);
    }
    return jointPositions;
}

localTP::localTP(ros::NodeHandle & n, Configuration conf) :
    objectsContainer(3),
    localTaskServer(n, "LTP", boost::bind(&localTP::localTaskCallback, this, _1), false)
{
    nh = n;
    if (conf.mode == 1) {
        ROS_INFO("Start RoboCup localTP");

        compVisionClient = nh.serviceClient<red_msgs::CameraTask>(conf.cvServiceName);
        if (!ros::service::waitForService(conf.cvServiceName, ros::Duration(3.0))) {
            ROS_ERROR("Server %s is not active!", conf.cvServiceName.c_str());
            ros::shutdown();
        }

        manipulationPointClient = nh.serviceClient<red_msgs::ArmPoses>(conf.manipServiceName[0]);
        if (!ros::service::waitForService(conf.cvServiceName, ros::Duration(3.0))) {
            ROS_ERROR("Server %s is not active!", conf.manipServiceName[0].c_str());
            ros::shutdown();
        }

        manipulationLineTrjClient = nh.serviceClient<red_msgs::ArmPoses>(conf.manipServiceName[1]);
        if (!ros::service::waitForService(conf.cvServiceName, ros::Duration(3.0))) {
            ROS_ERROR("Server %s is not active!", conf.manipServiceName[1].c_str());
            ros::shutdown();
        }
    }
    armPublisher = nh.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);

    // Setup containers
    objectsContainer.poses[0].x = -0.37;
    objectsContainer.poses[0].y = 0;
    objectsContainer.poses[0].z = -0.01;
    objectsContainer.poses[0].theta = -3.1415;
    objectsContainer.poses[0].psi = 0;

    objectsContainer.poses[1].x = -0.37;
    objectsContainer.poses[1].y = 0.1;
    objectsContainer.poses[1].z = -0.01;
    objectsContainer.poses[1].theta = -3.1415;
    objectsContainer.poses[1].psi = -0.25;

    objectsContainer.poses[2].x = -0.37;
    objectsContainer.poses[2].y = -0.1;
    objectsContainer.poses[2].z = -0.01;
    objectsContainer.poses[2].theta = -3.1415;
    objectsContainer.poses[2].psi = 0.25;
}

localTP::~localTP()
{}

void localTP::localTaskCallback(const red_msgs::LTPTaskGoalConstPtr & goal)
{

    std::vector<red_msgs::ManipulationObject> objects;
    red_msgs::LTPTaskFeedback feedback;
    red_msgs::LTPTaskResult result;

    objects = goal->objects;
    if (goal->task == 1) {           /// PICK
        executePICK(objects);
    } else if (goal->task == 2) {    /// PLACE
        executePLACE(objects);
    }

    result.result = objects;
    localTaskServer.setSucceeded(result);
}

int localTP::executePICK(std::vector<red_msgs::ManipulationObject> & objects) {
    ROS_INFO("[LTP] Start PICK execution");

    red_msgs::Pose recognPose, firstPose, secondPose;
    red_msgs::ArmPoses manipPoses;
    std_srvs::Empty empty;

    // Set initial position of manipulator for object recognition
    recognPose.x = 0.24; recognPose.y = 0; recognPose.z = 0.05;
    recognPose.theta = 3.1415; recognPose.psi = 0;

    ROS_INFO("[LTP] Go to first position.");
    manipPoses.request.poses.push_back(recognPose);
    if (manipulationPointClient.call(manipPoses)) {
        std::cout << "\t Successfull." << std::endl;
    } else {
        ROS_ERROR("ManipulatorPointClient is not active.");
    }
    manipPoses.request.poses.clear();
    ros::Duration(1).sleep();

    // Communicate with camera
    ROS_INFO("[LTP] Set request to camera with mode 1.");
    red_msgs::CameraTask cameraTask;
    cameraTask.request.mode = 1;
    callCamera(cameraTask);
    if (cameraError == 1) {
        cameraError = 0;
    }

    std::vector<red_msgs::Pose> regonizedPoses = cameraTask.response.poses;
    std::vector<long int> objIdenifiers = cameraTask.response.ids;

    // Activate tf for search transform
    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);

    JointValues optq; red_msgs::Pose objectTransform;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion cameraQuat;
    tf2::Matrix3x3 rotMatrix;
    try{
        transformStamped = tfBuffer.lookupTransform("arm_link_1", "realsense_camera", ros::Time(0), ros::Duration(3.0));
        ROS_INFO_STREAM("Transform: " << transformStamped);
        tf2::fromMsg(transformStamped.transform.rotation, cameraQuat);
        rotMatrix = tf2::Matrix3x3(cameraQuat);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    double camTransx = transformStamped.transform.translation.x;
    double camTransy = transformStamped.transform.translation.y;
    double zx = rotMatrix[0][2]; double zy = rotMatrix[1][2];
    double a1 = zx*camTransx + zy*camTransy, a2 = 0, h = 0, xtrans = 0, ytrans = 0;
    int actualObjectID = 0;

    for (int j = 0; j < regonizedPoses.size(); ++j) {
        actualObjectID = j;

        // Find manipulator configuration for optical axis of camera
        objectTransform.x = regonizedPoses[actualObjectID].x;
        objectTransform.y = regonizedPoses[actualObjectID].y;
        objectTransform.z = regonizedPoses[actualObjectID].z;
        optq(0) = atan2(recognPose.y, recognPose.x);

        // Find desired leinght of z axis
        a2 = camTransx*camTransx + camTransy*camTransy - (objectTransform.x*objectTransform.x + objectTransform.y*objectTransform.y);
        h = (-a1 + sqrt(a1*a1 - 4*a2))/2;                            // Coefficients
        ROS_INFO_STREAM("h: " << h);
        ROS_INFO_STREAM("a2: " << a2);

        ytrans = camTransy + h*zy;
        xtrans = camTransx + h*zx;
        optq(0) += atan2(objectTransform.y, objectTransform.x)              // Desired vector
            - atan2(ytrans, xtrans);                                        // Object translation

        makeYoubotArmOffsets(optq);
        ROS_INFO("Angle q1: %f", optq(0));

        // Create message
        std::vector<int> jnum = {0};
        brics_actuator::JointPositions jointPositions = createArmPositionMsg(optq, jnum);
        std::cout << jointPositions << std::endl;
        armPublisher.publish(jointPositions);       // Move to desired angle
        ros::Duration(3).sleep();

        // Communicate with camera
        ROS_INFO("[LTP] Set request to camera with mode 2.");
        cameraTask.request.mode = 2;
        cameraTask.request.request_id = cameraTask.response.ids[actualObjectID];
        callCamera(cameraTask);
        if (cameraError == 1) {
            cameraError = 0;
            continue;
        }

        red_msgs::Pose desPose;
        firstPose.x = cameraTask.response.poses[0].x;
        firstPose.y = cameraTask.response.poses[0].y;
        firstPose.z = cameraTask.response.poses[0].z + 0.1;
        firstPose.psi = -cameraTask.response.poses[0].psi;

        desPose = firstPose;
        desPose.z -= 0.1;

        ROS_WARN("[LTP] PICK THE Objects!!!!");
        manipPoses.request.poses.push_back(firstPose);
        manipPoses.request.poses.push_back(desPose);
        std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[0] << std::endl;
                std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[1] << std::endl;

        if (manipulationLineTrjClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
        } else {
            ROS_ERROR("ManipulationLineTrjClient is not active.");
        }
        manipPoses.request.poses.clear();
        ros::service::call("grasp", empty);

        desPose = objectsContainer.getFreeContainerPoseAndSetID(actualObjectID);
        firstPose = desPose;
        firstPose.z += 0.1;

        ROS_WARN("[LTP] PLACE TO TABLE Objects!!!!");
        manipPoses.request.poses.push_back(firstPose);
        manipPoses.request.poses.push_back(desPose);
        std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[0] << std::endl;
                std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[1] << std::endl;

        if (manipulationLineTrjClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
        } else {
            ROS_ERROR("ManipulationLineTrjClient is not active.");
        }
        manipPoses.request.poses.clear();
        ros::service::call("release_arm", empty);

        ROS_INFO("[LTP] recognPose: [%f, %f, %f, %f, %f]", recognPose.x, recognPose.y, recognPose.z, recognPose.theta, recognPose.psi);
        manipPoses.request.poses.push_back(recognPose);
        if (manipulationPointClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
        } else {
            ROS_ERROR("ManipulatorPointClient is not active.");
        }
        manipPoses.request.poses.clear();
        ros::Duration(1).sleep();
    }
    objectsContainer.printIDS();

    return 1;
}

int localTP::executePLACE(std::vector<red_msgs::ManipulationObject> & objects) {

    return 1;
}

void localTP::callCamera(red_msgs::CameraTask & task) {
    if (compVisionClient.call(task)) {
        ROS_INFO("Successfull");
        cameraError = task.response.error;
        std::cout << "Error: " << cameraError << std::endl;
        for (size_t i = 0; i < task.response.poses.size(); ++i) {
            // Transform between arm_link_2 and arm_link_5 | Task 1 -- find all objects
            // Transform between arm_link_0 and arm_link_5 | Task 2 -- get current object by id
            std::cout << "Pose: " << i << std::endl;
            std::cout << task.response.poses[i] << std::endl;
            std::cout << "Id: " << i << ": \t" << task.response.ids[i]<< std::endl;
            std::cout << "------------------------------------------" << std::endl;
        }
    } else {
        ROS_ERROR("CompVisionClient is not active.");
    }
    if (cameraError == 1 && task.request.mode == 2) {
        ROS_ERROR_STREAM("Object with id " << task.request.request_id << "is NOT FOUND");
        return;
    }

    if (task.request.mode == 2) {
        if (abs(task.response.poses[0].psi) > 100*M_PI/180)
            task.response.poses[0].psi += -sign(task.response.poses[0].psi)*M_PI;

    }
}