//// ***************************
//// Camera modes:
//// 1 -- Clarify the position of the object
//// 2 -- Capture the object
//// ***************************

#define COLOR_NORMAL    "\033[0m"
#define COLOR_RED       "\033[31m"
#define COLOR_GREEN     "\033[32m"
#define COLOR_YELLOW    "\033[33m"

#define RESEARCH_TABLE_DEBUG false

#include <local_task_planner/LTP.h>

// Other
#include <brics_actuator/JointPositions.h>
#include <std_srvs/Empty.h>

int findObjectIndexByID(const std::vector<red_msgs::ManipulationObject> & objects, int id)
{
    for (size_t i = 0; i < objects.size(); ++i) {
        ROS_WARN_STREAM("ID: (" << objects[i].obj << " | " << id << ")\t Dest: " << objects[i].dest);

        if (objects[i].obj == id && objects[i].dest == 2) {
            ROS_WARN_STREAM("Selected object: " << id);
            return i;
        }
    }
    return -1;
}

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
    localTaskServer(n, "LTP", boost::bind(&localTP::localTaskCallback, this, _1), false),
    destNaviClient("navi", true)
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
    localTaskServer.start();

    // Setup start recognize pose
    startPose.x = 0.24; startPose.y = 0; startPose.z = 0.05;
    startPose.theta = 3.1415; startPose.psi = 0;
    initialResearchAngle = 2.9496064359 - atan2(startPose.y, startPose.x);
    ROS_WARN_STREAM("Initial Research angle: " << initialResearchAngle);

    // Setup containers
    objectsContainer.poses[0].x = -0.33;
    objectsContainer.poses[0].y = 0;
    objectsContainer.poses[0].z = -0.02;
    objectsContainer.poses[0].theta = -3.1415;
    objectsContainer.poses[0].psi = 0;

    objectsContainer.poses[1].x = -0.33;
    objectsContainer.poses[1].y = 0.1;
    objectsContainer.poses[1].z = -0.02;
    objectsContainer.poses[1].theta = -3.1415;
    objectsContainer.poses[1].psi = -0.25;

    objectsContainer.poses[2].x = -0.33;
    objectsContainer.poses[2].y = -0.1;
    objectsContainer.poses[2].z = -0.02;
    objectsContainer.poses[2].theta = -3.1415;
    objectsContainer.poses[2].psi = 0.25;

    // Camera Research configuration
    cameraResearchAngleStep = M_PI/6;

    red_msgs::ArmPoses manipPoses;
    ROS_INFO("[LTP] Go to first position.");
    manipPoses.request.poses.push_back(startPose);
    if (manipulationPointClient.call(manipPoses)) {
        std::cout << "\t Successfull." << std::endl;
    } else {
        ROS_ERROR("ManipulatorPointClient is not active.");
    }
    ros::Duration(1).sleep();

    // Activate tf for search transform
    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion cameraQuat;
    try{
        transformStamped = tfBuffer.lookupTransform("arm_link_1", "realsense_camera", ros::Time(0), ros::Duration(3.0));
        // ROS_INFO_STREAM("Transform: " << transformStamped);
        tf2::fromMsg(transformStamped.transform.rotation, cameraQuat);
        tf2::fromMsg(transformStamped.transform.translation, cameraTranslation);
        cameraRotMatrix = tf2::Matrix3x3(cameraQuat);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

localTP::~localTP()
{}

void localTP::localTaskCallback(const red_msgs::LTPTaskGoalConstPtr & goal)
{

    std::vector<red_msgs::ManipulationObject> objects;
    red_msgs::LTPTaskFeedback feedback;
    red_msgs::LTPTaskResult result;

    objects = goal->objects;
    if (objects.empty())
        return;
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

    red_msgs::Pose firstPose, secondPose;
    red_msgs::ArmPoses manipPoses;
    std_srvs::Empty empty;
    int objectIndexOfMessage = 0;
    int graspedObjects = 0;

    // Set initial position of manipulator for object recognition
    firstPose.theta = 3.1415;
    secondPose.theta = 3.1415;

    std::vector<red_msgs::Pose> recognizedPoses;
    std::vector<long int> objIdenifiers;

    // Communicate with camera
    ROS_INFO("[LTP] Set request to camera with mode 1.");
    researchTableByCamera(recognizedPoses, objIdenifiers);
    if (cameraError == 1) {
        cameraError = 0;
    }
    ROS_INFO_STREAM("Finded OBJECTS num: " << recognizedPoses.size());

    // Activate tf for search transform
    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);

    JointValues optq; red_msgs::Pose objectTransform;
    double camTransx = cameraTranslation.m_floats[0];
    double camTransy = cameraTranslation.m_floats[1];
    double zx = cameraRotMatrix[0][2]; double zy = cameraRotMatrix[1][2];
    double a1 = zx*camTransx + zy*camTransy, a2 = 0, h = 0, xtrans = 0, ytrans = 0;
    int actualObjectID = 0;

    // Fill objects
    for (int i = 0; i < objects.size(); ++i) {
        objects[i].dest = 2;
    }

    if (RESEARCH_TABLE_DEBUG)
        return true;

    red_msgs::CameraTask cameraTask;
    for (int j = 0; j < recognizedPoses.size(); ++j) {

        /// Dest modes:
        /// 0 - object is not pick | desired objects more than object containers
        /// 1 - object is pick
        /// 2 - object not find
        /// 3 - object is not pick | fisicly cant find

        actualObjectID = j;
        objectIndexOfMessage = findObjectIndexByID(objects, objIdenifiers[actualObjectID]);
        if (objectIndexOfMessage == -1) {
            ROS_WARN("Object ids not equal");
            continue;
        }
        objects[objectIndexOfMessage].dest = 0;

        // Find manipulator configuration for optical axis of camera
        objectTransform = recognizedPoses[actualObjectID];

        // Find desired leinght of z axis
        a2 = camTransx*camTransx + camTransy*camTransy - (objectTransform.x*objectTransform.x + objectTransform.y*objectTransform.y);
        h = (-a1 + sqrt(a1*a1 - 4*a2))/2;                                   // Coefficients
        ROS_INFO_STREAM("h: " << h);
        ROS_INFO_STREAM("a2: " << a2);

        ytrans = camTransy + h*zy;
        xtrans = camTransx + h*zx;
        optq(0) = atan2(objectTransform.y, objectTransform.x)               // Desired vector
            - atan2(ytrans, xtrans);                                        // Object translation
        makeYoubotArmOffsets(optq);
        optq += initialResearchAngle;

        ROS_INFO("Angle q1: %f", optq(0));
        std::vector<int> jnum = {0};
        moveJoints(optq, jnum);
        ros::Duration(2).sleep();

        // Communicate with camera
        ROS_INFO_STREAM("[LTP] Set request to camera with mode 2.\t | id( " << objIdenifiers[actualObjectID] <<  ")");
        cameraTask.request.mode = 2;
        cameraTask.request.request_id = cameraTask.response.ids[actualObjectID];
        callCamera(cameraTask);
        if (cameraError == 1) {
            cameraError = 0;
            continue;
        }

        // firstPose.x = objectTransform.x;
        // firstPose.y = objectTransform.y;
        // firstPose.z = objectTransform.z + 0.1;
        // firstPose.psi = -objectTransform.psi;
        firstPose.x = cameraTask.response.poses[0].x;
        firstPose.y = cameraTask.response.poses[0].y;
        firstPose.z = cameraTask.response.poses[0].z + 0.1;
        firstPose.theta = 3.1415;
        firstPose.psi = -cameraTask.response.poses[0].psi;

        secondPose = firstPose;
        secondPose.z -= 0.09;

        // Pick object from table
        ROS_WARN_STREAM("[LTP] PICK THE Objects!!!!");
        manipPoses.request.poses.push_back(firstPose);
        manipPoses.request.poses.push_back(secondPose);
        std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[0] << std::endl;
                std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[1] << std::endl;

        if (manipulationLineTrjClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
        } else {
            ROS_ERROR("ManipulationLineTrjClient is not active.");
            continue;
        }
        manipPoses.request.poses.clear();
        ros::service::call("grasp", empty);
        manipPoses.request.poses.push_back(secondPose);
        manipPoses.request.poses.push_back(firstPose);
        if (manipulationLineTrjClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
        } else {
            ROS_ERROR("ManipulationLineTrjClient is not active.");
        }
        manipPoses.request.poses.clear();

        // If conteiner isn't full, get position of free container and save its id
        secondPose = objectsContainer.getFreeContainerPoseAndSetID(objIdenifiers[actualObjectID]);
        firstPose = secondPose;
        firstPose.z += 0.1;

        // TODO move to initial position by the segmental

        ROS_WARN("[LTP] PLACE TO TABLE Objects!!!!");
        manipPoses.request.poses.push_back(firstPose);
        manipPoses.request.poses.push_back(secondPose);
        manipPoses.request.vel = 1;
        manipPoses.request.accel = 0.5;
        std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[0] << std::endl;
                std::cout << "INFO: ++++++++++++++++++++++++++\n"
            << manipPoses.request.poses[1] << std::endl;

        if (manipulationLineTrjClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
            ++graspedObjects;
            objects[objectIndexOfMessage].dest = 1;
        } else {
            ROS_ERROR("ManipulationLineTrjClient is not active.");
            objects[objectIndexOfMessage].dest = 3;
            continue;
        }
        manipPoses.request.poses.clear();
        manipPoses.request.vel = 0;
        manipPoses.request.accel = 0;
        ros::service::call("release_arm", empty);

        manipPoses.request.poses.push_back(startPose);
        if (manipulationPointClient.call(manipPoses)) {
            std::cout << "\t Successfull." << std::endl;
        } else {
            ROS_ERROR("ManipulatorPointClient is not active.");
        }
        manipPoses.request.poses.clear();

        // If some problems are comming
        ros::service::call("release_arm", empty);

        // Check container is full
        if (objectsContainer.isFull())
            return 1;
    }
    objectsContainer.printIDS();

    return 1;
}

bool localTP::researchTableByCamera(std::vector<red_msgs::Pose> & recognizedPoses, std::vector<long int> & objIdenifiers)
{
    int posesNum = 3;
    double epsilon = 0.05;
    int initialSize = recognizedPoses.size();
    std::vector<int> joint1 = {0};
    JointValues currAng;
    bool valid = true;
    Vector3d poseDiff, currPose;
    red_msgs::CameraTask cameraTask;
    cameraTask.request.mode = 1;

    currAng(0) = initialResearchAngle - cameraResearchAngleStep;
    for (int i = 0; i < posesNum; ++i) {
        // Move joint 1 to angle
        moveJoints(currAng, joint1);
        ros::Duration(2).sleep();

        // Send task to camera with mode 1
        callCamera(cameraTask);

        // Check camera task size
        if (cameraTask.response.poses.empty())
            continue;

        for (int j = 0; j < cameraTask.response.poses.size(); ++j) {
            currPose = Vector3d(cameraTask.response.poses[j].x, cameraTask.response.poses[j].y, cameraTask.response.poses[j].z);

            // TODO check similarity of recognized objects
            // Check unique of object
            for (int k = 0; k < objIdenifiers.size(); ++k) {
                poseDiff = currPose - Vector3d(recognizedPoses[k].x, recognizedPoses[k].y, recognizedPoses[k].z);
                ROS_INFO_STREAM("[LTP] --- > Object poseDiff.norm() " << poseDiff.norm());
                if (objIdenifiers[k] == cameraTask.response.ids[j] && poseDiff.norm() < epsilon) {
                    valid = false;
                    break;
                }
            }
            if (valid) {
                recognizedPoses.push_back(cameraTask.response.poses[j]);
                objIdenifiers.push_back(cameraTask.response.ids[j]);
            }
            valid = true;
        }


        currAng(0) += cameraResearchAngleStep;
    }
    if (recognizedPoses.size() == initialSize)
        return false;

    for (int i = 0; i < recognizedPoses.size(); ++i) {
        ROS_INFO_STREAM("Rcognize: (" << objIdenifiers[i] << ")");
        std::cout << recognizedPoses[i] << std::endl;
    }
    return true;
}

int localTP::executePLACE(std::vector<red_msgs::ManipulationObject> & objects) {

    return 1;
}

bool localTP::moveJoints(JointValues angle, std::vector<int> jointNum)
{
    // Create message
    brics_actuator::JointPositions jointPositions = createArmPositionMsg(angle, jointNum);
    // std::cout << jointPositions << std::endl;
    // Move to desired angle
    armPublisher.publish(jointPositions);
}

int localTP::moveBase(geometry_msgs::Pose2D & Pose) {

    /// States:
    /// 1 - all ok
    /// 2 - cant move

    if (destNaviClient.isServerConnected()){
        red_msgs::DestGoal goal;
        goal.task = "dest";
        goal.dist = Pose;
        destNaviClient.sendGoal(goal);
        while (ros::ok()){
            bool finished_before_timeout = destNaviClient.waitForResult(ros::Duration(20.0));
            // If action service finished, return:
            if (finished_before_timeout){
                red_mgsg::DestResult result = destNaviClient.getResult();
                if (result.has_got)
                    return 1;
                else
                    return 2;
            }
        }
    }
    return 2;
}

void localTP::callCamera(red_msgs::CameraTask & task) {
    if (compVisionClient.call(task)) {
        ROS_INFO("Successfull");
        std::cout << "===============================================" << std::endl;
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
        std::cout << "===============================================" << std::endl;
    } else {
        ROS_ERROR("CompVisionClient is not active.");
    }
    if (cameraError == 1 && task.request.mode == 2) {
        ROS_ERROR_STREAM("Object with id " << task.request.request_id << "is NOT FOUND");
        return;
    }

    for (int i = 0; i < task.response.poses.size(); ++i) {
        double psi = 0;
        // psi = task.response.poses[0].psi % (2*M_PI);
        psi = std::fmod(task.response.poses[i].psi, 2*M_PI);
        if (std::abs(psi) > M_PI) {
            task.response.poses[i].psi -= sign(psi)*2*M_PI;
            psi -= sign(psi)*2*M_PI;
            ROS_ERROR_STREAM("Angle > " << 2*M_PI << " | sign psi: " << sign(psi)*2*M_PI);
        }
        if (std::abs(psi) > 100*M_PI/180) {
            task.response.poses[i].psi += -sign(psi)*M_PI;
            ROS_ERROR_STREAM("psi > : " << 100*M_PI/180 << " \t result: " << task.response.poses[i].psi);
        }
    }

}