#include <manipulation_control_node/ManipulationControlNode.h>
#include <manipulation_control_node/GraspingArmPositions.h>

#include <std_srvs/Empty.h>
#include <arm_kinematics/PoseArray.h>
#include <arm_kinematics/ManipulatorPose.h>
#include <red_msgs/CameraTask.h>
#include <red_msgs/CameraStop.h>
#include <red_msgs/GetRange.h>


ManipulationControlNode::ManipulationControlNode(ros::NodeHandle n) : nh(n), naviAc("navi", true)
{
    ROS_INFO_STREAM("[Local TP] Load Local TP Node...");

    nh.getParam("/move_by_camera/camera_offset_x", cameraOffsetX);
    nh.getParam("/move_by_camera/camera_offset_y", cameraOffsetY);
    nh.getParam("/move_by_camera/camera_offset_z", cameraOffsetZ);
    ROS_INFO_STREAM("Camera offset: " << cameraOffsetX << ", " << cameraOffsetY <<  ", " << cameraOffsetZ);

    ROS_INFO_STREAM("[Local TP] ServiceClient: /camera_task...");
    cameraTaskClient = nh.serviceClient<red_msgs::CameraTask>("/camera_task");

    ROS_INFO_STREAM("[Local TP] ServiceClient: /camera_stop...");
    cameraStopClient = nh.serviceClient<red_msgs::CameraStop>("/camera_stop");

    ROS_INFO_STREAM("[Local TP] ServiceServer: /manipulation_task...");
    manipulationTaskServer = nh.advertiseService("/manipulation_task", &ManipulationControlNode::pickAndPlaseFromTable, this);

    ROS_INFO_STREAM("[Local TP] ServiceClient: /grasp_object...");
    graspObjectClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/grasp_object");

    ROS_INFO_STREAM("[Local TP] ServiceClient: /manipulator_pose...");
    moveToPoseClient = nh.serviceClient<arm_kinematics::ManipulatorPose>("/manipulator_pose");

    ROS_INFO_STREAM("[Local TP] ServiceClient: /get_range...");
    rangefinderClient = nh.serviceClient<red_msgs::GetRange>("/get_range");

    ROS_INFO_STREAM("[Local TP] ServiceServer: /recognize_floor_objects...");
    startCameraServer = nh.advertiseService("/recognize_floor_objects", &ManipulationControlNode::startCamera, this);

    // ROS_INFO_STREAM("[Local TP] Connect to rangefinder to port" << "/dev/ttyACM1");
    // rf.open("/dev/ttyACM1");

    /////////////////////// INITIAL POSE FOR RECOGNIZED
    initialPoseForRecognized.position(0) = 0.3;
    initialPoseForRecognized.position(1) = 0.0;
    initialPoseForRecognized.position(2) = 0.5;

    /////////////////////// FIRST CONTAINER POINT
    firstContainerPoint.position(0) = -0.3;
    firstContainerPoint.position(1) = 0;
    firstContainerPoint.position(2) = -0.005;
    firstContainerPoint.orientation(0) = 0;
    firstContainerPoint.orientation(1) = 0;
    firstContainerPoint.orientation(2) = -3.1415;

    /////////////////////// SECOND CONTAINER POINT
    secondContainerPoint.position(0) = -0.25;
    secondContainerPoint.position(1) = -0.1;
    secondContainerPoint.position(2) = -0.005;
    secondContainerPoint.orientation(0) = 0;
    secondContainerPoint.orientation(1) = 0;
    secondContainerPoint.orientation(2) = -3.1415;

    /////////////////////// THIRD CONTAINER POINT
    thirdContainerPoint.position(0) = -0.25;
    thirdContainerPoint.position(1) = 0.1;
    thirdContainerPoint.position(2) = -0.005;
    thirdContainerPoint.orientation(0) = 0;
    thirdContainerPoint.orientation(1) = 0;
    thirdContainerPoint.orientation(2) = -3.1415;

    numberOfContainers = 3;
    std::vector<std::string> objectsName = {"", "", ""};
    std::vector<Pose> containersPose = {firstContainerPoint, secondContainerPoint, thirdContainerPoint};
    objectContainer = std::make_pair(objectsName, containersPose);

    // finding angle for camera vision manipulator position
    solver.solveFullyIK(initialPoseForRecognized, currentJointAngles);

    heightTrasholdKoeff = 1/3;
    openGripperWidth = 0.077;
}

ManipulationControlNode::~ManipulationControlNode()
{}


bool ManipulationControlNode::startCamera(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
    red_msgs::CameraTask task;
    task.request.mode = 4;
    task.request.shape = "";

    ROS_INFO_STREAM("Turn ON camera with mode: 4");
    if (!cameraTaskClient.call(task)) {
        ROS_FATAL_STREAM("Camera is not start with mode 4!");
        return false;
    }
    ROS_INFO_STREAM("Camera STARTING work");
    return true;
}
size_t  ManipulationControlNode::containerFilling(const std::vector<std::string> & containerObjectsName) {
    size_t count = 0;
    for (size_t i = 0; i < numberOfContainers; ++i) {
        if (containerObjectsName[i] == "") ++count;
    }
    return count;
}

bool ManipulationControlNode::pickAndPlaseFromTable(red_msgs::ManipulationObjects::Request & req, red_msgs::ManipulationObjects::Response & res)
{
    std::vector<std::string> objects = req.objects;
    std::vector<double> heights = req.height;
    if (req.task == red_msgs::ManipulationObjects::Request::PICK) {
        if (pickObjects(objects, heights)) {
            res.remaining_objects = objects;
            res.success = true;
            return true;
        } else {
            res.success = false;
            return false;
        }
    }
    if (req.task == red_msgs::ManipulationObjects::Request::PLACE) {
        if (placeObjects(req.objects)) {
            res.remaining_objects = objects;
            res.success = true;
            return true;
        } else {
            res.success = false;
            return false;
        }
    }
}
bool ManipulationControlNode::pickObjects(std::vector<std::string> & objects, std::vector<double> & heights)
{
    JointValues jointAngles;
    // Messages
    red_msgs::CameraTask task;
    red_msgs::GetRange range;
    Pose recognizedObjectPose;
    arm_kinematics::CertesianPose armPose;
    armPose.position.resize(3);
    armPose.orientation.resize(3);
    // TODO delete kostil!!!
    int objectNumber = 0;
    int graspingObjectNumber = 0;
    bool objectCapture = false;
    size_t containerSize = containerFilling(objectContainer.first);
    if (containerSize == 0) {
        ROS_FATAL_STREAM("Containers is fill!");
        return false;
    }

    arm_kinematics::ManipulatorPose manipulatorPose;
    armPose.position[0] = initialPoseForRecognized.position(0);
    armPose.position[1] = initialPoseForRecognized.position(1);
    armPose.position[2] = initialPoseForRecognized.position(2);
    armPose.orientation[0] = initialPoseForRecognized.orientation(0);
    armPose.orientation[1] = initialPoseForRecognized.orientation(1);
    armPose.orientation[2] = initialPoseForRecognized.orientation(2);
    manipulatorPose.request.pose = armPose;

    ROS_INFO_STREAM("Search free container.");
    size_t containerNumber = 0;
    ROS_INFO_STREAM("Container number: " << containerNumber);
    
    while (containerSize != 0 && nh.ok()) {

        if (moveToPoseClient.call(manipulatorPose)) ROS_INFO_STREAM("Go to initial pose.");
        else {
            ROS_FATAL_STREAM("Cant go to initial position");
            return false;
        }
        ros::Duration(2).sleep();

        if (rangefinderClient.call(range)) ROS_INFO_STREAM("Measuring distance.");
        else {
            ROS_WARN_STREAM("Cant read data from rangefinder.");
        }
        // distance = rf.getRange();
        distance = range.response.distance;
        ROS_INFO_STREAM("Distance : " << distance);

        ROS_INFO_STREAM("Turn ON camera.");
        ROS_INFO_STREAM("Reading data from camera.");
        task.request.mode = 1;
        task.request.shape = "";
        task.request.distance = distance;
        do {
            cameraTaskClient.call(task);
        } while (task.response.list.empty() && nh.ok());

        // TODO delete kostil
        for (objectNumber = 0; objectNumber < task.response.list.size(); ++objectNumber) {
            if (objects[graspingObjectNumber] == task.response.list[objectNumber].shape) {
                ROS_INFO_STREAM("Request: " << objects[graspingObjectNumber] << "| Camera: " << task.response.list[objectNumber].shape);
                double objectHeight = heights[graspingObjectNumber];

                recognizedObjectPose.position(0) = task.response.list[objectNumber].coordinates_center_frame[0] + cameraOffsetX;
                recognizedObjectPose.position(1) = task.response.list[objectNumber].coordinates_center_frame[1] + cameraOffsetY;
                recognizedObjectPose.position(2) = task.response.list[objectNumber].coordinates_center_frame[2] + cameraOffsetZ + objectHeight*heightTrasholdKoeff;
                recognizedObjectPose.orientation(0) = 0;
                recognizedObjectPose.orientation(1) = task.response.list[objectNumber].orientation[1];
                recognizedObjectPose.orientation(2) = 3.1415;

                ROS_INFO_STREAM("[Control Node] Recognized object position: (" 
                    << recognizedObjectPose.position(0) << ", "
                    << recognizedObjectPose.position(1) << ", " 
                    << recognizedObjectPose.position(2) << ")\t"
                    << "angle: " << recognizedObjectPose.orientation(1));

                Vector3d objectoPoseFromBase = solver.transformFromFrame5ToFrame0(currentJointAngles, recognizedObjectPose.position);
                recognizedObjectPose.position = objectoPoseFromBase;

                // Check to object is desire to grasp
                solver.solveFullyIK(recognizedObjectPose, jointAngles);
                if (sin(jointAngles(4))*sin(jointAngles(1) + jointAngles(2) + jointAngles(3)) < 2*objectHeight*heightTrasholdKoeff/openGripperWidth) {    // h - object height
                    geometry_msgs::Pose2D desiredShiftOfBase;
                    red_msgs::DestGoal naviAcGoal;

                    desiredShiftOfBase.x = 0;
                    desiredShiftOfBase.y = sqrt(pow(recognizedObjectPose.position(0), 2) + pow(recognizedObjectPose.position(1), 2))*sin(jointAngles(4));
                    desiredShiftOfBase.theta = 0;
                    naviAcGoal.task = "dist";
                    naviAcGoal.dist = desiredShiftOfBase;
                    naviAc.sendGoal(naviAcGoal);
                    naviAc.waitForResult();
                    actionlib::SimpleClientGoalState state_ac = naviAc.getState();
                    if(state_ac.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        ROS_INFO("[GTP] Goal is successfully processed");
                        break;
                    }
                }

                // Grasp object
                for (size_t i = 0; i < 3; ++i) {
                    armPose.position[i] = recognizedObjectPose.position(i);
                    armPose.orientation[i] = recognizedObjectPose.orientation(i);
                }
                manipulatorPose.request.pose = armPose;
                ROS_INFO_STREAM("Grasp the object.");
                if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Successfull grasp the object");
                else {
                    ROS_FATAL_STREAM("Cant got to camera position.");
                    return false;
                }

                // Put object
                for (size_t i = 0; i < 3; ++i) {
                    armPose.position[i] = objectContainer.second[containerNumber].position(i);
                    armPose.orientation[i] = objectContainer.second[containerNumber].orientation(i);
                }
                manipulatorPose.request.pose = armPose;

                ROS_INFO_STREAM("Put the object.");
                if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Turn ON camera.");
                else {
                    ROS_FATAL_STREAM("Cant got to camera position.");
                    return false;
                }
                objectContainer.first[containerNumber] = objects[graspingObjectNumber];

                ROS_INFO_STREAM("Search free container.");
                ++containerNumber;
                objects.erase(objects.begin());
                ROS_INFO_STREAM("Container number: " << containerNumber);

                ++graspingObjectNumber;
                // objectCapture = true;
                break;
            } else {
                continue;
                // TODO compute here
            }
        }

        // if (objects[graspingObjectNumber] != task.response.list[objectNumber - 1].shape) {
        //     ROS_FATAL_STREAM("Objects not found");
        //     return false;
        // }
    }
    return true;
}

bool ManipulationControlNode::placeObjects(std::vector<std::string> & objects)
{
    arm_kinematics::ManipulatorPose manipulatorPose;
    arm_kinematics::CertesianPose armPose;

    size_t containerSize = containerFilling(objectContainer.first);

    double experimentalX = 0.32;
    double yStep = 0.1;
    double yMin = -0.1;
    double places[numberOfContainers][3];

    for (size_t i = 0; i < numberOfContainers; ++i) {
        places[i][0] = experimentalX;
        places[i][1] = yMin + i*yStep ;
        places[i][2] = 0.1;
    }

    while (containerSize != numberOfContainers && nh.ok()) {
        for (size_t container = 0; container < (numberOfContainers - containerSize); ++container) {
            if (objects[containerSize] == objects[container]) {

                // Grasp object from container
                for (size_t j = 0; j < 3; ++j) {
                    armPose.position[j] = objectContainer.second[container].position(j);
                    armPose.orientation[j] = objectContainer.second[container].orientation(j);
                }
                manipulatorPose.request.pose = armPose;
                ROS_INFO_STREAM("Grasp the object.");
                if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Successfull grasp the object");
                else {
                    ROS_FATAL_STREAM("Cant got to camera position.");
                    return false;
                }

                // Place object to table
                for (size_t i = 0; i < 3; ++i) {
                    armPose.position[i] = places[containerSize][i];
                }
                armPose.orientation[0] = 0;
                armPose.orientation[1] = 0;
                armPose.orientation[2] = 3.1415;

                manipulatorPose.request.pose = armPose;
                ROS_INFO_STREAM("Grasp the object.");
                if (graspObjectClient.call(manipulatorPose)) ROS_INFO_STREAM("Successfull grasp the object");
                else {
                    ROS_FATAL_STREAM("Cant got to camera position.");
                    return false;
                }
                --containerSize;
                objects.erase(objects.begin());
                break;
            }
        }
    }
    return true;
}

void ManipulationControlNode::start() {

    while (nh.ok()) {
        ros::spin();
    }
}
