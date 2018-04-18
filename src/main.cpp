#include <local_task_planner/LTP.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "local_tp");
    ros::NodeHandle nh;

    Configuration conf;
    conf.mode = 1;
    conf.cvServiceName = "get_vision";
    conf.gtpServiceName = "pick_object";
    conf.manipServiceName = {"manipulator_pose", "MoveLine"};

    localTP tp(nh, conf);
    // LTP localTP(conf);
    while(ros::ok()) {
    	ros::spin();
    }

    return 0;
}