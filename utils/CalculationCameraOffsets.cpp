#include <arm_kinematics/ArmKinematics.h>

ArmKinematics kinematic;
double mrot[] = {
    0.0167879,    0.999843,  0.00565638,
    -0.998794,   0.0165079,    0.046234,
    0.0461335,   -0.00642542,   0.998915
};

Vector3d getOffsets(JointValues ang, Vector3d & goal, Pose & recognizedObjectPose)
{
    JointValues v;
    matrix::SquareMatrix<double, 3> R;
    matrix::Dcm<double> Roffset(mrot);
    // double phi = recognizedObjectPose.orientation(0);
    // double psi = recognizedObjectPose.orientation(1);
    // double theta = recognizedObjectPose.orientation(2);
    Vector3d cameraPoint = recognizedObjectPose.position, FK, offsets;


    makeKinematicModelOffsets(ang);
    v = kinematic.FK(ang);
    R = kinematic.getRotMatrix(ang);
    FK(0) = v(0); FK(1) = v(1); FK(2) = v(2);

    offsets = inv(R)*(goal - FK) - Roffset*cameraPoint;

    std::cout << "FK: \n" << FK << std::endl;
    std::cout << "goal: \n" << goal << std::endl;
    std::cout << "goal - FK: \n" << goal - FK << std::endl;

    return offsets;
}

int main(int argc, char ** argv)
{
    double ang[] = {2.731876611684124, 1.5674734884112917, -1.2564956897665058, 3.2057740640691175, 2.705685610958246};
    double point[] = {-0.0209, 0.0063, 0.2368};     // Point from camera
    double goal[] = {0.3, 0.05, -0.19};             // Goal point (recognized point)
    // double phi = M_PI/180, theta = -4.5 * M_PI/180, psi = M_PI/2;

    JointValues q(ang);
    Vector3d cameraPoint(point), g(goal), offsets;

    Pose p;
    p.position = cameraPoint;


    offsets = getOffsets(q, g, p);
    std::cout << "Offsets: \n" << offsets << std::endl;

    return 0;
}