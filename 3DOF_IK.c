#include <stdio.h>
#include <math.h>

#define PI 3.14159265

// 3dof robot manipulator struct
struct robot_manipulator {
    int num_joints;
    double joint_angles[3];
    double end_effector_x;
    double end_effector_y;
    double end_effector_z;
};

// inverse kinematics function
void calc_inverse_kinematics(struct robot_manipulator* manipulator) {
    double x = manipulator->end_effector_x;
    double y = manipulator->end_effector_y;
    double z = manipulator->end_effector_z;

    // link lengths
    double l1 = 10;
    double l2 = 10;
    double l3 = 10;

    // joint angles calculation
    manipulator->joint_angles[0] = atan2(y, x);
    double c2 = (x * x + y * y + (z - l1) * (z - l1) - l2 * l2 - l3 * l3) / (2 * l2 * l3);
    manipulator->joint_angles[1] = atan2(sqrt(1 - c2 * c2), c2);
    manipulator->joint_angles[2] = atan2(z - l1, sqrt(x * x + y * y)) - manipulator->joint_angles[1];

    // converting to degrees
    manipulator->joint_angles[0] *= 180 / PI;
    manipulator->joint_angles[1] *= 180 / PI;
    manipulator->joint_angles[2] *= 180 / PI;
}

int main() {
    // Create a manipulator instance
    struct robot_manipulator manipulator;
    manipulator.num_joints = 3;
    printf("Insert end effector's X coordinate:");
    scanf("%f", &manipulator.end_effector_x);
    printf("Insert end effector's Y coordinate:");
    scanf("%f", &manipulator.end_effector_y);
    printf("Insert end effector's Z coordinate:");
    scanf("%f", &manipulator.end_effector_z);

    // calculate the joint parameters
    calc_inverse_kinematics(&manipulator);

    // printing the joint parameters
    printf("Joint angles:\n [%.2f, %.2f, %.2f]\n", manipulator.joint_angles[0], manipulator.joint_angles[1], manipulator.joint_angles[2]);

    return 0;
}