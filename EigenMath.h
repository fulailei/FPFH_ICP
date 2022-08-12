#include <Eigen/Core>
#include "Eigen/Eigen"
#define M_PI 3.1415926
enum EnumHcRotationStatus
{
	RXYZn, RXZYn, RYXZn, RYZXn, RZXYn, RZYXn	   //zyx
};

Eigen::Matrix4f quaternionToRot(float q[4]);

void rotToQuaternion1(double R[3][3], double q[4]);

Eigen::Matrix4f poseToRT(Eigen::Vector3d euler, EnumHcRotationStatus direction);

Eigen::Matrix4d Rot_z(float z);

Eigen::Matrix4d Rot_y(float y);

Eigen::Matrix4d Rot_x(float x);

Eigen::Matrix4d verticalLine(Eigen::Vector3d line1, Eigen::Vector3d line2);
