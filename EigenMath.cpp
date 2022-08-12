#include "EigenMath.h"
using namespace Eigen;

Eigen::Matrix4f quaternionToRot(float q[4])      //q=(w,x,y,z)  四元数转旋转矩阵
{
	Eigen::Matrix4f R;
	R(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	R(0, 1) = 2.0*(q[1] * q[2] - q[0] * q[3]);
	R(0, 2) = 2.0*(q[0] * q[2] + q[1] * q[3]);
	R(0, 3) = 0;
	R(1, 0) = 2.0*(q[0] * q[3] + q[1] * q[2]);
	R(1, 1) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	R(1, 2) = 2.0*(q[2] * q[3] - q[0] * q[1]);
	R(1, 3) = 0;
	R(2, 0) = 2.0*(q[1] * q[3] - q[0] * q[2]);
	R(2, 1) = 2.0*(q[0] * q[1] + q[2] * q[3]);
	R(2, 2) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	R(2, 3) = 0;
	R(3, 0) = 0;
	R(3, 1) = 0;
	R(3, 2) = 0;
	R(3, 3) = 1;
	return R;
}
void rotToQuaternion1(double R[3][3], double q[4])//旋转矩阵转四元数
{
	float tr = R[0][0] + R[1][1] + R[2][2];
	float temp = 0.0;
	if (tr > 0.0)
	{
		temp = 0.5f / sqrtf(tr + 1);
		q[0] = 0.25f / temp;
		q[1] = (R[2][1] - R[1][2]) * temp;
		q[2] = (R[0][2] - R[2][0]) * temp;
		q[3] = (R[1][0] - R[0][1]) * temp;
	}
	else
	{
		if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
		{
			temp = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
			q[0] = (R[2][1] - R[1][2]) / temp;
			q[1] = 0.25f * temp;
			q[2] = (R[0][1] + R[1][0]) / temp;
			q[3] = (R[0][2] + R[2][0]) / temp;
		}
		else if (R[1][1] > R[2][2])
		{
			temp = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
			q[0] = (R[0][2] - R[2][0]) / temp;
			q[1] = (R[0][1] + R[1][0]) / temp;
			q[2] = 0.25f * temp;
			q[3] = (R[1][2] + R[2][1]) / temp;
		}
		else
		{
			temp = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
			q[0] = (R[1][0] - R[0][1]) / temp;
			q[1] = (R[0][2] + R[2][0]) / temp;
			q[2] = (R[1][2] + R[2][1]) / temp;
			q[3] = 0.25f * temp;
		}
	}
	if (q[3] > 0)
	{
		q[3] = sqrt(1 - q[1] * q[1] - q[2] * q[2] - q[0] * q[0]);
	}
	else
	{
		q[3] = -sqrt(1 - q[1] * q[1] - q[2] * q[2] - q[0] * q[0]);
	}
	if (q[0] < 0)
	{
		q[0] = -q[0];
		q[1] = -q[1];
		q[2] = -q[2];
		q[3] = -q[3];
	}
}

Eigen::Matrix4f poseToRT(Eigen::Vector3d euler, EnumHcRotationStatus direction)
{
	float x, y, z;
	x = euler.x();
	y = euler.y();
	z = euler.z();
	Eigen::Matrix4f R_z;
	Eigen::Matrix4f R_y;
	Eigen::Matrix4f R_x;
	Eigen::Matrix4f R_T;
	R_z << cos(z), -sin(z), 0, 0, sin(z), cos(z), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	R_y << cos(y), 0, sin(y), 0, 0, 1, 0, 0, -sin(y), 0, cos(y), 0, 0, 0, 0, 1;
	R_x << 1, 0, 0, 0, 0, cos(x), -sin(x), 0, 0, sin(x), cos(x), 0, 0, 0, 0, 1;
	switch (direction)
	{
	case(RXYZn):
		R_T = R_x * R_y*R_z;
		break;
	case(RXZYn):
		R_T = R_x * R_z*R_y;
		break;
	case(RYXZn):
		R_T = R_y * R_x*R_z;
		break;
	case(RYZXn):
		R_T = R_y * R_z*R_x;
		break;
	case(RZXYn):
		R_T = R_z * R_x*R_y;
		break;
	case(RZYXn):
		R_T = R_z * R_y*R_x;
		break;
	}
	return R_T;
}
Eigen::Matrix4d Rot_z(float z)
{
	Eigen::Matrix4d R_z;
	R_z << cos(z), -sin(z), 0, 0, sin(z), cos(z), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
	return R_z;
}
Eigen::Matrix4d Rot_y(float y)
{
	Eigen::Matrix4d R_y;
	R_y << cos(y), 0, sin(y), 0, 0, 1, 0, 0, -sin(y), 0, cos(y), 0, 0, 0, 0, 1;
	return R_y;
}
Eigen::Matrix4d Rot_x(float x)
{
	Eigen::Matrix4d R_x;
	R_x << 1, 0, 0, 0, 0, cos(x), -sin(x), 0, 0, sin(x), cos(x), 0, 0, 0, 0, 1;
	return R_x;
}
Eigen::Matrix4d verticalLine(Eigen::Vector3d line1, Eigen::Vector3d line2)
{
	Eigen::Vector3d axis_x = line1;
	Eigen::Vector3d axis_z = axis_x.cross(line2);
	Eigen::Vector3d axis_y = axis_x.cross(axis_z);
	axis_x.normalize();
	axis_y.normalize();
	axis_z.normalize();
	Eigen::Matrix4d mat;
	mat << axis_x(0), axis_y(0), axis_z(0), line2(0)/2,
		axis_x(1), axis_y(1), axis_z(1), line2(1)/2,
		axis_x(2), axis_y(2), axis_z(2), line2(2)/2,
		0, 0, 0, 1;
	return mat;
}