#pragma once
#pragma execution_character_set("utf-8")
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <iostream>
#include <string>
using namespace std;

/**@brief Set the quaternion using fixed axis RPY
* @param roll Angle around X
* @param pitch Angle around Y
* @param yaw Angle around Z*/
static inline Eigen::Matrix4d createQuaternionFromRPY(const float& roll, const float& pitch, const float& yaw, const float tx, const float ty, const float tz)
{
	float halfYaw = float(yaw) * float(0.5);
	float halfPitch = float(pitch) * float(0.5);
	float halfRoll = float(roll) * float(0.5);
	float cosYaw = cos(halfYaw);
	float sinYaw = sin(halfYaw);
	float cosPitch = cos(halfPitch);
	float sinPitch = sin(halfPitch);
	float cosRoll = cos(halfRoll);
	float sinRoll = sin(halfRoll);
	Eigen::Quaterniond q(
		cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw, //formerly yzx
		sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
		cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
		cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw);//z
	Eigen::Matrix3d R = q.normalized().toRotationMatrix();
	Eigen::Vector4d t(tx,ty,tz,1);
	Eigen::Matrix4d T;
	T.setZero();
	T.block(0, 0, 3, 3) = R.block(0,0,3,3);
	T.block(0, 3, 4, 1) = t.head(4);
	//cout << t.head(4) << endl;
	//cout << T.block(0, 3, 4, 1) << endl;
	//cout << T.block(0, 0, 4, 4) << endl;
	return T;
}
