#ifndef QUATERNION_H
#define QUATERNION_H

#include "Arduino.h"
#include "ArduinoEigenDense.h"
#include <ArduinoEigen/Eigen/Core>
#include <ArduinoEigen/Eigen/Dense>
#include <ArduinoEigen/Eigen/Cholesky>

#include <vector> 
#include <math.h>

class Quaternion
{
public:
	// Scalar part of Quaternion
	float s = 1;

	// Orthogonal complex number (vector part of Quaternion) 
	float v_1 = 0, v_2 = 0, v_3 = 0;

	Quaternion();
	Quaternion(float s, float v_1, float v_2, float v_3);
	Quaternion(float roll, float pitch, float yaw);
	// ~Quaternion();


	/*** Quaternion Operators ***/
	Quaternion operator+ (const Quaternion q2); // q3 = q1+q2
	Quaternion operator- (const Quaternion q2); // q3 = q1-q2
	Quaternion operator* (const Quaternion q2); // q3 = q1 * q2
	Quaternion operator/ (const Quaternion q2); // q3 = q1 / q2

	/*** Inverse/Conjugate ***/
	Quaternion conjugate();
	Quaternion inverse();

	/*** Vector rotated by quaternion ***/
	// Note: v is pure quaternion i.e v = 0<v> 
	Quaternion vector_rotation_by_quaternion(const Quaternion v);

	/*** Normalize quaternion so that it becomes UnitQuaternion ***/
	float norm2();
	void normalize();

	/*** Quaternion to RPY ***/
	float get_roll();
	float get_pitch();
	float get_yaw();

	/*** To vector method ***/
	Eigen::Vector4d to_quaternion_vector();
};

class UnitQuaternion : public Quaternion
{
public:
	UnitQuaternion();
	UnitQuaternion(float s, float v_1, float v_2, float v_3);
	static UnitQuaternion from_rotVec(float wx, float wy, float wz);
	static UnitQuaternion from_euler(float yaw, float pitch, float roll);
    static UnitQuaternion from_axis_angle(Eigen::Vector3d axis, float angle);

	static UnitQuaternion average_quaternions(std::vector<UnitQuaternion> quaternions, std::vector<double> weights);

	/*** UnitQuaternion operators ***/
	UnitQuaternion operator+ (const UnitQuaternion q2); // q3 = q1+q2
	UnitQuaternion operator- (const UnitQuaternion q2); // q3 = q1-q2
	UnitQuaternion operator* (const UnitQuaternion q2); // q3 = q1 * q2
	UnitQuaternion operator/ (const UnitQuaternion q2); // q3 = q1 / q2

	/*** Inverse/Conjugate ***/
	UnitQuaternion conjugate();
	UnitQuaternion inverse();

	/**
	 * @brief Convert quaternion to euler angles. 
	 * 
	 * @return Eigen::Vector3d the euler angles, in Body 3-2-1: Yaw, Pitch, Roll
	 */
	Eigen::Vector3d to_euler();
	Eigen::Vector3d to_rotVec();
	Eigen::Vector4d to_axis_angle();

	/*** Vector rotated by quaternion ***/
	// Note: v is pure quaternion i.e v = 0<v> 
	// BLA::Matrix<3> vector_rotation_by_quaternion(BLA::Matrix<3> v);
	Eigen::Vector3d vector_rotation_by_quaternion(Eigen::Vector3d v);
	

};



#endif