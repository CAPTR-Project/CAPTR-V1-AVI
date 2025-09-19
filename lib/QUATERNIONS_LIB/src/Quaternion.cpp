#include "quaternion.h"

/*** Constructors ***/
// -- Quaternion --
Quaternion::Quaternion()
{
	/***
	Quaternion Default Constructor
	***/
	this->s = 1;
	this->v_1 = 0.0;
	this->v_2 = 0.0;
	this->v_3 = 0.0;
}

Quaternion::Quaternion(float s, float v_1, float v_2, float v_3)
{
	/***
	Quaternion Generalized Constructor

	Inputs:
	s: scalar part of quaternion
	v_1: quaternion param of vector part of quaternion
	v_2: quaternion param of vector part of quaternion
	v_3: quaternion param of vector part of quaternion
	***/
	this->s = s;
	this->v_1 = v_1;
	this->v_2 = v_2;
	this->v_3 = v_3;
}


// Create a quaternion from roll,pitch,yaw angle
Quaternion::Quaternion(float roll, float pitch, float yaw)
{
	/***
	Quaternion Constructor from roll pitch yaw angles

	Inputs:
	roll: roll angle (rads)
	pitch: pitch angle (rads)
	yaw: yaw angle (rads)
	***/

	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);

	this->s = cr * cp * cy + sr * sp * sy;
	this->v_1 = sr * cp * cy - cr * sp * sy;
	this->v_2 = cr * sp * cy + sr * cp * sy;
	this->v_3 = cr * cp * sy - sr * sp * cy;
}


// -- UnitQuaternion --
UnitQuaternion::UnitQuaternion()
{
	/***
	Unit Quaternion Default Constructor
	***/
	this->s = 1;
	this->v_1 = 0.0;
	this->v_2 = 0.0;
	this->v_3 = 0.0;
}

UnitQuaternion::UnitQuaternion(float s, float v_1, float v_2, float v_3)
{
	/***
	Unit Quaternion Default Constructor

	Inputs:
	s: scalar part of unit quaternion
	v_1: quaternion param of vector part of unit quaternion
	v_2: quaternion param of vector part of unit quaternion
	v_3: quaternion param of vector part of unit quaternion
	***/	
	this->s = s;
	this->v_1 = v_1;
	this->v_2 = v_2;
	this->v_3 = v_3;
	(*this).normalize();
}


UnitQuaternion UnitQuaternion::from_rotVec(float wx, float wy, float wz)
{
	/***
	Unit Quaternion Constructor from angle times rotation vector.

	Constructs a UnitQuaternion from angle times rotation vector.
	EQN reference: https://math.stackexchange.com/questions/39553/how-do-i-apply-an-angular-velocity-vector3-to-a-unit-quaternion-orientation

	Inputs:
	s: scalar part of unit quaternion
	v_1: quaternion param of vector part of unit quaternion
	v_2: quaternion param of vector part of unit quaternion
	v_3: quaternion param of vector part of unit quaternion
	***/
	UnitQuaternion uq;

	float theta = sqrt(wx*wx + wy*wy + wz*wz);	// Add a little bit of eps to avoid blowing up

	if (abs(theta) < 1e-12) {
		uq.s = 1;
		uq.v_1 = 0;
		uq.v_2 = 0;
		uq.v_3 = 0;
	} else {

	uq.s = cos(theta/2.0);
	uq.v_1 = sin(theta/2.0) * (wx / theta);
	uq.v_2 = sin(theta/2.0) * (wy / theta);
	uq.v_3 = sin(theta/2.0) * (wz / theta);

	}

	return uq;
}

UnitQuaternion UnitQuaternion::from_euler(float yaw, float pitch, float roll)
{
	/***
	Unit Quaternion Constructor from Euler angles

	Constructs a UnitQuaternion from Euler angles.
	EQN reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	Inputs:
	roll: roll angle (rads)
	pitch: pitch angle (rads)
	yaw: yaw angle (rads)
	***/
	UnitQuaternion uq;

	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);
	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);

	uq.s = cr * cp * cy + sr * sp * sy;
	uq.v_1 = sr * cp * cy - cr * sp * sy;
	uq.v_2 = cr * sp * cy + sr * cp * sy;
	uq.v_3 = cr * cp * sy - sr * sp * cy;

	return uq;
}

/*** Destructors ***/
// Quaternion::~Quaternion()
// {
// 	// Destructor
// }


/*** Quaternion Operators ***/
// -- Quaternions --
Quaternion Quaternion::operator+(const Quaternion q2)
{
	/***
	Quaternion Addition
	***/
	Quaternion quaternion_result;
	quaternion_result.s = this->s + q2.s;
	quaternion_result.v_1 = this->v_1 + q2.v_1;
	quaternion_result.v_2 = this->v_2 + q2.v_2;
	quaternion_result.v_3 = this->v_3 + q2.v_3;
	return quaternion_result;
}



Quaternion Quaternion::operator-(const Quaternion q2)
{
	/***
	Quaternion Subtraction
	***/	
	Quaternion quaternion_result;
	quaternion_result.s = this->s - q2.s;
	quaternion_result.v_1 = this->v_1 - q2.v_1;
	quaternion_result.v_2 = this->v_2 - q2.v_2;
	quaternion_result.v_3 = this->v_3 - q2.v_3;
	return quaternion_result;
}


Quaternion Quaternion::operator*(const Quaternion q2)
{
	/***
	Quaternion Multiplication (Hamiltonian product)
	***/	
	Quaternion quaternion_result;
	quaternion_result.s = s*q2.s - v_1*q2.v_1 - v_2*q2.v_2 - v_3*q2.v_3;
	quaternion_result.v_1 = v_1*q2.s + s*q2.v_1 + v_2*q2.v_3 - v_3*q2.v_2;
	quaternion_result.v_2 = s*q2.v_2 - v_1*q2.v_3 + v_2*q2.s + v_3*q2.v_1;
	quaternion_result.v_3 = s*q2.v_3 + v_1*q2.v_2 - v_2*q2.v_1 + v_3*q2.s;
	return quaternion_result;
}



UnitQuaternion UnitQuaternion::operator+(const UnitQuaternion q2)
{
	/***
	Unit Quaternion Addition
	***/
	UnitQuaternion quaternion_result;
	quaternion_result.s = this->s + q2.s;
	quaternion_result.v_1 = this->v_1 + q2.v_1;
	quaternion_result.v_2 = this->v_2 + q2.v_2;
	quaternion_result.v_3 = this->v_3 + q2.v_3;
	return quaternion_result;
}


// -- Unit Quaternions --
UnitQuaternion UnitQuaternion::operator-(const UnitQuaternion q2)
{
	/***
	Unit Quaternion Subtraction
	***/
	UnitQuaternion quaternion_result;
	quaternion_result.s = this->s - q2.s;
	quaternion_result.v_1 = this->v_1 - q2.v_1;
	quaternion_result.v_2 = this->v_2 - q2.v_2;
	quaternion_result.v_3 = this->v_3 - q2.v_3;
	return quaternion_result;
}


UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion q2)
{
	/***
	Unit Quaternion Multiplication
	***/
	UnitQuaternion quaternion_result;
	quaternion_result.s = s*q2.s - v_1*q2.v_1 - v_2*q2.v_2 - v_3*q2.v_3;
	quaternion_result.v_1 = v_1*q2.s + s*q2.v_1 + v_2*q2.v_3 - v_3*q2.v_2;
	quaternion_result.v_2 = s*q2.v_2 - v_1*q2.v_3 + v_2*q2.s + v_3*q2.v_1;
	quaternion_result.v_3 = s*q2.v_3 + v_1*q2.v_2 - v_2*q2.v_1 + v_3*q2.s;
	return quaternion_result;
}


/*** Quaternion Inverse/Conjugate (since inverse of quaternion is its conjugate) ***/
// -- Quaternions --
Quaternion Quaternion::conjugate()
{
	/***
	Quaternion Conjugate
	***/
	Quaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

Quaternion Quaternion::inverse()
{
	/***
	Quaternion Inverse

	Note that the inverse of quaternion is just its conjugate
	***/
	Quaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

// -- Unit Quaternions --
UnitQuaternion UnitQuaternion::conjugate()
{
	/***
	Unit Quaternion Conjugate
	***/
	UnitQuaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

UnitQuaternion UnitQuaternion::inverse()
{
	/***
	Unit Quaternion Inverse

	Note that the inverse of quaternion is just its conjugate
	***/
	UnitQuaternion quat_conjugate(s,-v_1,-v_2,-v_3);
	return quat_conjugate;
}

/*** Quaternion rotation by vector (v' = dot(q,v,q_conj)) ***/
Quaternion Quaternion::vector_rotation_by_quaternion(const Quaternion v)
{
	/***
	Apply a quaternion rotation to a vector.
	EQN Reference: Robotics vision and control quaternion vector rotation (p45)

	Input:
	v: vector to be rotated by quaternion
	***/
	return (*this) * v * conjugate();

}

Eigen::Vector3d UnitQuaternion::vector_rotation_by_quaternion(Eigen::Vector3d v)
{
	/***
	Apply a quaternion rotation to a vector.
	EQN Reference: Robotics vision and control quaternion vector rotation (p45)

	Input:
	v: vector to be rotated by quaternion
	
	Output:
	rotated_vec: vector after being rotated by quaternion. Returns the vector itself instead 
				 of a pure quaternion.
	***/
	UnitQuaternion v_unit_quat = UnitQuaternion(0, v(0), v(1), v(2));
	UnitQuaternion quat_result =  (*this) * v_unit_quat * conjugate();

	Eigen::Vector3d  rotated_vec(quat_result.v_1, quat_result.v_2, quat_result.v_3);
	return rotated_vec;		
}


/*** Normalize Quaternion ***/
float Quaternion::norm2()
{
	/*** 
	Returns the magnitude of quaternions
	***/
	float norm2 = s*s + v_1*v_1 + v_2*v_2 + v_3*v_3;
	return sqrt(norm2);
}

void Quaternion::normalize()
{
	/*** 
	Returns a normalize quaternion
	***/	
	float q_magnitude = norm2() + 1.0e-12;
	s /= q_magnitude;
	v_1 /= q_magnitude;
	v_2 /= q_magnitude;
	v_3 /= q_magnitude;
}


/*** Quaternion to RPY ***/
float Quaternion::get_roll()
{
	/***
	Get roll from quaternion
	***/
	float roll = atan2(2*(s*v_1 + v_2*v_3), 1 - 2*(v_1*v_1 + v_2*v_2));
	return roll;
}

float Quaternion::get_pitch()
{
	/***
	Get pitch from quaternion
	***/	
	float sinp = 2*(s*v_2 - v_3*v_1);
	return asin(sinp);
}

float Quaternion::get_yaw()
{
	/***
	Get yaw from quaternion
	***/
	float yaw = atan2(2*(s*v_3 + v_1*v_2), 1 - 2*(v_2*v_2 + v_3*v_3));
	return yaw;
}


/*** To vector method ***/
Eigen::Vector4d Quaternion::to_quaternion_vector()	
{
	/***
	Output the quaternion into a Eigen::Vector4d
	***/	
	Eigen::Vector4d quat_vect(this->s, this->v_1, this->v_2, this->v_3);
	return quat_vect;
}

UnitQuaternion UnitQuaternion::average_quaternions(std::vector<UnitQuaternion> quaternions, std::vector<double> weights) {
	Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
	int sz = quaternions.size();
	double wSum = 0;

	for (int i = 0; i < sz; i++) {
		Eigen::Vector4d q = quaternions[i].to_quaternion_vector();
		double w_i = weights[i];
		A = w_i * (q * q.transpose()) + A;
		wSum += weights[i];
	}

	A = (1 / wSum) * A;

	Eigen::EigenSolver<Eigen::Matrix4d> es(A);
	Eigen::Vector4cd eig = es.eigenvalues();
	Eigen::Matrix4cd eigVec = es.eigenvectors();

	int maxIdx = 0;
	double maxVal = eig[0].real();
	for (int i = 1; i < 4; i++) {
		if (eig[i].real() > maxVal) {
			maxVal = eig[i].real();
			maxIdx = i;
		}
	}

	Eigen::Vector4d avgQuat = eigVec.col(maxIdx).real();
	return UnitQuaternion(avgQuat(0), avgQuat(1), avgQuat(2), avgQuat(3));
}

Eigen::Vector3d UnitQuaternion::to_rotVec() {
	Eigen::Vector3d rotVec;
	rotVec << v_1, v_2, v_3;
	double theta = 2 * atan2(rotVec.norm(), s);
	if (theta == 0) rotVec = Eigen::Vector3d::Zero();
	else rotVec = theta * rotVec / rotVec.norm();
	return rotVec;
}

Eigen::Vector4d UnitQuaternion::to_axis_angle() {
	Eigen::Vector3d rotVec;
	rotVec << v_1, v_2, v_3;
	Eigen::Vector4d axis_angle;
	axis_angle.head<3>() = rotVec;
	axis_angle(3) = 2 * atan2(rotVec.norm(), s);
	if (axis_angle(3) == 0) axis_angle.head<3>() = Eigen::Vector3d::Zero();
	else axis_angle.head<3>() = axis_angle.head<3>() / axis_angle.head<3>().norm();
	return axis_angle;
}

Eigen::Vector3d UnitQuaternion::to_euler() {
	// double roll = atan2(2 * (s * v_1 + v_2 * v_3), 1 - 2 * (v_1 * v_1 + v_2 * v_2));
	// double pitch = asin(2 * (s * v_2 - v_3 * v_1));
	// double yaw = atan2(2 * (s * v_3 + v_1 * v_2), 1 - 2 * (v_2 * v_2 + v_3 * v_3));
	// return Eigen::Vector3d(roll, pitch, yaw);

	Eigen::Vector3d angles;    //yaw pitch roll
    float x = v_1;
    float y = v_2;
    float z = v_3;
    float w = s;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[0] = atan2(siny_cosp, cosy_cosp);
    return angles;
}
