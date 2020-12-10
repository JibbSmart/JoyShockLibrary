#pragma once

#include "JoyShockLibrary.h"

struct Quat
{
	float w;
	float x;
	float y;
	float z;

	Quat();

	Quat(float inW, float inX, float inY, float inZ);

	static Quat AngleAxis(float inAngle, float inX, float inY, float inZ);

	void Set(float inW, float inX, float inY, float inZ);

	Quat& operator*=(const Quat& rhs);

	friend Quat operator*(Quat lhs, const Quat& rhs);

	void Normalize();

	Quat Normalized() const;

	void Invert();

	Quat Inverse() const;
};

struct Vec
{
	float x;
	float y;
	float z;

	Vec();

	Vec(float inX, float inY, float inZ);

	void Set(float inX, float inY, float inZ);

	float Length() const;

	void Normalize();

	Vec Normalized() const;

	Vec& operator+=(const Vec& rhs);

	friend Vec operator+(Vec lhs, const Vec& rhs);

	Vec& operator-=(const Vec& rhs);

	friend Vec operator-(Vec lhs, const Vec& rhs);

	Vec& operator*=(const float rhs);

	friend Vec operator*(Vec lhs, const float rhs);

	Vec& operator/=(const float rhs);

	friend Vec operator/(Vec lhs, const float rhs);

	Vec& operator*=(const Quat& rhs);

	friend Vec operator*(Vec lhs, const Quat& rhs);

	Vec operator-() const;

	float Dot(const Vec& other) const;

	Vec Cross(const Vec& other) const;
};

struct Motion
{
	Quat Quaternion;
	Vec Accel;
	Vec Grav;

	const int NumGravDirectionSamples = 10;
	Vec GravDirectionSamples[10];
	int LastGravityIdx = 9;
	int NumGravDirectionSamplesCounted = 0;
	float TimeCorrecting = 0.0f;

	Motion();

	MOTION_STATE GetMotionState();

	void Reset();

	/// <summary>
	/// The gyro inputs should be calibrated degrees per second but have no other processing
	/// </summary>
	void Update(float inGyroX, float inGyroY, float inGyroZ, float inAccelX, float inAccelY, float inAccelZ, float gravityLength, float deltaTime);
};