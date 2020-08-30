
#include "JoyShockLibrary.h"
#define _USE_MATH_DEFINES
#include <math.h>

struct Quat
{
	float w;
	float x;
	float y;
	float z;

	Quat()
	{
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Quat(float inW, float inX, float inY, float inZ)
	{
		w = inW;
		x = inX;
		y = inY;
		z = inZ;
	}

	static Quat AngleAxis(float inAngle, float inX, float inY, float inZ)
	{
		Quat result = Quat(cosf(inAngle * 0.5f), inX, inY, inZ);
		result.Normalize();
		return result;
	}

	void Set(float inW, float inX, float inY, float inZ)
	{
		w = inW;
		x = inX;
		y = inY;
		z = inZ;
	}

	Quat& operator*=(const Quat& rhs)
	{
		Set(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
			w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
			w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
			w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
		return *this;
	}

	friend Quat operator*(Quat lhs, const Quat& rhs)
	{
		lhs *= rhs;
		return lhs;
	}

	void Normalize()
	{
		//printf("Normalizing: %.4f, %.4f, %.4f, %.4f\n", w, x, y, z);
		const float length = sqrtf(x * x + y * y + z * z);
		float targetLength = 1.0f - w * w;
		if (targetLength <= 0.0f || length <= 0.0f)
		{
			Set(1.0f, 0.0f, 0.0f, 0.0f);
			return;
		}
		targetLength = sqrtf(targetLength);
		const float fixFactor = targetLength / length;

		x *= fixFactor;
		y *= fixFactor;
		z *= fixFactor;

		//printf("Normalized: %.4f, %.4f, %.4f, %.4f\n", w, x, y, z);
		return;
	}

	Quat Normalized() const
	{
		Quat result = *this;
		result.Normalize();
		return result;
	}

	void Invert()
	{
		x = -x;
		y = -y;
		z = -z;
		return;
	}

	Quat Inverse() const
	{
		Quat result = *this;
		result.Invert();
		return result;
	}
};

struct Vec
{
	float x;
	float y;
	float z;

	Vec()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Vec(float inX, float inY, float inZ)
	{
		x = inX;
		y = inY;
		z = inZ;
	}

	void Set(float inX, float inY, float inZ)
	{
		x = inX;
		y = inY;
		z = inZ;
	}

	float Length() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	void Normalize()
	{
		const float length = Length();
		if (length == 0.0)
		{
			return;
		}
		const float fixFactor = 1.0f / length;

		x *= fixFactor;
		y *= fixFactor;
		z *= fixFactor;
		return;
	}

	Vec Normalized() const
	{
		Vec result = *this;
		result.Normalize();
		return result;
	}

	Vec& operator+=(const Vec& rhs)
	{
		Set(x + rhs.x, y + rhs.y, z + rhs.z);
		return *this;
	}

	friend Vec operator+(Vec lhs, const Vec& rhs)
	{
		lhs += rhs;
		return lhs;
	}

	Vec& operator-=(const Vec& rhs)
	{
		Set(x - rhs.x, y - rhs.y, z - rhs.z);
		return *this;
	}

	friend Vec operator-(Vec lhs, const Vec& rhs)
	{
		lhs -= rhs;
		return lhs;
	}

	Vec& operator*=(const float rhs)
	{
		Set(x * rhs, y * rhs, z * rhs);
		return *this;
	}

	friend Vec operator*(Vec lhs, const float rhs)
	{
		lhs *= rhs;
		return lhs;
	}

	Vec& operator/=(const float rhs)
	{
		Set(x / rhs, y / rhs, z / rhs);
		return *this;
	}

	friend Vec operator/(Vec lhs, const float rhs)
	{
		lhs /= rhs;
		return lhs;
	}

	Vec& operator*=(const Quat& rhs)
	{
		Quat temp = rhs * Quat(0.0f, x, y, z) * rhs.Inverse();
		Set(temp.x, temp.y, temp.z);
		return *this;
	}

	friend Vec operator*(Vec lhs, const Quat& rhs)
	{
		lhs *= rhs;
		return lhs;
	}

	Vec operator-() const {
		Vec result = Vec(-x, -y, -z);
		return result;
	}

	float Dot(const Vec& other) const
	{
		return x * other.x + y * other.y + z * other.z;
	}

	Vec Cross(const Vec& other) const
	{
		return Vec(y * other.z - z * other.y,
			z * other.x - x * other.z,
			x * other.y - y * other.x);
	}
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

	Motion()
	{
		Reset();
	}

	MOTION_STATE GetMotionState()
	{
		MOTION_STATE result = MOTION_STATE();
		result.quatW = Quaternion.w;
		result.quatX = Quaternion.x;
		result.quatY = Quaternion.y;
		result.quatZ = Quaternion.z;
		result.accelX = Accel.x;
		result.accelY = Accel.y;
		result.accelZ = Accel.z;
		result.gravX = Grav.x;
		result.gravY = Grav.y;
		result.gravZ = Grav.z;
		return result;
	}

	void Reset()
	{
		Quaternion.Set(1.0f, 0.0f, 0.0f, 0.0f);
		Accel.Set(0.0f, 0.0f, 0.0f);
		Grav.Set(0.0f, 0.0f, 0.0f);
		NumGravDirectionSamplesCounted = 0;
	}

	/// <summary>
	/// The gyro inputs should be calibrated degrees per second but have no other processing
	/// </summary>
	void Update(float inGyroX, float inGyroY, float inGyroZ, float inAccelX, float inAccelY, float inAccelZ, float gravityLength, float deltaTime)
	{
		const Vec axis = Vec(inGyroX, inGyroY, inGyroZ);
		const Vec accel = Vec(inAccelX, inAccelY, inAccelZ);
		float angle = axis.Length() * (float)M_PI / 180.0f;
		angle *= deltaTime;

		// rotate
		Quat rotation = Quat::AngleAxis(angle, axis.x, axis.y, axis.z);
		Quaternion *= rotation; // do it this way because it's a local rotation, not global
		//printf("Quat: %.4f %.4f %.4f %.4f _",
		//	Quaternion.w, Quaternion.x, Quaternion.y, Quaternion.z);
		float accelMagnitude = accel.Length();
		if (accelMagnitude > 0.0f)
		{
			const Vec accelNorm = accel / accelMagnitude;
			LastGravityIdx = (LastGravityIdx + NumGravDirectionSamples - 1) % NumGravDirectionSamples;
			// for comparing and perhaps smoothing gravity samples, we need them to be global
			Vec absoluteAccel = accel * Quaternion;
			GravDirectionSamples[LastGravityIdx] = absoluteAccel;
			Vec gravityMin = absoluteAccel;
			Vec gravityMax = absoluteAccel;
			//GravDirectionSamples[LastGravityIdx] = accel;
			//Vec gravityMin = accel;
			//Vec gravityMax = accel;
			const float steadyGravityThreshold = 0.05f;
			NumGravDirectionSamplesCounted++;
			const int numGravSamples = NumGravDirectionSamplesCounted < NumGravDirectionSamples ? NumGravDirectionSamplesCounted : NumGravDirectionSamples;
			for (int idx = 1; idx < numGravSamples; idx++)
			{
				Vec thisSample = GravDirectionSamples[(LastGravityIdx + idx) % NumGravDirectionSamples];
				if (thisSample.x > gravityMax.x)
				{
					gravityMax.x = thisSample.x;
				}
				if (thisSample.y > gravityMax.y)
				{
					gravityMax.y = thisSample.y;
				}
				if (thisSample.z > gravityMax.z)
				{
					gravityMax.z = thisSample.z;
				}
				if (thisSample.x < gravityMin.x)
				{
					gravityMin.x = thisSample.x;
				}
				if (thisSample.y < gravityMin.y)
				{
					gravityMin.y = thisSample.y;
				}
				if (thisSample.y < gravityMin.y)
				{
					gravityMin.z = thisSample.z;
				}
			}
			const Vec gravityBoxSize = gravityMax - gravityMin;
			//printf(" Gravity Box Size: %.4f _ ", gravityBoxSize.Length());
			if (gravityBoxSize.x <= steadyGravityThreshold &&
				gravityBoxSize.y <= steadyGravityThreshold &&
				gravityBoxSize.z <= steadyGravityThreshold)
			{
				absoluteAccel = gravityMin + (gravityBoxSize * 0.5f);
				//const Vec localGravity = -(absoluteAccel * Quaternion.Inverse());
				const Vec gravityDirection = -absoluteAccel.Normalized();
				//const Vec localGravityNormalized = localGravity.Normalized();
				const Vec expectedGravity = Vec(0.0f, -1.0f, 0.0f) * Quaternion.Inverse();
				//const float errorAngle = acosf(expectedLocalGravity.Normalized().Dot(localGravityNormalized)) * 180.0f / (float)M_PI;
				const float errorAngle = acosf(Vec(0.0f, -1.0f, 0.0f).Dot(gravityDirection)) * 180.0f / (float)M_PI;

				const Vec flattened = gravityDirection.Cross(Vec(0.0f, -1.0f, 0.0f)).Normalized();

				if (errorAngle > 0.0f)
				{
					const float EaseInTime = 0.25f;
					TimeCorrecting += deltaTime;

					//printf("Angle Error: %.4f\n", errorAngle);
					//printf("Angle Error: %.4f ... Local Gravity: %.4f, %.4f, %.4f\n             Expected Local Gravity: %.4f, %.4f, %.4f   (%.4f, %.4f, %.4f, %.4f)... ",
					//	errorAngle,
					//	localGravity.x, localGravity.y, localGravity.z,
					//	expectedLocalGravity.x, expectedLocalGravity.y, expectedLocalGravity.z,
					//	Quaternion.w, Quaternion.x, Quaternion.y, Quaternion.z);

					const float tighteningThreshold = 5.0f;
					//const float maxCorrectionRate = 15.0f * deltaTime;

					float confidentSmoothCorrect = errorAngle;
					//if (errorAngle < tighteningThreshold)
					//{
					//	confidentSmoothCorrect *= errorAngle / tighteningThreshold;
					//}
					confidentSmoothCorrect *= 1.0f - exp2f(-deltaTime * 4.0f);
					if (TimeCorrecting < EaseInTime)
					{
						confidentSmoothCorrect *= TimeCorrecting / EaseInTime;
					}
					//confidentSmoothCorrect *= deltaTime;
					//if (confidentSmoothCorrect > maxCorrectionRate)
					//{
					//	confidentSmoothCorrect = maxCorrectionRate;
					//}
					Quaternion = Quat::AngleAxis(confidentSmoothCorrect * (float)M_PI / 180.0f, flattened.x, flattened.y, flattened.z) * Quaternion;
				}
				else
				{
					TimeCorrecting = 0.0f;
				}

				Grav = Vec(0.0f, -gravityLength, 0.0f) * Quaternion.Inverse();
				//const Vec localGravityNormalized = -(absoluteAccel * Quaternion.Inverse());
				//Vec trueGrav = localGravityNormalized * gravityLength;
				//Accel = accel + trueGrav;
				Accel = accel + Grav; // gravity won't be shaky. accel might. so let's keep using the quaternion's calculated gravity vector.

				//printf("NEW GRAVITY VECTOR ... ");
			}
			else
			{
				TimeCorrecting = 0.0f;
				Grav = Vec(0.0f, -gravityLength, 0.0f) * Quaternion.Inverse();
				Accel = accel + Grav;
			}
		}
		else
		{
			TimeCorrecting = 0.0f;
			Accel.Set(0.0f, 0.0f, 0.0f);
		}
		Quaternion.Normalize();
	}
};