/* ---------------------------------------------------------------------------------------------------
    derived from Simon D. Levy's multi-variable multi-sensor EKF example
    https://github.com/simondlevy/TinyEKF/blob/master/examples/SensorFusion/SensorFusion.ino

    thank you Simon for making this library and the example code!
--------------------------------------------------------------------------------------------------- */

// These must be defined before including TinyEKF.h
#define EKF_N 10     // number of measurements
#define EKF_M 10     // number of sensors

#include <stdio.h>
#include "tinyekf.h"

using namespace std;

static const float EPS = 1e-4;

float tempTrust = 1;
float pressureTrust = 1;
float altitudeTrust = 1;
float velocityTrust = 1;
float gyroXTrust = 1;
float gyroYTrust = 1;
float gyroZTrust = 1;
float accelXTrust = 1;
float accelYTrust = 1;
float accelZTrust = 1;

static const float Q[EKF_N*EKF_N] = {

	EPS*tempTrust, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, EPS*pressureTrust, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, EPS*altitudeTrust, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, EPS*velocityTrust, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, EPS*gyroXTrust, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, EPS*gyroYTrust, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, EPS*gyroZTrust, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, EPS*accelXTrust, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, EPS*accelYTrust, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, EPS*accelZTrust
};

static const float R[EKF_M*EKF_M] = {

	EPS, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, EPS, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, EPS, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, EPS, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, EPS, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, EPS, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, EPS, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, EPS, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, EPS, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, EPS
};

// So process model Jacobian is identity matrix
static const float F[EKF_N*EKF_N] = {
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1
};

static const float H[EKF_M*EKF_N] = {

	1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1
};

static ekf_t _ekf;

// this code is from h1, it will have to be changed for this sensor layout

float baroTemperature = 0.0;
float baroPressure = 0.0;
float altitude = 0.0;
float velocity = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float filtered[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void updateValues(){
	float baroTemperature = Temp;
	float baroPressure = Pressure;
	float altitude = Alt;
	float velocity = imuvelocity;
	float gyroX = imugyroX;
	float gyroY = imugyroY;
	float gyroZ = imugyroZ;
	float accelX = imuaccelX;
	float accelY = imuaccelY;
	float accelZ = imuaccelZ;
}

void filterSetup() {
	// Use identity matrix as initiali covariance matrix
	const float Pdiag[EKF_N] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

	ekf_initialize(&_ekf, Pdiag);
}

void filterLoop() {

	// Set the observation vector z
	const float z[EKF_M] = {baroTemperature, baroPressure, altitude, velocity,
	                        gyroX, gyroY, gyroZ, accelX, accelY, accelZ
	                       };

	// Process model is f(x) = x
	const float fx[EKF_N] = { _ekf.x[0], _ekf.x[1], _ekf.x[2], _ekf.x[3], _ekf.x[4], _ekf.x[5],
	                          _ekf.x[6], _ekf.x[7], _ekf.x[8], _ekf.x[9]
	                        };

	// Run the prediction step of the DKF
	ekf_predict(&_ekf, fx, F, Q);

	// Measurement function simplifies the relationship between state
	// and sensor readings for convenience.  A more realistic
	// measurement function would distinguish between state value and
	// measured value; e.g.:
	//   hx[0] = pow(this->x[0], 1.03);
	//   hx[1] = 1.005 * this->x[1];
	//   hx[2] = .9987 * this->x[1] + .001;
	const float hx[EKF_M] = {_ekf.x[0], _ekf.x[1], _ekf.x[2], _ekf.x[3], _ekf.x[4], _ekf.x[5],
	                         _ekf.x[6], _ekf.x[7], _ekf.x[8], _ekf.x[9]
	                        };

	// Run the update step
	ekf_update(&_ekf, z, hx, H, R);

	// Report measured and predicte/fused values

	filtered[0] = _ekf.x[0];
	filtered[1] = _ekf.x[1];
	filtered[2] = _ekf.x[2];
	filtered[3] = _ekf.x[3];
	filtered[4] = _ekf.x[4];
	filtered[5] = _ekf.x[5];
	filtered[6] = _ekf.x[6];
	filtered[7] = _ekf.x[7];
	filtered[8] = _ekf.x[8];
	filtered[9] = _ekf.x[9];
}