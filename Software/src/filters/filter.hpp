/* ---------------------------------------------------------------------------------------------------
    derived from Simon D. Levy's multi-variable multi-sensor EKF example
    https://github.com/simondlevy/TinyEKF/blob/master/examples/SensorFusion/SensorFusion.ino

    thank you Simon for making this library and the example code!
--------------------------------------------------------------------------------------------------- */

// These must be defined before including TinyEKF.h
#define EKF_N 12     // number of measurements
#define EKF_M 12     // number of sensors

#include <stdio.h>
#include "tinyekf.h"
#include <string>

using namespace std;

string toString(float i, int precision) { //converts float to string with specified precision
	string j;
	char buffer[10];
	switch(precision) {
	case 0:
		sprintf(buffer, "%.0f", i);
		break;
	case 2:
		sprintf(buffer, "%.2f", i);
		break;
	case 6:
		sprintf(buffer, "%.6f", i);
		break;
	}
	for (unsigned int k=0; k<sizeof(buffer); k++) {
		j += buffer[k];
	}
	return j;
}

static const float EPS = 1e-4;
string packetFiltered;

float tempTrust = 1;
float pressureTrust = 1;
float altitudeTrust = 1;
float velocityTrust = 1;
float longitudeTrust = 1;
float latitudeTrust = 1;
float gyroXTrust = 1;
float gyroYTrust = 1;
float gyroZTrust = 1;
float accelXTrust = 1;
float accelYTrust = 1;
float accelZTrust = 1;

static const float Q[EKF_N*EKF_N] = {

	EPS*tempTrust, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, EPS*pressureTrust, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, EPS*altitudeTrust, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, EPS*velocityTrust, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, EPS*longitudeTrust, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, EPS*latitudeTrust, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, EPS*gyroXTrust, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, EPS*gyroYTrust, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, EPS*gyroZTrust, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, EPS*accelXTrust, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EPS*accelYTrust, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EPS*accelZTrust
};

static const float R[EKF_M*EKF_M] = {

	EPS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, EPS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, EPS, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, EPS, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, EPS, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, EPS, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, EPS, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, EPS, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, EPS, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, EPS, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EPS, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EPS
};

// So process model Jacobian is identity matrix
static const float F[EKF_N*EKF_N] = {
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
};

static const float H[EKF_M*EKF_N] = {

	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
};

static ekf_t _ekf;

// this code is from h1, it will have to be changed for this sensor layout

float baroTemperature = 0.0;
float baroPressure = 0.0;
float altitude = 0.0;
float velocity = 0.0;
float longitude = 0.0;
float latitude = 0.0;
float voltage = 0.0;
int satellitesConnected = 0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
int signalStrength = 0;

void decompilePacket(string* packet) { //function for decompiling packets (suggest improvements, this could always be more efficent and error correctable)
	void *endPTR;
	size_t end;
	char delimiters[15] = {'T', 'P', 'A', 'v', 'l', 'L', 'V', 'S', 'x', 'y', 'z', 'X', 'Y', 'Z', 's'}; //characters that split each variable
	for(int i=0; i<15; i++) {

		unsigned int start = packet->find(delimiters[i]);
		unsigned int end;
		
		if (i == 14) { //reached the end of the packet where there is no delimiter
            end = packet->length();
        }
        else {
            end = packet->find(delimiters[i+1]);
        }
        
        string bufferValue = packet->substr(start+1, end);

		//converts the buffer into the correct variable using atof or atoi
		switch (delimiters[i]) {
		case 'T':
			baroTemperature = atof(bufferValue.c_str());
			break;
		case 'P':
			baroPressure = atof(bufferValue.c_str());
			break;
		case 'A':
			altitude = atof(bufferValue.c_str());
			break;
		case 'v':
			velocity = atof(bufferValue.c_str());
			break;
		case 'l':
			longitude = atof(bufferValue.c_str());
			break;
		case 'L':
			latitude = atof(bufferValue.c_str());
			break;
		case 'V':
			voltage = atof(bufferValue.c_str());
			break;
		case 'S':
			satellitesConnected = atoi(bufferValue.c_str());
			break;
		case 'x':
			gyroX = atof(bufferValue.c_str());
			break;
		case 'y':
			gyroY = atof(bufferValue.c_str());
			break;
		case 'z':
			gyroZ = atof(bufferValue.c_str());
			break;
		case 'X':
			accelX = atof(bufferValue.c_str());
			break;
		case 'Y':
			accelY = atof(bufferValue.c_str());
			break;
		case 'Z':
			accelZ = atof(bufferValue.c_str());
			break;
		case 's':
			signalStrength = atoi(bufferValue.c_str());
			break;
		}
	}
}

void compileFilterPacket() {
	// FORMAT:
	packetFiltered = "@T" + toString(_ekf.x[0], 2) + "P" + toString(_ekf.x[1], 2) + "A" + toString(_ekf.x[2], 2)
	                 + "v" + toString(_ekf.x[3], 2)+ "l" + toString(_ekf.x[4], 6) + "L" + toString(_ekf.x[5], 6)
	                 + "V" + toString(voltage, 0) + "S" + toString(satellitesConnected, 0) + "x" + toString(_ekf.x[6], 2)
	                 + "y" + toString(_ekf.x[7], 2) + "z" + toString(_ekf.x[8], 2) + "X" + toString(_ekf.x[9], 2)
	                 + "Y" + toString(_ekf.x[10], 2) + "Z" + toString(_ekf.x[11], 2) + "s" + toString(signalStrength, 0);
}

void filterSetup() {
	// Use identity matrix as initiali covariance matrix
	const float Pdiag[EKF_N] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

	ekf_initialize(&_ekf, Pdiag);
}

void filterLoop() {
	// Set the observation vector z
	const float z[EKF_M] = {baroTemperature, baroPressure, altitude, velocity, longitude, latitude,
	                        gyroX, gyroY, gyroZ, accelX, accelY, accelZ
	                       };

	// Process model is f(x) = x
	const float fx[EKF_N] = { _ekf.x[0], _ekf.x[1], _ekf.x[2], _ekf.x[3], _ekf.x[4], _ekf.x[5],
	                          _ekf.x[6], _ekf.x[7], _ekf.x[8], _ekf.x[9], _ekf.x[10], _ekf.x[11]
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
	                         _ekf.x[6], _ekf.x[7], _ekf.x[8], _ekf.x[9], _ekf.x[10], _ekf.x[11]
	                        };

	// Run the update step
	ekf_update(&_ekf, z, hx, H, R);

	// Report measured and predicte/fused values

	compileFilterPacket();
}