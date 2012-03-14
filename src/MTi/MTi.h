/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita
* Notes: Original code in Cocoa from 07/08/2009, went C++ on 10/11/2010
*********************************************************************/
#include <cereal_port/CerealPort.h>
#include "MTMessage.h"

#include <string>
#include <vector>

#define PRE 0xFA
#define BID 0xFF

namespace Xsens
{
	class MTi
	{
		public:
		MTi();
		~MTi();
		
		typedef struct _outputMode
		{
			bool temperatureData;
			bool calibratedData;
			bool orientationData;
			bool auxiliaryData;
			bool positionData;
			bool velocityData;
			bool statusData;
			bool rawGPSData;
			bool rawInertialData;
			
		} outputMode;
		
		typedef struct _outputSettings
		{
			bool timeStamp;
			MTOrientationMode orientationMode;
			
		} outputSettings;
		
		bool setOutputModeAndSettings(outputMode mode, outputSettings settings, int timeout);

		bool openPort(char * name, int baudrate);
		bool closePort();

		void makeMessage(MTMessageIdentifier mid, std::vector<unsigned char> * data, std::vector<unsigned char> * message);
		void addMessageToQueue(MTMessageIdentifier messageID, std::vector<unsigned char> * data, MTMessageIdentifier ack);
		bool waitForQueueToFinish(int timeout);

		//void resetOrientation();
		
		float accelerometer_x() { return accX; }
		float accelerometer_y() { return accY; }
		float accelerometer_z() { return accZ; }
		float gyroscope_x() { return gyrX; }
		float gyroscope_y() { return gyrY; }
		float gyroscope_z() { return gyrZ; }
		float compass_x() { return magX; }
		float compass_y() { return magY; }
		float compass_z() { return magZ; }
		float temperature() { return temp; }
		float quaternion_x() {return q1; }
		float quaternion_y() { return q2; }
		float quaternion_z() { return q3; }
		float quaternion_w() { return q0; }
		float roll() { return eroll; }
		float pitch() { return epitch; }
		float yaw() { return eyaw; }
	
		private:
		// Serial Port variables
		cereal::CerealPort serial_port;
	
		// OutputMode
		std::vector<unsigned char> outputModeData;
		outputMode output_mode;
		
		// OutputSettings
		std::vector<unsigned char> outputSettingsData;
		outputSettings output_settings;
	
		// To manage incoming packages
		std::vector<unsigned char> package;
		int packageLength;
		bool packageInTransit;
		bool packageIsExtended;
		int packageIndex;
	
		bool ConfigState;
	
		float yawCompensation;
	
		int numOfBytes;
		
		// ******* MTi Data *******
		float accX, accY, accZ;
		float gyrX, gyrY, gyrZ;
		float magX, magY, magZ;
		float temp;
		float q0, q1, q2, q3;
		float eroll, epitch, eyaw;
		unsigned int ts;
		// ************************
	
		// Message queue
		std::vector<MTMessage> queue;
		MTMessageIdentifier queueAck;
		bool queueIsRunning;
		bool queueIsWaiting;
		
		void resetPackage();
		void manageQueue();
		
		bool serialPortSendData(std::vector<unsigned char> * data);
		void serialPortReadData(char * data, int length);
		void manageIncomingData(std::vector<unsigned char> * data, bool dataIsExtended);
	};
}

// EOF

