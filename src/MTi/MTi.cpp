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
#include "MTi.h"
#include <stdio.h>
#include <stdlib.h>

// Small workaround for the hexa to float problem...
float hexa2float(unsigned char * buffer)
{
	union
	{
		float value;
		unsigned char buffer[4];
		
	}floatUnion;
	
	floatUnion.buffer[0] = buffer[3];
	floatUnion.buffer[1] = buffer[2];
	floatUnion.buffer[2] = buffer[1];
	floatUnion.buffer[3] = buffer[0];
	
	return floatUnion.value;
}

Xsens::MTi::MTi() : serial_port()
{
	numOfBytes = 4;
	
	// Serial Port Settings
	this->resetPackage();
	
	// Queue Settings
	queueIsWaiting = false;
	queueIsRunning = false;
	
	accX = accY = accZ = 0.0;
	gyrX = gyrY = gyrZ = 0.0;
	magX = magY = magZ = 0.0;
	q0 = q1 = q2 = q3 = 0.0;
	eroll = epitch = eyaw = 0.0;
	temp = 0.0;
	ts = 0;
}

Xsens::MTi::~MTi()
{
	if(serial_port.portOpen()) this->closePort();
}

bool Xsens::MTi::setOutputModeAndSettings(outputMode mode, outputSettings settings, int timeout)
{
	this->outputModeData.clear();
	this->outputSettingsData.clear();
	
	char byte;
	char temperatureData = mode.temperatureData;
	char calibratedData = mode.calibratedData;
	char orientationData = mode.orientationData;	
	char auxiliaryData = mode.auxiliaryData;
	char positionData = mode.positionData;
	char velocityData = mode.velocityData;
	char statusData = mode.statusData;
	char rawGPSData = mode.rawGPSData;
	char rawInertialData = mode.rawInertialData;
	
	byte = 0;
	byte = statusData<<3 | rawGPSData<<4 | rawInertialData<<6;  
	outputModeData.push_back(byte);
	byte = 0;
	byte = temperatureData | calibratedData<<1 | orientationData<<2 |  auxiliaryData<<3 | positionData<<4 | velocityData<<5;
	outputModeData.push_back(byte);
	
	outputSettingsData.push_back(0x00);
	outputSettingsData.push_back(0x00);
	outputSettingsData.push_back(0x00);
	byte = 0;
	byte = settings.timeStamp | settings.orientationMode<<2;
	outputSettingsData.push_back(byte);
	
	// Go into config mode, set our preferences and go back to measuring
	this->addMessageToQueue(GoToConfig, NULL, GoToConfigAck);
	this->addMessageToQueue(SetOutputMode, &outputModeData, SetOutputModeAck);
	this->addMessageToQueue(SetOutputSettings, &outputSettingsData, SetOutputSettingsAck);
	this->addMessageToQueue(ReqOutputMode, NULL, ReqOutputModeAck);
	this->addMessageToQueue(ReqOutputSettings, NULL, ReqOutputSettingsAck);
	this->addMessageToQueue(GoToMeasurement, NULL, GoToMeasurementAck);
	
	return this->waitForQueueToFinish(timeout);
}

void Xsens::MTi::resetPackage()
{
	packageInTransit = false;
	packageLength = 0;
	packageIndex = 0;
	package.clear();
}

void Xsens::MTi::addMessageToQueue(MTMessageIdentifier messageID, std::vector<unsigned char> * data, MTMessageIdentifier ack)
{
	this->queue.push_back(MTMessage(messageID, data, ack));
	if(queueIsRunning == false) this->manageQueue();
}

void Xsens::MTi::manageQueue()
{
	queueIsRunning = true;
	if(queueIsWaiting == true)
	{
		this->queue.erase(queue.begin());
		queueIsWaiting = false;
		
		if(this->queue.size() == 0) queueIsRunning = false;
	}
	if(queueIsWaiting == false && queueIsRunning == true)
	{
		MTMessage message2send = this->queue[0];
		std::vector<unsigned char> data2send;
		this->makeMessage(message2send.getMessageID(), message2send.getData(), &data2send);
		queueAck = message2send.getMessageAck();
		queueIsWaiting = true;
		this->serialPortSendData(&data2send);
	}
}

bool Xsens::MTi::waitForQueueToFinish(int timeout)
{
	for(int i=0 ; i<timeout ; i++)
	{
		usleep(1000);	// sleep for 1ms
		if(queueIsRunning == false) return true;
	}
	
	queue.clear();
	queueIsWaiting = false;
	queueIsRunning = false;
	
	this->resetPackage();
	
	return false;
}

bool Xsens::MTi::openPort(char * name, int baudrate)
{
	try{ serial_port.open(name, baudrate); }
	catch(cereal::Exception& e)
	{
		return false;
	}
	return serial_port.startReadStream(boost::bind(&Xsens::MTi::serialPortReadData, this, _1, _2));
}

bool Xsens::MTi::closePort()
{
	try{ serial_port.close(); }
	catch(cereal::Exception& e)
	{ 
		return false;
	}
	return true;
}

bool Xsens::MTi::serialPortSendData(std::vector<unsigned char> * data)
{
	/*printf("Sending data -");
	for(int i=0 ; i<data->size() ; i++) printf(" 0x%X", data->at(i));
	printf("\n");*/
	
	char buffer[data->size()];
	
	int i;
	std::vector<unsigned char>::iterator it;
	for(i=0, it=data->begin() ; it!=data->end() ; i++, it++) buffer[i] = (char)*it;
	
	try{ serial_port.write(buffer, data->size()); }
	catch(cereal::Exception& e)
	{
		return false;
	}
	return true;
}

void Xsens::MTi::serialPortReadData(char * data, int length)
{	
	if(length > 0)
	{
		// Manage the received data...
		unsigned char buffer;
		for(int i=0 ; i<length ; i++)
		{
			buffer = (unsigned char)data[i];
			
			// PREAMBLE
			if(packageInTransit == false)
			{
				if(buffer == PRE)
				{
					this->package.clear();
					this->package.push_back(buffer);
					packageInTransit = true;
					packageIndex = 1;
				}
				
			} else {
				
				// CHECKSUM
				if( (packageIsExtended == true && packageIndex == 6+packageLength) || (packageIsExtended == false && packageIndex == 4+packageLength) )
				{
					package.push_back(buffer);

					unsigned char checksum = 0;
					for(unsigned int i=1 ; i<this->package.size() ; i++)
					{
						buffer = this->package[i];
						checksum += buffer;
					}
					// If message is ok manage it else reset the package
					if(checksum == 0x00) this->manageIncomingData(&this->package, packageIsExtended);
					else this->resetPackage();
				}
				// DATA
				if((packageIndex >= 6 && packageIndex < 6+packageLength) || (packageIsExtended == false && packageIndex >= 4 && packageIndex < 4+packageLength) )
				{
					this->package.push_back(buffer);
					packageIndex++;
				}
				// EXT_LEN
				if(packageIsExtended == true && packageIndex == 4)
				{
					this->package.push_back(buffer);
					packageIndex = 5;
				}
				if(packageIsExtended == true && packageIndex == 5)
				{
					this->package.push_back(buffer);
					packageIndex = 6;
					
					union
					{
						unsigned int value;
						unsigned char buffer[2];
					
					} intUnion;
					intUnion.buffer[0] = this->package[4];
					intUnion.buffer[1] = this->package[5];
					packageLength = intUnion.value;
				}
				// LEN
				if(packageIndex == 3)
				{
					this->package.push_back(buffer);
					packageIndex = 4;
					
					if(buffer == 0xFF) packageIsExtended = true;
					else
					{	
						packageIsExtended = false;
						packageLength = (int)buffer;
					}
				}
				// MID
				if(packageIndex == 2)
				{
					this->package.push_back(buffer);
					packageIndex = 3;
				}
				// BID
				if(buffer == BID && packageIndex == 1)
				{
					this->package.push_back(buffer);
					packageIndex = 2;
				}
				if(packageIndex == 1 && buffer != BID)
				{
					this->resetPackage();
				}				
			}
		}
	}
}

void Xsens::MTi::manageIncomingData(std::vector<unsigned char> * incomingData, bool dataIsExtended)
{	
	/*printf("Getting data -");
	for(int i=0 ; i<incomingData->size() ; i++) printf(" 0x%X", incomingData->at(i));
	printf("\n");*/

	int dataIndex = 4;
	if(dataIsExtended) dataIndex = 6;
	
	// And now finnaly actualy manage the data
	std::vector<unsigned char> data;
	
	std::vector<unsigned char>::iterator it;
	for(it=incomingData->begin()+dataIndex ; it!=incomingData->end() ; it++)
	{
		data.push_back((unsigned char)*it);
	}
	
	unsigned char MID;
	MID = incomingData->at(2);
	
	this->resetPackage();
	
	if(queueIsWaiting == true && MID == queueAck) this->manageQueue();
	
	// Variables useful to manage the data
	unsigned char floatBuffer[numOfBytes];
	unsigned char buffer;
	unsigned char mask;
	int index;
	
	// Switch case for managing the various MIDs that might arrive
	switch(MID)
	{
		case GoToConfigAck:
			ConfigState = true;
			break;
			
		case GoToMeasurementAck:
			ConfigState = false;
			break;
			
		case ReqOutputModeAck:
			if(data.size()>0)
			{
				buffer = data[1];
				mask = 0x01;
				if((buffer & mask) == mask) output_mode.temperatureData = true;
				else output_mode.temperatureData = false;
				mask = mask << 1;
				if((buffer & mask) == mask) output_mode.calibratedData = true;
				else output_mode.calibratedData = false;
				mask = mask << 1;
				if((buffer & mask) == mask) output_mode.orientationData = true;
				else output_mode.orientationData = false;
				mask = mask << 1;
				if((buffer & mask) == mask) output_mode.auxiliaryData = true;
				else output_mode.auxiliaryData = false;
				mask = mask << 1;
				if((buffer & mask) == mask) output_mode.positionData = true;
				else output_mode.positionData = false;
				mask = mask << 1;
				if((buffer & mask) == mask) output_mode.velocityData = true;
				else output_mode.velocityData = false;
				buffer = data[0];
				mask = 0x04;
				if((buffer & mask) == mask) output_mode.statusData = true;
				else output_mode.statusData = false;
				mask = mask << 1;
				if((buffer & mask) == mask) output_mode.rawGPSData = true;
				else output_mode.rawGPSData = false;
				mask = mask << 2;
				if((buffer & mask) == mask) output_mode.rawInertialData = true;
				else output_mode.rawInertialData = false;
			}
			break;
			
		case ReqOutputSettingsAck:
			if(data.size()>0)
			{
				buffer = data[3];
				mask = 0x01;
				if((buffer & mask) == 0x01) output_settings.timeStamp = true;
				else output_settings.timeStamp = false;
				mask = 0x03;
				if((buffer>>2 & mask) == 0x00) output_settings.orientationMode = Quaternion;
				if((buffer>>2 & mask) == 0x01) output_settings.orientationMode = EulerAngles;
				if((buffer>>2 & mask) == 0x02) output_settings.orientationMode = Matrix;
			}
			break;
			
		// Read incoming data according to mode and settings data
		case MTData:
			index = 0;
			if(output_mode.temperatureData == true)
			{
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				temp = hexa2float(floatBuffer);
				index += numOfBytes;
			}
			if(output_mode.calibratedData == true)
			{
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				accX = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				accY = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				accZ = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				gyrX = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				gyrY = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				gyrZ = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				magX = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				magY = hexa2float(floatBuffer);
				index += numOfBytes;
				for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
				magZ = hexa2float(floatBuffer);
				index += numOfBytes;
			}
			if(output_mode.orientationData == true)
			{
				if(output_settings.orientationMode == Quaternion)
				{
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					q0 = hexa2float(floatBuffer);
					index += numOfBytes;
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					q1 = hexa2float(floatBuffer);
					index += numOfBytes;
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					q2 = hexa2float(floatBuffer);
					index += numOfBytes;
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					q3 = hexa2float(floatBuffer);
					index += numOfBytes;
				}
				if(output_settings.orientationMode == EulerAngles)
				{
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					eroll = hexa2float(floatBuffer);
					index += numOfBytes;
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					epitch = hexa2float(floatBuffer);
					index += numOfBytes;
					for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
					eyaw = hexa2float(floatBuffer);
					index += numOfBytes;
				}
				if(output_settings.orientationMode == Matrix)
				{
					index += 9*numOfBytes;
				}
			}
			if(output_mode.auxiliaryData == true)
			{
				index += 2*numOfBytes;
			}
			if(output_mode.positionData == true)
			{
				index += 3*numOfBytes;
			}
			if(output_mode.velocityData == true)
			{
				index += 3*numOfBytes;
			}
			if(output_mode.statusData == true)
			{
				index += numOfBytes;
			}
			if(output_settings.timeStamp == true)
			{
				index += 2*numOfBytes;
			}
			break;
		
		case ResetOrientationAck:
			// Reset ok.
			break;
	}
}

void Xsens::MTi::makeMessage(MTMessageIdentifier mid, std::vector<unsigned char> * data, std::vector<unsigned char> * message)
{
	int dataLength = 0;
	if(data!=NULL) dataLength = data->size();
	unsigned char byte;
	
	message->clear();
	// PREAMBLE
	message->push_back(PRE);
	// BID
	message->push_back(BID);
	// MID
	byte = (unsigned char)mid;
	message->push_back(byte);
	// LEN / EXT_LEN
	if(dataLength < 0xFF)
	{
		byte = (unsigned char)dataLength;
		message->push_back(byte);
		
	} else {
	
		byte = 0xFF;
		message->push_back(byte);
		
		union
		{
			int length;
			unsigned char buffer[2];
		}lengthUnion;
		
		lengthUnion.length = dataLength;
		message->push_back(lengthUnion.buffer[0]);
		message->push_back(lengthUnion.buffer[1]);
	}
	//  DATA
	if(data!=NULL)
	{
		if(data->size() > 0)
		{
			std::vector<unsigned char>::iterator it;
			for(it=data->begin() ; it!=data->end() ; it++) message->push_back(*it);
		}
	}
	// CHECKSUM
	unsigned char checksum = 0;
	// BID
	checksum += message->at(1);
	// MID
	checksum += message->at(2);
	// LEN
	if(dataLength < 0xFF)
	{
		checksum += message->at(3);
		
	} else {
		
		checksum += message->at(3);
		checksum += message->at(4);
		checksum += message->at(5);
	}
	// DATA
	for(int i=0 ; i<dataLength ; i++)
	{
		int dataIndex = 6;
		if(dataLength < 0xFF) dataIndex = 4;
		checksum += message->at(dataIndex+i);
	}
	int c = 0x100;
	byte = (unsigned char)(c-(int)checksum);
	message->push_back(byte);
}

/*void Xsens::MTi::resetOrientation()
{
	std::vector<unsigned char> buffer;
	
	// Align reset
	buffer.push_back(0x00);
	buffer.push_back(0x04);

	this->addMessageToQueue(ResetOrientation, &buffer, ResetOrientationAck);
}*/

// EOF

