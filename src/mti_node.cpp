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
* Author: Gonçalo Cabrita on 08/11/2010
*********************************************************************/
#include "MTi/MTi.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mti_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	// Params
	std::string portname;
	int baudrate;
	std::string frame_id;
	pn.param<std::string>("port", portname, "/dev/ttyUSB0");
	pn.param("baudrate", baudrate, 115200);
	pn.param<std::string>("frame_id", frame_id, "/base_imu");
	
	Xsens::MTi * mti = new Xsens::MTi();
	
	if(!mti->openPort((char*)portname.c_str(), baudrate))
	{
		ROS_FATAL("MTi -- Unable to connect to the MTi.");
		ROS_BREAK();
	}
	ROS_INFO("MTi -- Successfully connected to the MTi!");
	
	Xsens::MTi::outputMode outputMode;
	outputMode.temperatureData = false;
	outputMode.calibratedData = true;
	outputMode.orientationData = true;
	outputMode.auxiliaryData = false;
	outputMode.positionData = false;
	outputMode.velocityData = false;
	outputMode.statusData = false;
	outputMode.rawGPSData = false;
	outputMode.rawInertialData = false;
	
	Xsens::MTi::outputSettings outputSettings;
	outputSettings.timeStamp = false;
	outputSettings.orientationMode = Xsens::Quaternion;
	
	if(!mti->setOutputModeAndSettings(outputMode, outputSettings, 1000))
	{
		ROS_FATAL("MTi -- Unable to set the output mode and settings.");
		ROS_BREAK();
	}
	ROS_INFO("MTi -- Setup complete! Initiating data streaming...");

	ros::Publisher mti_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);

	ros::Rate r(20);
  	while(ros::ok())
	{	
		sensor_msgs::Imu mti_msg;
		mti_msg.header.stamp = ros::Time::now();
		mti_msg.header.frame_id = frame_id.c_str();
		
		mti_msg.orientation.x = mti->quaternion_x();
		mti_msg.orientation.y = mti->quaternion_y();
		mti_msg.orientation.z = mti->quaternion_z();
		mti_msg.orientation.w = mti->quaternion_w();
		
		mti_msg.angular_velocity.x = mti->gyroscope_x();
		mti_msg.angular_velocity.y = mti->gyroscope_y();
		mti_msg.angular_velocity.z = mti->gyroscope_z();
		
		mti_msg.linear_acceleration.x = mti->accelerometer_x();
		mti_msg.linear_acceleration.y = mti->accelerometer_y();
		mti_msg.linear_acceleration.z = mti->accelerometer_z();
		
		mti_pub.publish(mti_msg);
		
		r.sleep();
	}
	
  	return(0);
}

// EOF

