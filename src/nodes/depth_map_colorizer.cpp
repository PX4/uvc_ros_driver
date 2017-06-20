/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * depth_map_colorizer.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: Vilhjalmur
 *
 */

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

int MIN_DEPTH = 8;
int MAX_DEPTH = 255;

class DepthMapPublisher {
  public:
	DepthMapPublisher() {} 
	DepthMapPublisher(ros::NodeHandle &nh, std::string topic_name) 
		: topic_name(topic_name)
	{
		publisher = nh.advertise<sensor_msgs::Image>(topic_name + "_color", 1);
		subscriber = nh.subscribe(topic_name, 
  						 1, 
  						 &DepthMapPublisher::publishWithColor, 
  						 this);
	}

	// Publish a colorized version of msg
	void publishWithColor(const sensor_msgs::ImageConstPtr& msg) 
	{
		// std::cout << topic_name << std::endl;
		cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
		colorizeDepth(disparity->image);
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;
		out_msg.encoding = "rgb8";
		out_msg.image = colored_mat;
	  	publisher.publish(out_msg.toImageMsg());
	}

	// Update colored_mat
	void colorizeDepth(const cv::Mat &gray) {
		if (gray.size() != colored_mat.size()) {
	  		colored_mat.create(gray.size(), CV_8UC3);
		}
		for (int y = 0; y < gray.rows; y++) {
			for (int x = 0; x < gray.cols; x++) {
				colored_mat.at<cv::Point3_<unsigned char> >(y, x) = 
					spectralColor(gray.at<unsigned char>(y, x));
			}
		}
	}

	// Returns a spectral color between blue (hue=0) and red (hue=255)
	cv::Point3_<unsigned char> spectralColor(int hue) {
		if (hue < MIN_DEPTH) {
			// No depth information
			return cv::Point3_<unsigned char>(0,0,0); 
		}

		// Crude approximation of spectral colors
		hue = std::min(hue, MAX_DEPTH);
		unsigned char r = std::max(0, 2*hue  - MAX_DEPTH);
		unsigned char g = MAX_DEPTH - 2 * std::abs(hue - MAX_DEPTH/2);
		unsigned char b = std::max(0, MAX_DEPTH - 2*hue);
		return cv::Point3_<unsigned char>(r,g,b);
	}

	// Fields
	ros::Publisher publisher;
	ros::Subscriber subscriber;
	std::string topic_name;
	cv::Mat colored_mat;
};


// Returns true if topic is a depth map
bool isDepthMapPublisher(std::string topic_name) {
	return topic_name.find('image_depth') != std::string::npos;
}


int main(int argc, char **argv)
{
	// Create node
	ros::init(argc, argv, "depth_map_colorizer");
	ros::NodeHandle nh("~");

	// Get topic Names
	ros::master::V_TopicInfo topic_infos;
	ros::master::getTopics(topic_infos);

	// Filter depth-map topics
	std::vector<std::string> depth_topics;
	for (auto topic : topic_infos) {
		if (isDepthMapPublisher(topic.name)) {
			depth_topics.push_back(topic.name);
		}
	} 

	// Reserve space so the publishers-vector doesn't move elements in memory
	int n = depth_topics.size();
	std::vector<DepthMapPublisher> publishers;
	publishers.reserve(n);

	// Create publishers and matching subscribers
	for (int i=0; i < n; ++i) {
		std::string topic_name = depth_topics[i];
  		publishers.emplace_back(nh, topic_name);
	}

	ros::spin();
	return 0;
}