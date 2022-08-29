/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * pixel_cloud_fusion.cpp
 *
 *  Created on: May, 19th, 2018
 */


#include "pixel_cloud_fusion/pixel_cloud_fusion.h"
#include "iostream"
using namespace std;

velodyne_pointcloud::PointXYZIR ROSPixelCloudFusionApp::TransformPoint(const velodyne_pointcloud::PointXYZIR &in_point, const tf::StampedTransform &in_transform)
{
	tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
	tf::Vector3 tf_point_t = in_transform * tf_point;
	velodyne_pointcloud::PointXYZIR xyzir;
	xyzir.x=tf_point_t.x();
	xyzir.y=tf_point_t.y();
	xyzir.z=tf_point_t.z();
	xyzir.intensity=in_point.intensity;
	xyzir.ring=in_point.ring;
	return xyzir;
}

pcl::PointXYZ ROSPixelCloudFusionApp::TransformPoint_rgb(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
	tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
	tf::Vector3 tf_point_t = in_transform * tf_point;
	return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

void ROSPixelCloudFusionApp::ImageCallback_1(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!image_1.camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::undistort(in_image, image_1.current_frame_, image_1.camera_instrinsics_, image_1.distortion_coefficients_);
	
	image_1.image_frame_id_ = in_image_msg->header.frame_id;
	image_1.image_size_.height = image_1.current_frame_.rows;
	image_1.image_size_.width = image_1.current_frame_.cols;
}
void ROSPixelCloudFusionApp::ImageCallback_2(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!image_2.camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::undistort(in_image, image_2.current_frame_, image_2.camera_instrinsics_, image_2.distortion_coefficients_);
	
	image_2.image_frame_id_ = in_image_msg->header.frame_id;
	image_2.image_size_.height = image_2.current_frame_.rows;
	image_2.image_size_.width = image_2.current_frame_.cols;
}
void ROSPixelCloudFusionApp::ImageCallback_3(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!image_3.camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::undistort(in_image, image_3.current_frame_, image_3.camera_instrinsics_, image_3.distortion_coefficients_);
	
	image_3.image_frame_id_ = in_image_msg->header.frame_id;
	image_3.image_size_.height = image_3.current_frame_.rows;
	image_3.image_size_.width = image_3.current_frame_.cols;
}
void ROSPixelCloudFusionApp::ImageCallback_4(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!image_4.camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::undistort(in_image, image_4.current_frame_, image_4.camera_instrinsics_, image_4.distortion_coefficients_);
	
	image_4.image_frame_id_ = in_image_msg->header.frame_id;
	image_4.image_size_.height = image_4.current_frame_.rows;
	image_4.image_size_.width = image_4.current_frame_.cols;
}
void ROSPixelCloudFusionApp::ImageCallback_5(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!image_5.camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::undistort(in_image, image_5.current_frame_, image_5.camera_instrinsics_, image_5.distortion_coefficients_);
	
	image_5.image_frame_id_ = in_image_msg->header.frame_id;
	image_5.image_size_.height = image_5.current_frame_.rows;
	image_5.image_size_.width = image_5.current_frame_.cols;
}
void ROSPixelCloudFusionApp::ImageCallback_6(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!image_6.camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::undistort(in_image, image_6.current_frame_, image_6.camera_instrinsics_, image_6.distortion_coefficients_);
	
	image_6.image_frame_id_ = in_image_msg->header.frame_id;
	image_6.image_size_.height = image_6.current_frame_.rows;
	image_6.image_size_.width = image_6.current_frame_.cols;
}

void ROSPixelCloudFusionApp::CloudCallback_intensity(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr in_cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
	pcl::fromROSMsg(*in_cloud_msg, *in_cloud);
	std::unordered_map<int, cv::Point> projection_map;
	std::unordered_map<cv::Point, int> image_number_map;

std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
	////////////////////image_1////////////////////
	if (image_1.current_frame_.empty() || image_1.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_1.camera_lidar_tf_ok_)
	{
		image_1.camera_lidar_tf_ok_ = false;
		image_1.transform_listener_->lookupTransform(image_1.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_1.camera_lidar_tf_);
		image_1.camera_lidar_tf_ok_ = true;
	}
	if (!image_1.camera_info_ok_ || !image_1.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<velodyne_pointcloud::PointXYZIR> cam_cloud_1(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_1[i] = TransformPoint(in_cloud->points[i], image_1.camera_lidar_tf_);
		int u = int(cam_cloud_1[i].x * image_1.fx_ / cam_cloud_1[i].z + image_1.cx_);
		int v = int(cam_cloud_1[i].y * image_1.fy_ / cam_cloud_1[i].z + image_1.cy_);
		if ((u >= 0) && (u < image_1.image_size_.width)
			&& (v >= 0) && (v < image_1.image_size_.height)
			&& cam_cloud_1[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 1));
		}
	}
	////////////////////image_1////////////////////
	////////////////////image_2////////////////////
	if (image_2.current_frame_.empty() || image_2.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_2.camera_lidar_tf_ok_)
	{
		image_2.camera_lidar_tf_ok_ = false;
		image_2.transform_listener_->lookupTransform(image_2.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_2.camera_lidar_tf_);
		image_2.camera_lidar_tf_ok_ = true;
	}
	if (!image_2.camera_info_ok_ || !image_2.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<velodyne_pointcloud::PointXYZIR> cam_cloud_2(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_2[i] = TransformPoint(in_cloud->points[i], image_2.camera_lidar_tf_);
		int u = int(cam_cloud_2[i].x * image_2.fx_ / cam_cloud_2[i].z + image_2.cx_);
		int v = int(cam_cloud_2[i].y * image_2.fy_ / cam_cloud_2[i].z + image_2.cy_);
		if ((u >= 0) && (u < image_2.image_size_.width)
			&& (v >= 0) && (v < image_2.image_size_.height)
			&& cam_cloud_2[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 2));
		}
	}
	////////////////////image_2////////////////////
	////////////////////image_3////////////////////
	if (image_3.current_frame_.empty() || image_3.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_3.camera_lidar_tf_ok_)
	{
		image_3.camera_lidar_tf_ok_ = false;
		image_3.transform_listener_->lookupTransform(image_3.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_3.camera_lidar_tf_);
		image_3.camera_lidar_tf_ok_ = true;
	}
	if (!image_3.camera_info_ok_ || !image_3.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<velodyne_pointcloud::PointXYZIR> cam_cloud_3(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_3[i] = TransformPoint(in_cloud->points[i], image_3.camera_lidar_tf_);
		int u = int(cam_cloud_3[i].x * image_3.fx_ / cam_cloud_3[i].z + image_3.cx_);
		int v = int(cam_cloud_3[i].y * image_3.fy_ / cam_cloud_3[i].z + image_3.cy_);
		if ((u >= 0) && (u < image_3.image_size_.width)
			&& (v >= 0) && (v < image_3.image_size_.height)
			&& cam_cloud_3[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 3));
		}
	}
	////////////////////image_3////////////////////
	////////////////////image_4////////////////////
	if (image_4.current_frame_.empty() || image_4.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_4.camera_lidar_tf_ok_)
	{
		image_4.camera_lidar_tf_ok_ = false;
		image_4.transform_listener_->lookupTransform(image_4.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_4.camera_lidar_tf_);
		image_4.camera_lidar_tf_ok_ = true;
	}
	if (!image_4.camera_info_ok_ || !image_4.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<velodyne_pointcloud::PointXYZIR> cam_cloud_4(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_4[i] = TransformPoint(in_cloud->points[i], image_4.camera_lidar_tf_);
		int u = int(cam_cloud_4[i].x * image_4.fx_ / cam_cloud_4[i].z + image_4.cx_);
		int v = int(cam_cloud_4[i].y * image_4.fy_ / cam_cloud_4[i].z + image_4.cy_);
		if ((u >= 0) && (u < image_4.image_size_.width)
			&& (v >= 0) && (v < image_4.image_size_.height)
			&& cam_cloud_4[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 4));
		}
	}
	////////////////////image_4////////////////////
	////////////////////image_5////////////////////
	if (image_5.current_frame_.empty() || image_5.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_5.camera_lidar_tf_ok_)
	{
		image_5.camera_lidar_tf_ok_ = false;
		image_5.transform_listener_->lookupTransform(image_5.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_5.camera_lidar_tf_);
		image_5.camera_lidar_tf_ok_ = true;
	}
	if (!image_5.camera_info_ok_ || !image_5.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<velodyne_pointcloud::PointXYZIR> cam_cloud_5(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_5[i] = TransformPoint(in_cloud->points[i], image_5.camera_lidar_tf_);
		int u = int(cam_cloud_5[i].x * image_5.fx_ / cam_cloud_5[i].z + image_5.cx_);
		int v = int(cam_cloud_5[i].y * image_5.fy_ / cam_cloud_5[i].z + image_5.cy_);
		if ((u >= 0) && (u < image_5.image_size_.width)
			&& (v >= 0) && (v < image_5.image_size_.height)
			&& cam_cloud_5[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 5));
		}
	}
	////////////////////image_5////////////////////
	////////////////////image_6////////////////////
	if (image_6.current_frame_.empty() || image_6.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_6.camera_lidar_tf_ok_)
	{
		image_6.camera_lidar_tf_ok_ = false;
		image_6.transform_listener_->lookupTransform(image_6.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_6.camera_lidar_tf_);
		image_6.camera_lidar_tf_ok_ = true;
	}
	if (!image_6.camera_info_ok_ || !image_6.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<velodyne_pointcloud::PointXYZIR> cam_cloud_6(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_6[i] = TransformPoint(in_cloud->points[i], image_6.camera_lidar_tf_);
		int u = int(cam_cloud_6[i].x * image_6.fx_ / cam_cloud_6[i].z + image_6.cx_);
		int v = int(cam_cloud_6[i].y * image_6.fy_ / cam_cloud_6[i].z + image_6.cy_);
		if ((u >= 0) && (u < image_6.image_size_.width)
			&& (v >= 0) && (v < image_6.image_size_.height)
			&& cam_cloud_6[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 6));
		}
	}
	////////////////////image_6////////////////////
	
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr out_cloud_intensity(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		std::unordered_map<int, cv::Point>::const_iterator iterator_3d_2d;
		std::unordered_map<cv::Point, int>::const_iterator iterator_image_number;
		velodyne_pointcloud::PointXYZIR corresponding_3d_point;
        velodyne_pointcloud::PointXYZIR colored_3d_point_intensity;
		iterator_3d_2d = projection_map.find(i);
		if (iterator_3d_2d != projection_map.end())
		{
			corresponding_3d_point = in_cloud->points[iterator_3d_2d->first];
			iterator_image_number = image_number_map.find(iterator_3d_2d->second);
			cv::Vec3b rgb_pixel;
			switch(iterator_image_number->second) 
			{
				case 1:
					rgb_pixel = image_1.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
			    	case 2:
					rgb_pixel = image_2.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 3:
					rgb_pixel = image_3.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 4:
					rgb_pixel = image_4.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 5:
					rgb_pixel = image_5.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 6:
					rgb_pixel = image_6.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;

			}
			colored_3d_point_intensity.x = corresponding_3d_point.x;
			colored_3d_point_intensity.y = corresponding_3d_point.y;
			colored_3d_point_intensity.z = corresponding_3d_point.z;
			std::uint32_t rgb = (int)rgb_pixel[2] << 16 | (int)rgb_pixel[1] << 8 | (int)rgb_pixel[0];
		    colored_3d_point_intensity.intensity = *reinterpret_cast<int*>(&rgb);
			colored_3d_point_intensity.ring = corresponding_3d_point.ring;
		    out_cloud_intensity->points.push_back(colored_3d_point_intensity);

		}
		else
		{
			colored_3d_point_intensity.x = std::numeric_limits<float>::quiet_NaN();
			colored_3d_point_intensity.y = std::numeric_limits<float>::quiet_NaN();
			colored_3d_point_intensity.z = std::numeric_limits<float>::quiet_NaN();
		    //colored_3d_point_intensity.intensity = 0;
			//colored_3d_point_intensity.ring = corresponding_3d_point.ring;
		    out_cloud_intensity->points.push_back(colored_3d_point_intensity);
		}
	}

std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
//cout << "-----time(ms):" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

	// Publish PC
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*out_cloud_intensity, cloud_msg);
	cloud_msg.header = in_cloud_msg->header;
	publisher_fused_cloud.publish(cloud_msg);
}

void ROSPixelCloudFusionApp::CloudCallback_rgb(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*in_cloud_msg, *in_cloud);
	std::unordered_map<int, cv::Point> projection_map;
	std::unordered_map<cv::Point, int> image_number_map;

	////////////////////image_1////////////////////
	if (image_1.current_frame_.empty() || image_1.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_1.camera_lidar_tf_ok_)
	{
		image_1.camera_lidar_tf_ok_ = false;
		image_1.transform_listener_->lookupTransform(image_1.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_1.camera_lidar_tf_);
		image_1.camera_lidar_tf_ok_ = true;
	}
	if (!image_1.camera_info_ok_ || !image_1.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<pcl::PointXYZ> cam_cloud_1(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_1[i] = TransformPoint_rgb(in_cloud->points[i], image_1.camera_lidar_tf_);
		int u = int(cam_cloud_1[i].x * image_1.fx_ / cam_cloud_1[i].z + image_1.cx_);
		int v = int(cam_cloud_1[i].y * image_1.fy_ / cam_cloud_1[i].z + image_1.cy_);
		if ((u >= 0) && (u < image_1.image_size_.width)
			&& (v >= 0) && (v < image_1.image_size_.height)
			&& cam_cloud_1[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 1));
		}
	}
	////////////////////image_1////////////////////
	////////////////////image_2////////////////////
	if (image_2.current_frame_.empty() || image_2.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_2.camera_lidar_tf_ok_)
	{
		image_2.camera_lidar_tf_ok_ = false;
		image_2.transform_listener_->lookupTransform(image_2.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_2.camera_lidar_tf_);
		image_2.camera_lidar_tf_ok_ = true;
	}
	if (!image_2.camera_info_ok_ || !image_2.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<pcl::PointXYZ> cam_cloud_2(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_2[i] = TransformPoint_rgb(in_cloud->points[i], image_2.camera_lidar_tf_);
		int u = int(cam_cloud_2[i].x * image_2.fx_ / cam_cloud_2[i].z + image_2.cx_);
		int v = int(cam_cloud_2[i].y * image_2.fy_ / cam_cloud_2[i].z + image_2.cy_);
		if ((u >= 0) && (u < image_2.image_size_.width)
			&& (v >= 0) && (v < image_2.image_size_.height)
			&& cam_cloud_2[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 2));
		}
	}
	////////////////////image_2////////////////////
	////////////////////image_3////////////////////
	if (image_3.current_frame_.empty() || image_3.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_3.camera_lidar_tf_ok_)
	{
		image_3.camera_lidar_tf_ok_ = false;
		image_3.transform_listener_->lookupTransform(image_3.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_3.camera_lidar_tf_);
		image_3.camera_lidar_tf_ok_ = true;
	}
	if (!image_3.camera_info_ok_ || !image_3.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<pcl::PointXYZ> cam_cloud_3(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_3[i] = TransformPoint_rgb(in_cloud->points[i], image_3.camera_lidar_tf_);
		int u = int(cam_cloud_3[i].x * image_3.fx_ / cam_cloud_3[i].z + image_3.cx_);
		int v = int(cam_cloud_3[i].y * image_3.fy_ / cam_cloud_3[i].z + image_3.cy_);
		if ((u >= 0) && (u < image_3.image_size_.width)
			&& (v >= 0) && (v < image_3.image_size_.height)
			&& cam_cloud_3[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 3));
		}
	}
	////////////////////image_3////////////////////
	////////////////////image_4////////////////////
	if (image_4.current_frame_.empty() || image_4.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_4.camera_lidar_tf_ok_)
	{
		image_4.camera_lidar_tf_ok_ = false;
		image_4.transform_listener_->lookupTransform(image_4.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_4.camera_lidar_tf_);
		image_4.camera_lidar_tf_ok_ = true;
	}
	if (!image_4.camera_info_ok_ || !image_4.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<pcl::PointXYZ> cam_cloud_4(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_4[i] = TransformPoint_rgb(in_cloud->points[i], image_4.camera_lidar_tf_);
		int u = int(cam_cloud_4[i].x * image_4.fx_ / cam_cloud_4[i].z + image_4.cx_);
		int v = int(cam_cloud_4[i].y * image_4.fy_ / cam_cloud_4[i].z + image_4.cy_);
		if ((u >= 0) && (u < image_4.image_size_.width)
			&& (v >= 0) && (v < image_4.image_size_.height)
			&& cam_cloud_4[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 4));
		}
	}
	////////////////////image_4////////////////////
	////////////////////image_5////////////////////
	if (image_5.current_frame_.empty() || image_5.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_5.camera_lidar_tf_ok_)
	{
		image_5.camera_lidar_tf_ok_ = false;
		image_5.transform_listener_->lookupTransform(image_5.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_5.camera_lidar_tf_);
		image_5.camera_lidar_tf_ok_ = true;
	}
	if (!image_5.camera_info_ok_ || !image_5.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<pcl::PointXYZ> cam_cloud_5(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_5[i] = TransformPoint_rgb(in_cloud->points[i], image_5.camera_lidar_tf_);
		int u = int(cam_cloud_5[i].x * image_5.fx_ / cam_cloud_5[i].z + image_5.cx_);
		int v = int(cam_cloud_5[i].y * image_5.fy_ / cam_cloud_5[i].z + image_5.cy_);
		if ((u >= 0) && (u < image_5.image_size_.width)
			&& (v >= 0) && (v < image_5.image_size_.height)
			&& cam_cloud_5[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 5));
		}
	}
	////////////////////image_5////////////////////
	////////////////////image_6////////////////////
	if (image_6.current_frame_.empty() || image_6.image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!image_6.camera_lidar_tf_ok_)
	{
		image_6.camera_lidar_tf_ok_ = false;
		image_6.transform_listener_->lookupTransform(image_6.image_frame_id_, in_cloud_msg->header.frame_id, ros::Time(0), image_6.camera_lidar_tf_);
		image_6.camera_lidar_tf_ok_ = true;
	}
	if (!image_6.camera_info_ok_ || !image_6.camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	std::vector<pcl::PointXYZ> cam_cloud_6(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud_6[i] = TransformPoint_rgb(in_cloud->points[i], image_6.camera_lidar_tf_);
		int u = int(cam_cloud_6[i].x * image_6.fx_ / cam_cloud_6[i].z + image_6.cx_);
		int v = int(cam_cloud_6[i].y * image_6.fy_ / cam_cloud_6[i].z + image_6.cy_);
		if ((u >= 0) && (u < image_6.image_size_.width)
			&& (v >= 0) && (v < image_6.image_size_.height)
			&& cam_cloud_6[i].z > 0
				)
		{
			projection_map.insert(std::pair<int, cv::Point>(i, cv::Point(u, v)));
			image_number_map.insert(std::pair<cv::Point, int>(cv::Point(u, v), 6));
		}
	}
	////////////////////image_6////////////////////
	

std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		std::unordered_map<int, cv::Point>::const_iterator iterator_3d_2d;
		std::unordered_map<cv::Point, int>::const_iterator iterator_image_number;
		pcl::PointXYZ corresponding_3d_point;
		pcl::PointXYZRGB colored_3d_point_rgb;
		iterator_3d_2d = projection_map.find(i);
		if (iterator_3d_2d != projection_map.end())
		{
			corresponding_3d_point = in_cloud->points[iterator_3d_2d->first];
			iterator_image_number = image_number_map.find(iterator_3d_2d->second);
			cv::Vec3b rgb_pixel;
			switch(iterator_image_number->second) 
			{
				case 1:
					rgb_pixel = image_1.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
			    	case 2:
					rgb_pixel = image_2.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 3:
					rgb_pixel = image_3.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 4:
					rgb_pixel = image_4.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 5:
					rgb_pixel = image_5.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;
				case 6:
					rgb_pixel = image_6.current_frame_.at<cv::Vec3b>(iterator_3d_2d->second);
					break;

			}

			colored_3d_point_rgb.x = corresponding_3d_point.x;
			colored_3d_point_rgb.y = corresponding_3d_point.y;
			colored_3d_point_rgb.z = corresponding_3d_point.z;
			colored_3d_point_rgb.r = rgb_pixel[2];
			colored_3d_point_rgb.g = rgb_pixel[1];
			colored_3d_point_rgb.b = rgb_pixel[0];
		    out_cloud_rgb->points.push_back(colored_3d_point_rgb);
		}
		else
		{
			colored_3d_point_rgb.x = std::numeric_limits<float>::quiet_NaN();
			colored_3d_point_rgb.y = std::numeric_limits<float>::quiet_NaN();
			colored_3d_point_rgb.z = std::numeric_limits<float>::quiet_NaN();
		    out_cloud_rgb->points.push_back(colored_3d_point_rgb);
		}
	}

std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
//cout << "-----time(ms):" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

	// Publish PC
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*out_cloud_rgb, cloud_msg);
	cloud_msg.header = in_cloud_msg->header;
	publisher_fused_cloud.publish(cloud_msg);
}

void ROSPixelCloudFusionApp::IntrinsicsCallback_1(const sensor_msgs::CameraInfo &in_message)
{
	image_1.image_size_.height = in_message.height;
	image_1.image_size_.width = in_message.width;

	image_1.camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			image_1.camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	image_1.distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		image_1.distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	image_1.fx_ = static_cast<float>(in_message.P[0]);
	image_1.fy_ = static_cast<float>(in_message.P[5]);
	image_1.cx_ = static_cast<float>(in_message.P[2]);
	image_1.cy_ = static_cast<float>(in_message.P[6]);

	image_1.intrinsics_subscriber_.shutdown();
	image_1.camera_info_ok_ = true;
}
void ROSPixelCloudFusionApp::IntrinsicsCallback_2(const sensor_msgs::CameraInfo &in_message)
{
	image_2.image_size_.height = in_message.height;
	image_2.image_size_.width = in_message.width;

	image_2.camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			image_2.camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	image_2.distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		image_2.distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	image_2.fx_ = static_cast<float>(in_message.P[0]);
	image_2.fy_ = static_cast<float>(in_message.P[5]);
	image_2.cx_ = static_cast<float>(in_message.P[2]);
	image_2.cy_ = static_cast<float>(in_message.P[6]);

	image_2.intrinsics_subscriber_.shutdown();
	image_2.camera_info_ok_ = true;
}
void ROSPixelCloudFusionApp::IntrinsicsCallback_3(const sensor_msgs::CameraInfo &in_message)
{
	image_3.image_size_.height = in_message.height;
	image_3.image_size_.width = in_message.width;

	image_3.camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			image_3.camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	image_3.distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		image_3.distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	image_3.fx_ = static_cast<float>(in_message.P[0]);
	image_3.fy_ = static_cast<float>(in_message.P[5]);
	image_3.cx_ = static_cast<float>(in_message.P[2]);
	image_3.cy_ = static_cast<float>(in_message.P[6]);

	image_3.intrinsics_subscriber_.shutdown();
	image_3.camera_info_ok_ = true;
}
void ROSPixelCloudFusionApp::IntrinsicsCallback_4(const sensor_msgs::CameraInfo &in_message)
{
	image_4.image_size_.height = in_message.height;
	image_4.image_size_.width = in_message.width;

	image_4.camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			image_4.camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	image_4.distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		image_4.distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	image_4.fx_ = static_cast<float>(in_message.P[0]);
	image_4.fy_ = static_cast<float>(in_message.P[5]);
	image_4.cx_ = static_cast<float>(in_message.P[2]);
	image_4.cy_ = static_cast<float>(in_message.P[6]);

	image_4.intrinsics_subscriber_.shutdown();
	image_4.camera_info_ok_ = true;
}
void ROSPixelCloudFusionApp::IntrinsicsCallback_5(const sensor_msgs::CameraInfo &in_message)
{
	image_5.image_size_.height = in_message.height;
	image_5.image_size_.width = in_message.width;

	image_5.camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			image_5.camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	image_5.distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		image_5.distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	image_5.fx_ = static_cast<float>(in_message.P[0]);
	image_5.fy_ = static_cast<float>(in_message.P[5]);
	image_5.cx_ = static_cast<float>(in_message.P[2]);
	image_5.cy_ = static_cast<float>(in_message.P[6]);

	image_5.intrinsics_subscriber_.shutdown();
	image_5.camera_info_ok_ = true;
}
void ROSPixelCloudFusionApp::IntrinsicsCallback_6(const sensor_msgs::CameraInfo &in_message)
{
	image_6.image_size_.height = in_message.height;
	image_6.image_size_.width = in_message.width;

	image_6.camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			image_6.camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	image_6.distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		image_6.distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	image_6.fx_ = static_cast<float>(in_message.P[0]);
	image_6.fy_ = static_cast<float>(in_message.P[5]);
	image_6.cx_ = static_cast<float>(in_message.P[2]);
	image_6.cy_ = static_cast<float>(in_message.P[6]);

	image_6.intrinsics_subscriber_.shutdown();
	image_6.camera_info_ok_ = true;
}

void ROSPixelCloudFusionApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_src, fused_topic_str = "/points_fused";
	std::string name_space_str = ros::this_node::getNamespace();
	std::string image_src1, camera_info_src1;
	std::string image_src2, camera_info_src2;
	std::string image_src3, camera_info_src3;
	std::string image_src4, camera_info_src4;
	std::string image_src5, camera_info_src5;
	std::string image_src6, camera_info_src6;

	in_private_handle.param<std::string>("points_src", points_src, "/points_raw");
	in_private_handle.param<std::string>("fused_output_topic", fused_topic_str, "/points_fused");

	in_private_handle.param<std::string>("image_src1", image_src1, "/image_rectified");
	in_private_handle.param<std::string>("camera_info_src1", camera_info_src1, "/camera_info");
	in_private_handle.param<std::string>("image_src2", image_src2, "/image_rectified");
	in_private_handle.param<std::string>("camera_info_src2", camera_info_src2, "/camera_info");
	in_private_handle.param<std::string>("image_src3", image_src3, "/image_rectified");
	in_private_handle.param<std::string>("camera_info_src3", camera_info_src3, "/camera_info");
	in_private_handle.param<std::string>("image_src4", image_src4, "/image_rectified");
	in_private_handle.param<std::string>("camera_info_src4", camera_info_src4, "/camera_info");
	in_private_handle.param<std::string>("image_src5", image_src5, "/image_rectified");
	in_private_handle.param<std::string>("camera_info_src5", camera_info_src5, "/camera_info");
	in_private_handle.param<std::string>("image_src6", image_src6, "/image_rectified");
	in_private_handle.param<std::string>("camera_info_src6", camera_info_src6, "/camera_info");

	if (name_space_str != "/")
	{
		if (name_space_str.substr(0, 2) == "//")
		{
			name_space_str.erase(name_space_str.begin());
		}
		fused_topic_str = name_space_str + fused_topic_str;
		image_src1 = name_space_str + image_src1;
		image_src2 = name_space_str + image_src2;
		image_src3 = name_space_str + image_src3;
		image_src4 = name_space_str + image_src4;
		image_src5 = name_space_str + image_src5;
		image_src6 = name_space_str + image_src6;
		camera_info_src1 = name_space_str + camera_info_src1;
		camera_info_src2 = name_space_str + camera_info_src2;
		camera_info_src3 = name_space_str + camera_info_src3;
		camera_info_src4 = name_space_str + camera_info_src4;
		camera_info_src5 = name_space_str + camera_info_src5;
		camera_info_src6 = name_space_str + camera_info_src6;
	}

	//generate subscribers and sychronizers
	cloud_subscriber_ = in_private_handle.subscribe(points_src, 1, &ROSPixelCloudFusionApp::CloudCallback_intensity, this);
	//cloud_subscriber_ = in_private_handle.subscribe(points_src, 1, &ROSPixelCloudFusionApp::CloudCallback_rgb, this);
	publisher_fused_cloud = node_handle_.advertise<sensor_msgs::PointCloud2>(fused_topic_str, 1);

	image_1.intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src1, 1, &ROSPixelCloudFusionApp::IntrinsicsCallback_1, this);
	image_2.intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src2, 1, &ROSPixelCloudFusionApp::IntrinsicsCallback_2, this);
	image_3.intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src3, 1, &ROSPixelCloudFusionApp::IntrinsicsCallback_3, this);
	image_4.intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src4, 1, &ROSPixelCloudFusionApp::IntrinsicsCallback_4, this);
	image_5.intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src5, 1, &ROSPixelCloudFusionApp::IntrinsicsCallback_5, this);
	image_6.intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src6, 1, &ROSPixelCloudFusionApp::IntrinsicsCallback_6, this);
	image_1.image_subscriber_ = in_private_handle.subscribe(image_src1, 1, &ROSPixelCloudFusionApp::ImageCallback_1, this);
	image_2.image_subscriber_ = in_private_handle.subscribe(image_src2, 1, &ROSPixelCloudFusionApp::ImageCallback_2, this);
	image_3.image_subscriber_ = in_private_handle.subscribe(image_src3, 1, &ROSPixelCloudFusionApp::ImageCallback_3, this);
	image_4.image_subscriber_ = in_private_handle.subscribe(image_src4, 1, &ROSPixelCloudFusionApp::ImageCallback_4, this);
	image_5.image_subscriber_ = in_private_handle.subscribe(image_src5, 1, &ROSPixelCloudFusionApp::ImageCallback_5, this);
	image_6.image_subscriber_ = in_private_handle.subscribe(image_src6, 1, &ROSPixelCloudFusionApp::ImageCallback_6, this);
}


void ROSPixelCloudFusionApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener1;
	tf::TransformListener transform_listener2;
	tf::TransformListener transform_listener3;
	tf::TransformListener transform_listener4;
	tf::TransformListener transform_listener5;
	tf::TransformListener transform_listener6;

	image_1.transform_listener_ = &transform_listener1;
	image_2.transform_listener_ = &transform_listener2;
	image_3.transform_listener_ = &transform_listener3;
	image_4.transform_listener_ = &transform_listener4;
	image_5.transform_listener_ = &transform_listener5;
	image_6.transform_listener_ = &transform_listener6;

	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END", __APP_NAME__);
}

ROSPixelCloudFusionApp::ROSPixelCloudFusionApp()
{
	image_1.camera_lidar_tf_ok_ = false;
	image_1.camera_info_ok_ = false;
	image_1.image_frame_id_ = "";

	image_2.camera_lidar_tf_ok_ = false;
	image_2.camera_info_ok_ = false;
	image_2.image_frame_id_ = "";

	image_3.camera_lidar_tf_ok_ = false;
	image_3.camera_info_ok_ = false;
	image_3.image_frame_id_ = "";

	image_4.camera_lidar_tf_ok_ = false;
	image_4.camera_info_ok_ = false;
	image_4.image_frame_id_ = "";

	image_5.camera_lidar_tf_ok_ = false;
	image_5.camera_info_ok_ = false;
	image_5.image_frame_id_ = "";

	image_6.camera_lidar_tf_ok_ = false;
	image_6.camera_info_ok_ = false;
	image_6.image_frame_id_ = "";
}
