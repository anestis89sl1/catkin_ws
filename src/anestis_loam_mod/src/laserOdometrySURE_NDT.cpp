// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <cmath>

#include <anestis_loam_mod/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/registration/icp.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

void run();
const float scanPeriod = 0.1;

const int skipFrameNum = 1;
bool systemInited = false;

double timeInterestPoints= 0;
double timeLaserCloudFullRes = 0;
double timeImuTrans = 0;

bool newInterestPoints= false;
bool newLaserCloudFullRes = false;
bool newImuTrans = false;
bool reset=false;

pcl::PointCloud<PointType>::Ptr interestPoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr interestPointsLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());

int interestPointsLastNum;



float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;

  
void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                       float alx, float aly, float alz, float &acx, float &acy, float &acz)
{
  float sbcx = sin(bcx);
  float cbcx = cos(bcx);
  float sbcy = sin(bcy);
  float cbcy = cos(bcy);
  float sbcz = sin(bcz);
  float cbcz = cos(bcz);

  float sblx = sin(blx);
  float cblx = cos(blx);
  float sbly = sin(bly);
  float cbly = cos(bly);
  float sblz = sin(blz);
  float cblz = cos(blz);

  float salx = sin(alx);
  float calx = cos(alx);
  float saly = sin(aly);
  float caly = cos(aly);
  float salz = sin(alz);
  float calz = cos(alz);

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
            - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
            - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  acx = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
  
  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
               - calx*calz*cblx*sblz);
  acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}

void laserCloudInterestHandler(const sensor_msgs::PointCloud2ConstPtr& interestPoints2)
{
	timeInterestPoints = interestPoints2->header.stamp.toSec();
    interestPoints->clear();

    pcl::fromROSMsg(*interestPoints2, *interestPoints);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*interestPoints,*interestPoints, indices);
    newInterestPoints= true;
}


void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
  timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

  laserCloudFullRes->clear();
  pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudFullRes,*laserCloudFullRes, indices);
  newLaserCloudFullRes = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2)
{
  timeImuTrans = imuTrans2->header.stamp.toSec();

  imuTrans->clear();
  pcl::fromROSMsg(*imuTrans2, *imuTrans);

  imuPitchStart = imuTrans->points[0].x;
  imuYawStart = imuTrans->points[0].y;
  imuRollStart = imuTrans->points[0].z;

  imuPitchLast = imuTrans->points[1].x;
  imuYawLast = imuTrans->points[1].y;
  imuRollLast = imuTrans->points[1].z;

  imuShiftFromStartX = imuTrans->points[2].x;
  imuShiftFromStartY = imuTrans->points[2].y;
  imuShiftFromStartZ = imuTrans->points[2].z;

  imuVeloFromStartX = imuTrans->points[3].x;
  imuVeloFromStartY = imuTrans->points[3].y;
  imuVeloFromStartZ = imuTrans->points[3].z;

  newImuTrans = true;
}

void paramHandler(const std_msgs::Float32MultiArray::ConstPtr& array) { ros::shutdown(); }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laserOdometry");
	ros::NodeHandle nh;

	ros::Subscriber subInterestPoints = nh.subscribe<sensor_msgs::PointCloud2>
										 ("/laser_interest", 2, laserCloudInterestHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> 
										 ("/velodyne_cloud_2", 2, laserCloudFullResHandler);

	ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2> 
								("/imu_trans", 5, imuTransHandler);

	ros::Subscriber subFeatParams = nh.subscribe<std_msgs::Float32MultiArray> ("/SUREparams", 5, paramHandler);

	ros::Publisher pubInterestPointsLast = nh.advertise<sensor_msgs::PointCloud2>
										   ("/laser_interest_last", 2);

	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> 
										("/velodyne_cloud_3", 2);

	ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
	nav_msgs::Odometry laserOdometry;
	laserOdometry.header.frame_id = "/camera_init";
	laserOdometry.child_frame_id = "/laser_odom";

	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform laserOdometryTrans;
	laserOdometryTrans.frame_id_ = "/camera_init";
	laserOdometryTrans.child_frame_id_ = "/laser_odom";




	int frameCount = skipFrameNum;
	ros::Rate rate(100);
	bool status = ros::ok();
	while (status) {
		ros::spinOnce();

		if (newInterestPoints && newLaserCloudFullRes && newImuTrans &&
			fabs(timeLaserCloudFullRes - timeInterestPoints) < 0.005 &&
			fabs(timeImuTrans - timeInterestPoints) < 0.005) 
		{
			newInterestPoints = false;
			newLaserCloudFullRes = false;
			newImuTrans = false;
			if (!systemInited) 
			{
				pcl::PointCloud<PointType>::Ptr laserCloudTemp = interestPoints;
				interestPoints = interestPointsLast;
				interestPointsLast = laserCloudTemp;

				sensor_msgs::PointCloud2 interestPointsLast2;
				pcl::toROSMsg(*interestPointsLast, interestPointsLast2);
				interestPointsLast2.header.stamp = ros::Time().fromSec(timeInterestPoints);
				interestPointsLast2.header.frame_id = "/camera";
				pubInterestPointsLast.publish(interestPointsLast2);


				systemInited = true;
				continue;
			}

			int InterestPointsNum = interestPoints->points.size();
			int InterestPointsLNum = interestPointsLast->points.size();
			if(InterestPointsNum<9||InterestPointsLNum<9)continue;

		     ////ICP
			pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
			icp.setInputSource(interestPointsLast);
			icp.setInputTarget(interestPoints);

			// Set the max correspondence distance to 5cm 
			// (e.g., correspondences with higher distances will be ignored)
			//icp.setMaxCorrespondenceDistance (0.05);
			// Set the maximum number of iterations (criterion 1)
			icp.setMaximumIterations (50);
			// Set the transformation epsilon (criterion 2)
			//icp.setTransformationEpsilon (1e-8);
			// Set the euclidean distance difference epsilon (criterion 3)
			//icp.setEuclideanFitnessEpsilon (1);





			//for(int i=0;i<InterestPointsNum;i++)
			//	std::cout<<interestPoints->points[i]<<std::endl;
			pcl::PointCloud<pcl::PointXYZI> Final;
			icp.align(Final);
			Eigen::Matrix4f Tm=icp.getFinalTransformation();
			////ICP
			tf::Matrix3x3 tf3d;
			tf3d.setValue(
					static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)), 
					static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)), 
					static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2))
					);

			tf::Quaternion tfqt;
			tf::Vector3 origin;
			origin.setValue(
					static_cast<double>(Tm(0,3)),
					static_cast<double>(Tm(1,3)),
					static_cast<double>(Tm(2,3)));
			tf3d.getRotation(tfqt);
			laserOdometryTrans.setOrigin(origin);
			laserOdometryTrans.setRotation(tfqt);
			laserOdometryTrans.stamp_ = ros::Time().fromSec(timeInterestPoints);
			tfBroadcaster.sendTransform(laserOdometryTrans);

			double rxd, ryd, rzd;
			float tx, ty, tz;
			//float rx, ry, rz, tx, ty, tz;
			tf3d.getRPY(rxd,ryd,rzd);
			tx= static_cast<float>(Tm(0,3));
			ty= static_cast<float>(Tm(1,3));
			tz= static_cast<float>(Tm(2,3));

			pcl::transformPointCloud(*interestPoints,*interestPointsLast, Tm);
			pcl::transformPointCloud(*laserCloudFullRes,*laserCloudFullRes,Tm);


			geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rzd, -rxd, -ryd);

			laserOdometry.header.stamp = ros::Time().fromSec(timeInterestPoints);
			laserOdometry.pose.pose.orientation.x = -geoQuat.y;
			laserOdometry.pose.pose.orientation.y = -geoQuat.z;
			laserOdometry.pose.pose.orientation.z = geoQuat.x;
			laserOdometry.pose.pose.orientation.w = geoQuat.w;
			laserOdometry.pose.pose.position.x = tx;
			laserOdometry.pose.pose.position.y = ty;
			laserOdometry.pose.pose.position.z = tz;
			pubLaserOdometry.publish(laserOdometry);

			/*
			//////////////// 
			InterestPointsNum = interestPoints->points.size();
			///////////////
			frameCount++;
			
			if (frameCount >= skipFrameNum + 1) {
				int laserCloudFullResNum = laserCloudFullRes->points.size();
				for (int i = 0; i < laserCloudFullResNum; i++) 
				{
					TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
				}
			}

			pcl::PointCloud<PointType>::Ptr laserCloudTemp =interestPoints;
			interestPoints= interestPointsLast;
			interestPointsLast= laserCloudTemp;
			*/

			interestPointsLastNum = interestPointsLast->points.size();

			if (frameCount >= skipFrameNum + 1) 
			{
				frameCount = 0;

				sensor_msgs::PointCloud2 interestPointsLast2;
				pcl::toROSMsg(*interestPointsLast, interestPointsLast2);
				interestPointsLast2.header.stamp = ros::Time().fromSec(timeInterestPoints);
				interestPointsLast2.header.frame_id = "/camera";
				pubInterestPointsLast.publish(interestPointsLast2);

				sensor_msgs::PointCloud2 laserCloudFullRes3;
				pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
				laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeInterestPoints);
				laserCloudFullRes3.header.frame_id = "/camera";
				pubLaserCloudFullRes.publish(laserCloudFullRes3);
			}
		}
		status = ros::ok();
		rate.sleep();
	}
	return 0;
}
