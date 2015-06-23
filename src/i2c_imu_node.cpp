//  Copyright (c) 2014 Justin Eskesen
//
//  This file is part of i2c_imu
//
//  i2c_imu is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  i2c_imu is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with i2c_imu.  If not, see <http://www.gnu.org/licenses/>.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <angles/angles.h>
#include <geometry_msgs/Vector3.h>
#include <i2c_imu/SetCalibration.h>
#include <std_srvs/Empty.h>

#include "RTIMULib.h"
#include "RTIMUSettings.h"

#define G_2_MPSS 9.80665
#define uT_2_T 1000000

class I2cImu
{
public:
	I2cImu();

	void update();
	void spin();
	bool updateAccelCalibration(i2c_imu::SetCalibration::Request &req, i2c_imu::SetCalibrationResponse &resp);
	bool updateCompassCalibration(i2c_imu::SetCalibration::Request &req, i2c_imu::SetCalibrationResponse &resp);
	bool disableCompassCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
	bool disableAccelCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
	bool enableCompassCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
	bool enableAccelCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);


private:
	//ROS Stuff
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	// sensor msg topic output
	sensor_msgs::Imu imu_msg;

	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher imu_pub_;
	ros::Publisher magnetometer_pub_;
	ros::Publisher euler_pub_;

	ros::ServiceServer disable_accel_cal_srv_;
	ros::ServiceServer disable_compass_cal_srv_;
	ros::ServiceServer enable_accel_cal_srv_;
	ros::ServiceServer enable_compass_cal_srv_;
	ros::ServiceServer set_compass_cal_;
	ros::ServiceServer set_accel_cal_;

	std::string imu_frame_id_;

	ros::Time last_update_;
	double declination_radians_;

	//RTUIMULib stuff
	RTIMU *imu_;

	class ImuSettings: public RTIMUSettings
	{
	public:
		ImuSettings(ros::NodeHandle* nh) : settings_nh_(nh){setDefaults();}
		virtual bool loadSettings();
		virtual bool saveSettings(){return true;}
		void updateMinMaxAccelCalibration(geometry_msgs::Vector3 &min, geometry_msgs::Vector3 &max)
		{
			this->m_compassCalMin.setX((RTFLOAT)min.x);
			this->m_compassCalMin.setY((RTFLOAT)min.y);
			this->m_compassCalMin.setZ((RTFLOAT)min.z);
			this->m_compassCalMax.setX((RTFLOAT)max.x);
			this->m_compassCalMax.setY((RTFLOAT)max.y);
			this->m_compassCalMax.setZ((RTFLOAT)max.z);
		}
		void updateMinMaxCompassCalibration(geometry_msgs::Vector3 &min, geometry_msgs::Vector3 &max)
		{
			this->m_accelCalMin.setX((RTFLOAT)min.x);
			this->m_accelCalMin.setY((RTFLOAT)min.y);
			this->m_accelCalMin.setZ((RTFLOAT)min.z);
			this->m_accelCalMax.setX((RTFLOAT)max.x);
			this->m_accelCalMax.setY((RTFLOAT)max.y);
			this->m_accelCalMax.setZ((RTFLOAT)max.z);
		}
	private:
		ros::NodeHandle* settings_nh_;

	} imu_settings_;
};

I2cImu::I2cImu() :
		nh_(), private_nh_("~"), imu_settings_(&private_nh_)
{

	// do all the ros parameter reading & pulbishing
	private_nh_.param<std::string>("frame_id", imu_frame_id_, "imu_link");

	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("data",10);

	disable_accel_cal_srv_=nh_.advertiseService("disable_accel_cal", &I2cImu::disableAccelCalibrationMode,this);
	set_accel_cal_=nh_.advertiseService("set_accel_cal", &I2cImu::updateAccelCalibration,this);
	enable_accel_cal_srv_=nh_.advertiseService("enable_accel_cal", &I2cImu::enableAccelCalibrationMode,this);

	disable_compass_cal_srv_=nh_.advertiseService("disable_compass_cal", &I2cImu::disableCompassCalibrationMode,this);
	set_compass_cal_=nh_.advertiseService("set_compass_cal", &I2cImu::updateCompassCalibration,this);
	enable_compass_cal_srv_=nh_.advertiseService("enable_compass_cal", &I2cImu::enableCompassCalibrationMode,this);



	bool magnetometer;
	private_nh_.param("publish_magnetometer", magnetometer, false);
	if (magnetometer)
	{
		magnetometer_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag", 10, false);
	}

	bool euler;
	private_nh_.param("publish_euler", euler, false);
	if (euler)
	{
		euler_pub_ = nh_.advertise<geometry_msgs::Vector3>("euler", 10, false);
	}

	std::vector<double> orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance;
	if (private_nh_.getParam("orientation_covariance", orientation_covariance) && orientation_covariance.size() == 9)
	{
		for(int i=0; i<9; i++)
		{
			imu_msg.orientation_covariance[i]=orientation_covariance[i];
		}
	}

	if (private_nh_.getParam("angular_velocity_covariance", angular_velocity_covariance) && angular_velocity_covariance.size() == 9)
	{
		for(int i=0; i<9; i++)
		{
			imu_msg.angular_velocity_covariance[i]=angular_velocity_covariance[i];
		}
	}

	if (private_nh_.getParam("linear_acceleration_covariance", linear_acceleration_covariance) && linear_acceleration_covariance.size() == 9)
	{
		for(int i=0; i<9; i++)
		{
			imu_msg.linear_acceleration_covariance[i]=linear_acceleration_covariance[i];
		}
	}

	imu_settings_.loadSettings();

	private_nh_.param("magnetic_declination", declination_radians_, 0.0);

	// now set up the IMU

	imu_ = RTIMU::createIMU(&imu_settings_);
	if (imu_ == NULL)
	{
		ROS_FATAL("I2cImu - %s - Failed to open the i2c device", __FUNCTION__);
		ROS_BREAK();
	}

	if (!imu_->IMUInit())
	{
		ROS_FATAL("I2cImu - %s - Failed to init the IMU", __FUNCTION__);
		ROS_BREAK();
	}

        imu_->setSlerpPower(0.02);
        imu_->setGyroEnable(true);
        imu_->setAccelEnable(true);
        imu_->setCompassEnable(true);
	
}

void I2cImu::update()
{

	while (imu_->IMURead())
	{
		RTIMU_DATA imuData = imu_->getIMUData();

		ros::Time current_time = ros::Time::now();


		imu_msg.header.stamp = current_time;
		imu_msg.header.frame_id = imu_frame_id_;
		imu_msg.orientation.x = imuData.fusionQPose.x();
		imu_msg.orientation.y = imuData.fusionQPose.y();
		imu_msg.orientation.z = imuData.fusionQPose.z();
		imu_msg.orientation.w = imuData.fusionQPose.scalar();

		imu_msg.angular_velocity.x = imuData.gyro.x();
		imu_msg.angular_velocity.y = imuData.gyro.y();
		imu_msg.angular_velocity.z = imuData.gyro.z();

		imu_msg.linear_acceleration.x = imuData.accel.x() * G_2_MPSS;
		imu_msg.linear_acceleration.y = imuData.accel.y() * G_2_MPSS;
		imu_msg.linear_acceleration.z = imuData.accel.z() * G_2_MPSS;

		imu_pub_.publish(imu_msg);

		if (magnetometer_pub_ != NULL && imuData.compassValid)
		{
			sensor_msgs::MagneticField msg;

			msg.header.frame_id=imu_frame_id_;
			msg.header.stamp=ros::Time::now();

			msg.magnetic_field.x = imuData.compass.x()/uT_2_T;
			msg.magnetic_field.y = imuData.compass.y()/uT_2_T;
			msg.magnetic_field.z = imuData.compass.z()/uT_2_T;

			magnetometer_pub_.publish(msg);
		}

		if (euler_pub_ != NULL)
		{
			geometry_msgs::Vector3 msg;
			msg.x = imuData.fusionPose.x();
			msg.y = imuData.fusionPose.y();
			msg.z = -imuData.fusionPose.z();
			msg.z = (-imuData.fusionPose.z()) - declination_radians_;
			euler_pub_.publish(msg);
		}
	}

}

bool I2cImu::ImuSettings::loadSettings()
{
	ROS_INFO("%s: reading IMU parameters from param server", __FUNCTION__);
	int temp_int;

	// General
	settings_nh_->getParam("imu_type", m_imuType);
	settings_nh_->getParam("fusion_type", m_fusionType);

	if(settings_nh_->getParam("i2c_bus", temp_int))
		m_I2CBus = (unsigned char) temp_int;

	if(settings_nh_->getParam("i2c_slave_address", temp_int))
			m_I2CSlaveAddress = (unsigned char) temp_int;


	//double declination_radians;
	//settings_nh_->param("magnetic_declination", declination_radians, 0.0);
	//m_compassAdjDeclination = angles::to_degrees(declination_radians);

	//MPU9150
	settings_nh_->getParam("mpu9150/gyro_accel_sample_rate", m_MPU9150GyroAccelSampleRate);
	settings_nh_->getParam("mpu9150/compass_sample_rate", m_MPU9150CompassSampleRate);
	settings_nh_->getParam("mpu9150/accel_full_scale_range", m_MPU9150AccelFsr);
	settings_nh_->getParam("mpu9150/gyro_accel_low_pass_filter", m_MPU9150GyroAccelLpf);
	settings_nh_->getParam("mpu9150/gyro_full_scale_range", m_MPU9150GyroFsr);

	//MPU9250
	settings_nh_->getParam("mpu9250/gyro_accel_sample_rate", m_MPU9250GyroAccelSampleRate);
	settings_nh_->getParam("mpu9250/compass_sample_rate", m_MPU9250CompassSampleRate);
	settings_nh_->getParam("mpu9250/accel_full_scale_range", m_MPU9250AccelFsr);
	settings_nh_->getParam("mpu9250/accel_low_pass_filter", m_MPU9250AccelLpf);
	settings_nh_->getParam("mpu9250/gyro_full_scale_range", m_MPU9250GyroFsr);
	settings_nh_->getParam("mpu9250/gyro_low_pass_filter", m_MPU9250GyroLpf);

	//GD20HM303D
	settings_nh_->getParam("GD20HM303D/gyro_sample_rate", m_GD20HM303DGyroSampleRate);
	settings_nh_->getParam("GD20HM303D/accel_sample_rate", m_GD20HM303DAccelSampleRate);
	settings_nh_->getParam("GD20HM303D/compass_sample_rate", m_GD20HM303DCompassSampleRate);
	settings_nh_->getParam("GD20HM303D/accel_full_scale_range", m_GD20HM303DAccelFsr);
	settings_nh_->getParam("GD20HM303D/gyro_full_scale_range", m_GD20HM303DGyroFsr);
	settings_nh_->getParam("GD20HM303D/compass_full_scale_range", m_GD20HM303DCompassFsr);
	settings_nh_->getParam("GD20HM303D/accel_low_pass_filter", m_GD20HM303DAccelLpf);
	settings_nh_->getParam("GD20HM303D/gyro_high_pass_filter", m_GD20HM303DGyroHpf);
	settings_nh_->getParam("GD20HM303D/gyro_bandwidth", m_GD20HM303DGyroBW);

	//GD20M303DLHC
	settings_nh_->getParam("GD20M303DLHC/gyro_sample_rate",m_GD20M303DLHCGyroSampleRate);
	settings_nh_->getParam("GD20M303DLHC/accel_sample_rate",m_GD20M303DLHCAccelSampleRate);
	settings_nh_->getParam("GD20M303DLHC/compass_sample_rate",m_GD20M303DLHCCompassSampleRate);
	settings_nh_->getParam("GD20M303DLHC/accel_full_scale_range",m_GD20M303DLHCAccelFsr);
	settings_nh_->getParam("GD20M303DLHC/gyro_full_scale_range",m_GD20M303DLHCGyroFsr);
	settings_nh_->getParam("GD20M303DLHC/compass_full_scale_range",m_GD20M303DLHCCompassFsr);
	settings_nh_->getParam("GD20M303DLHC/gyro_high_pass_filter",m_GD20M303DLHCGyroHpf);
	settings_nh_->getParam("GD20M303DLHC/gyro_bandwidth",m_GD20M303DLHCGyroBW);

	//GD20HM303DLHC
	settings_nh_->getParam("GD20HM303DLHC/gyro_sample_rate", m_GD20HM303DLHCGyroSampleRate);
	settings_nh_->getParam("GD20HM303DLHC/accel_sample_rate",m_GD20HM303DLHCAccelSampleRate);
	settings_nh_->getParam("GD20HM303DLHC/compass_sample_rate",m_GD20HM303DLHCCompassSampleRate);
	settings_nh_->getParam("GD20HM303DLHC/accel_full_scale_range",m_GD20HM303DLHCAccelFsr);
	settings_nh_->getParam("GD20HM303DLHC/gyro_full_scale_range",m_GD20HM303DLHCGyroFsr);
	settings_nh_->getParam("GD20HM303DLHC/compass_full_scale_range",m_GD20HM303DLHCCompassFsr);
	settings_nh_->getParam("GD20HM303DLHC/gyro_high_pass_filter",m_GD20HM303DLHCGyroHpf);
	settings_nh_->getParam("GD20HM303DLHC/gyro_bandwidth",m_GD20HM303DLHCGyroBW);

	//LSM9DS0
	settings_nh_->getParam("LSM9DS0/gyro_sample_rate",m_LSM9DS0GyroSampleRate);
	settings_nh_->getParam("LSM9DS0/accel_sample_rate",m_LSM9DS0AccelSampleRate);
	settings_nh_->getParam("LSM9DS0/compass_sample_rate",m_LSM9DS0CompassSampleRate);
	settings_nh_->getParam("LSM9DS0/accel_full_scale_range",m_LSM9DS0AccelFsr);
	settings_nh_->getParam("LSM9DS0/gyro_full_scale_range",m_LSM9DS0GyroFsr);
	settings_nh_->getParam("LSM9DS0/compass_full_scale_range",m_LSM9DS0CompassFsr);
	settings_nh_->getParam("LSM9DS0/accel_low_pass_filter",m_LSM9DS0AccelLpf);
	settings_nh_->getParam("LSM9DS0/gyro_high_pass_filter",m_LSM9DS0GyroHpf);
	settings_nh_->getParam("LSM9DS0/gyro_bandwidth",m_LSM9DS0GyroBW);

	std::vector<double> compass_max, compass_min;
	if (settings_nh_->getParam("calib/compass_min", compass_min)
			&& settings_nh_->getParam("calib/compass_max", compass_max)
			&& compass_min.size() == 3 && compass_max.size() == 3)
	{
		m_compassCalMin = RTVector3(compass_min[0], compass_min[1], compass_min[2]);
		m_compassCalMax = RTVector3(compass_max[0],compass_max[1], compass_max[2]);
		m_compassCalValid = true;
		
		ROS_INFO("Got Calibration for Compass");
	}
	else
	{
	  	ROS_INFO("No Calibration for Compass");
	}
	
	std::vector<double> accel_max, accel_min;
	if (settings_nh_->getParam("calib/accel_min", accel_min)
			&& settings_nh_->getParam("calib/accel_max", accel_max)
			&& accel_min.size() == 3 && accel_max.size() == 3)
	{
		m_accelCalMin = RTVector3(accel_min[0], accel_min[1], accel_min[2]);
		m_accelCalMax = RTVector3(accel_max[0],accel_max[1], accel_max[2]);
		m_accelCalValid = true;
		
		ROS_INFO("Got Calibration for Accelerometer");
	}
	else
	{
	  	ROS_INFO("No Calibration for Accelerometer"); 
	}


	return true;
}

void I2cImu::spin()
{
	ros::Rate r(1.0 / (imu_->IMUGetPollInterval() / 1000.0));
	while (nh_.ok())
	{
		update();

		ros::spinOnce();
		r.sleep();
	}
}

bool I2cImu::updateAccelCalibration(i2c_imu::SetCalibration::Request &req,
		                            i2c_imu::SetCalibrationResponse &resp)
{
	imu_settings_.updateMinMaxAccelCalibration(req.min,req.max);
	imu_->setCalibrationData();
	return true;
}

bool I2cImu::updateCompassCalibration(i2c_imu::SetCalibration::Request &req,
									  i2c_imu::SetCalibrationResponse &resp)
{
	imu_settings_.updateMinMaxCompassCalibration(req.min,req.max);
	imu_->setCalibrationData();
	return true;
}

bool I2cImu::disableCompassCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	imu_->setCompassCalibrationMode(false);
	return true;
}

bool I2cImu::disableAccelCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	imu_->setAccelCalibrationMode(false);
	return true;
}

bool I2cImu::enableCompassCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	imu_->setCompassCalibrationMode(true);
	return true;
}

bool I2cImu::enableAccelCalibrationMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	imu_->setAccelCalibrationMode(true);
	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "i2c_imu_node");

	ROS_INFO("RTIMU Node for ROS");

	I2cImu i2c_imu;
	i2c_imu.spin();

	return (0);
}
// EOF
