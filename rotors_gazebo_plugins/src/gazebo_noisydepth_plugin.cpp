/*
 * Copyright 2018 Michael Pantic, ASL, ETH Zurich, Switzerland
 *
 * Forked from GazeboNoisyDepth Plugin, retaining original copyright header:
 *
 * Copyright 2013 Open Source Robotics Foundation
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
 *
*/

/*
   Desc: GazeboNoisyDepth plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <rotors_gazebo_plugins/gazebo_noisydepth_plugin.h>

#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>


#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(GazeboNoisyDepth)

////////////////////////////////////////////////////////////////////////////////
// Constructor
    GazeboNoisyDepth::GazeboNoisyDepth()
    {
        this->depth_info_connect_count_ = 0;
        this->depth_image_connect_count_ = 0;
        this->last_depth_image_camera_info_update_time_ = common::Time(0);
    }

////////////////////////////////////////////////////////////////////////////////
// Destructor
    GazeboNoisyDepth::~GazeboNoisyDepth()
    {
    }

////////////////////////////////////////////////////////////////////////////////
// Load the controller
    void GazeboNoisyDepth::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        DepthCameraPlugin::Load(_parent, _sdf);

        // copying from DepthCameraPlugin into GazeboRosCameraUtils
        this->parentSensor_ = this->parentSensor;
        this->width_ = this->width;
        this->height_ = this->height;
        this->depth_ = this->depth;
        this->format_ = this->format;
        this->camera_ = this->depthCamera;
        this->camera_->SetClipDist(0.2,100.0);

        // using a different default
        if (!_sdf->HasElement("imageTopicName"))
            this->image_topic_name_ = "ir/image_raw";
        if (!_sdf->HasElement("cameraInfoTopicName"))
            this->camera_info_topic_name_ = "ir/camera_info";


        // depth image stuff
        if (!_sdf->HasElement("depthImageTopicName"))
            this->depth_image_topic_name_ = "depth/image_raw";
        else
            this->depth_image_topic_name_ = _sdf->GetElement("depthImageTopicName")->Get<std::string>();

        if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
            this->depth_image_camera_info_topic_name_ = "depth/camera_info";
        else
            this->depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();


        load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboNoisyDepth::Advertise, this));
        this->depth_cutoff_min_ = 0.2;
        this->depth_cutoff_max_ = 20;
        this->noise_model.reset(new KinectDepthNoiseModel());
        GazeboRosCameraUtils::Load(_parent, _sdf);

    }

    void GazeboNoisyDepth::Advertise()
    {

        ros::AdvertiseOptions depth_image_ao =
                ros::AdvertiseOptions::create< sensor_msgs::Image >(
                        this->depth_image_topic_name_,1,
                        boost::bind( &GazeboNoisyDepth::DepthImageConnect,this),
                        boost::bind( &GazeboNoisyDepth::DepthImageDisconnect,this),
                        ros::VoidPtr(), &this->camera_queue_);
        this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

        ros::AdvertiseOptions depth_image_camera_info_ao =
                ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
                        this->depth_image_camera_info_topic_name_,1,
                        boost::bind( &GazeboNoisyDepth::DepthInfoConnect,this),
                        boost::bind( &GazeboNoisyDepth::DepthInfoDisconnect,this),
                        ros::VoidPtr(), &this->camera_queue_);
        this->depth_image_camera_info_pub_ = this->rosnode_->advertise(depth_image_camera_info_ao);
    }


////////////////////////////////////////////////////////////////////////////////
// Increment count
    void GazeboNoisyDepth::DepthImageConnect()
    {
        this->depth_image_connect_count_++;
        this->parentSensor->SetActive(true);
    }
////////////////////////////////////////////////////////////////////////////////
// Decrement count
    void GazeboNoisyDepth::DepthImageDisconnect()
    {
        this->depth_image_connect_count_--;
    }

////////////////////////////////////////////////////////////////////////////////
// Increment count
    void GazeboNoisyDepth::DepthInfoConnect()
    {
        this->depth_info_connect_count_++;
    }
////////////////////////////////////////////////////////////////////////////////
// Decrement count
    void GazeboNoisyDepth::DepthInfoDisconnect()
    {
        this->depth_info_connect_count_--;
    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
    void GazeboNoisyDepth::OnNewDepthFrame(const float *_image,
                                                unsigned int _width, unsigned int _height, unsigned int _depth,
                                                const std::string &_format)
    {
        if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
            return;

        this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();
        if (this->parentSensor->IsActive())
        {
            if (this->depth_image_connect_count_ <= 0 &&
                (*this->image_connect_count_) <= 0)
            {
                this->parentSensor->SetActive(false);
            }
            else
            {
                if (this->depth_image_connect_count_ > 0)
                    this->FillDepthImage(_image);
            }
        }
        else
        {
            if (this->depth_image_connect_count_ <= 0)
                // do this first so there's chance for sensor to run 1 frame after activate
                this->parentSensor->SetActive(true);
        }
        PublishCameraInfo();
    }

////////////////////////////////////////////////////////////////////////////////
// Update the controller
    void GazeboNoisyDepth::OnNewImageFrame(const unsigned char *_image,
                                                unsigned int _width, unsigned int _height, unsigned int _depth,
                                                const std::string &_format)
    {
        if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
            return;

        //ROS_ERROR_NAMED("openni_kinect", "camera_ new frame %s %s",this->parentSensor_->Name().c_str(),this->frame_name_.c_str());
        this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();

        if (this->parentSensor->IsActive())
        {
            if (this->depth_image_connect_count_ <= 0 &&
                (*this->image_connect_count_) <= 0)
            {
                this->parentSensor->SetActive(false);
            }
            else
            {
                if ((*this->image_connect_count_) > 0)
                    this->PutCameraData(_image);
            }
        }
        else
        {
            if ((*this->image_connect_count_) > 0)
                // do this first so there's chance for sensor to run 1 frame after activate
                this->parentSensor->SetActive(true);
        }
    }


////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
    void GazeboNoisyDepth::FillDepthImage(const float *_src)
    {
        this->lock_.lock();
        // copy data into image
        this->depth_image_msg_.header.frame_id = this->frame_name_;
        this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
        this->depth_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

        ///copy from depth to depth image message
        FillDepthImageHelper(this->depth_image_msg_,
                             this->height,
                             this->width,
                             this->skip_,
                             (void*)_src );

        this->depth_image_pub_.publish(this->depth_image_msg_);

        this->lock_.unlock();
    }



// Fill depth information
    bool GazeboNoisyDepth::FillDepthImageHelper(
            sensor_msgs::Image& image_msg,
            uint32_t rows_arg, uint32_t cols_arg,
            uint32_t step_arg, void* data_arg)
    {
        image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        image_msg.height = rows_arg;
        image_msg.width = cols_arg;
        image_msg.step = sizeof(float) * cols_arg;
        image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
        image_msg.is_bigendian = 0;


        float* dest = (float*)(&(image_msg.data[0]));
        float* toCopyFrom = (float*)data_arg;
        int index = 0;

        memcpy(dest, toCopyFrom, sizeof(float)*width*height);

        noise_model->ApplyNoise(dest, width, height);

        return true;
    }

    void GazeboNoisyDepth::PublishCameraInfo()
    {
        ROS_DEBUG_NAMED("openni_kinect", "publishing default camera info, then openni kinect camera info");
        GazeboRosCameraUtils::PublishCameraInfo();

        if (this->depth_info_connect_count_ > 0)
        {
            this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
#if GAZEBO_MAJOR_VERSION >= 8
            common::Time cur_time = this->world_->SimTime();
#else
            common::Time cur_time = this->world_->GetSimTime();
#endif
            if (this->sensor_update_time_ - this->last_depth_image_camera_info_update_time_ >= this->update_period_)
            {
                this->PublishCameraInfo(this->depth_image_camera_info_pub_);
                this->last_depth_image_camera_info_update_time_ = this->sensor_update_time_;
            }
        }
    }


}