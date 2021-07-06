#include "rotors_gazebo_plugins/gazebo_os1_plugin.h"
#include "rotors_gazebo_plugins/common.h"

#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#endif
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <gazebo/sensors/RaySensor.hh>
#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#define STR_Gpu "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu ""
#define STR_GPU_ ""
#endif

namespace gazebo {

GazeboOS1Plugin::GazeboOS1Plugin()
    : nh_(NULL), gaussian_noise_(0), min_range_(0), max_range_(0) {}

GazeboOS1Plugin::~GazeboOS1Plugin() {
  // Finalize the controller / Custom Callback Queue
  laser_queue_.clear();
  laser_queue_.disable();
  if (nh_) {
    nh_->shutdown();
    delete nh_;
    nh_ = NULL;
  }
  callback_laser_queue_thread_.join();
}

void GazeboOS1Plugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  RayPlugin::Load(_parent, _sdf);

  // Initialize Gazebo node
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

// Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
  parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
  parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
  if (!parent_ray_sensor_) {
    gzthrow("GazeboRosOS1" << STR_Gpu << "Laser controller requires a "
                           << STR_Gpu << "Ray Sensor as its parent");
  }
  // Get the parameters
  getSdfParam<std::string>(_sdf, "robot_namespace", robot_namespace_, "/");
  getSdfParam<std::string>(_sdf, "frame_name", frame_name_, "/world");
  getSdfParam<std::string>(_sdf, "topic_name", topic_name_, "/points");
  getSdfParam<double>(_sdf, "gaussian_noise", gaussian_noise_, 0.0);
  getSdfParam<double>(_sdf, "min_range", min_range_, 0.0);
  getSdfParam<double>(_sdf, "max_range", max_range_, INFINITY);
  getSdfParam<double>(_sdf, "rotational_frequency_hz_",
                      rotational_frequency_hz_, 10.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
           "gazebo_ros package)");
    return;
  }

  // Create node handle
  nh_ = new ros::NodeHandle(robot_namespace_);

  // Resolve tf prefix
  std::string prefix;
  nh_->getParam(std::string("tf_prefix"), prefix);
  if (robot_namespace_ != "/") {
    prefix = robot_namespace_;
  }
  boost::trim_right_if(prefix, boost::is_any_of("/"));
  frame_name_ = tf::resolve(prefix, frame_name_);

  // Advertise publisher with a custom callback queue
  if (topic_name_ != "") {
    ros::AdvertiseOptions ao =
        ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            topic_name_, 1, boost::bind(&GazeboOS1Plugin::ConnectCb, this),
            boost::bind(&GazeboOS1Plugin::ConnectCb, this), ros::VoidPtr(),
            &laser_queue_);
    pub_ = nh_->advertise(ao);
  }

  // Sensor generation off by default
  parent_ray_sensor_->SetActive(false);

  // Start custom queue for laser
  callback_laser_queue_thread_ =
      boost::thread(boost::bind(&GazeboOS1Plugin::LaserQueueThread, this));
#if GAZEBO_MAJOR_VERSION >= 7
  ROS_INFO("OS1 %s plugin ready, %i lasers", STR_GPU_,
           parent_ray_sensor_->VerticalRangeCount());
#else
  ROS_INFO("OS1 %s plugin ready, %i lasers", STR_GPU_,
           parent_ray_sensor_->GetVerticalRangeCount());
#endif
}

void GazeboOS1Plugin::ConnectCb() {
  boost::lock_guard<boost::mutex> lock(lock_);
  if (pub_.getNumSubscribers()) {
    if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                     &GazeboOS1Plugin::OnScan, this);
#else
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
                                     &GazeboOS1Plugin::OnScan, this);
#endif
    }
    parent_ray_sensor_->SetActive(true);
  } else {
#if GAZEBO_MAJOR_VERSION >= 7
    if (sub_) {
      sub_->Unsubscribe();
      sub_.reset();
    }
#endif
    parent_ray_sensor_->SetActive(false);
  }
}

void GazeboOS1Plugin::OnScan(const ConstLaserScanStampedPtr &_msg) {
#if GAZEBO_MAJOR_VERSION >= 7
  const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
  const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

  const double maxRange = parent_ray_sensor_->RangeMax();
  const double minRange = parent_ray_sensor_->RangeMin();

  const int rayCount = parent_ray_sensor_->RayCount();
  const int rangeCount = parent_ray_sensor_->RangeCount();

  const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

  const ignition::math::Angle verticalMaxAngle =
      parent_ray_sensor_->VerticalAngleMax();
  const ignition::math::Angle verticalMinAngle =
      parent_ray_sensor_->VerticalAngleMin();
#else
  math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

  const double maxRange = parent_ray_sensor_->GetRangeMax();
  const double minRange = parent_ray_sensor_->GetRangeMin();

  const int rayCount = parent_ray_sensor_->GetRayCount();
  const int rangeCount = parent_ray_sensor_->GetRangeCount();

  const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

  const math::Angle verticalMaxAngle =
      parent_ray_sensor_->GetVerticalAngleMax();
  const math::Angle verticalMinAngle =
      parent_ray_sensor_->GetVerticalAngleMin();
#endif

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);

  // time for one revolution (us) / number of horizontal rays
  const int32_t time_offset_per_beam_us =
      round(1e6 / (rotational_frequency_hz_ * rayCount));

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(7);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "time_offset_us";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::INT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "reflectivity";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "intensity";
  msg.fields[5].offset = 22;
  msg.fields[5].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[5].count = 1;
  msg.fields[6].name = "ring";
  msg.fields[6].offset = 24;
  msg.fields[6].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[6].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  int i, j;
  uint8_t *ptr = msg.data.data();
  for (i = 0; i < rangeCount; i++) {
    int32_t time_offset = i * time_offset_per_beam_us;
    for (j = 0; j < verticalRangeCount; j++) {
      // Range
      double r = _msg->scan().ranges(i + j * rangeCount);
      if ((MIN_RANGE >= r) || (r >= MAX_RANGE)) {
        continue;
      }

      // Noise
      if (gaussian_noise_ != 0.0) {
        r += GaussianKernel(0, gaussian_noise_);
      }

      // Intensity
      double intensity = _msg->scan().intensities(i + j * rangeCount);

      // Get angles of ray to get xyz for point
      double yAngle;
      double pAngle;

      if (rangeCount > 1) {
        yAngle = i * yDiff / (rangeCount - 1) + minAngle.Radian();
      } else {
        yAngle = minAngle.Radian();
      }

      if (verticalRayCount > 1) {
        pAngle =
            j * pDiff / (verticalRangeCount - 1) + verticalMinAngle.Radian();
      } else {
        pAngle = verticalMinAngle.Radian();
      }

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float *)(ptr + 0)) = r * cos(pAngle) * cos(yAngle);
        *((float *)(ptr + 4)) = r * cos(pAngle) * sin(yAngle);
#if GAZEBO_MAJOR_VERSION > 2
        *((float *)(ptr + 8)) = r * sin(pAngle);
#else
        *((float *)(ptr + 8)) = -r * sin(pAngle);
#endif
        *((int32_t *)(ptr + 16)) = time_offset;
        *((uint16_t *)(ptr + 20)) = 0u;  // reflectivity placeholder
        *((uint16_t *)(ptr + 22)) = intensity;

#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t *)(ptr + 24)) = j;  // ring
#else
        *((uint16_t *)(ptr + 20)) = verticalRangeCount - 1 - j;  // ring
#endif
        ptr += POINT_STEP;
      }
    }
  }
  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step);  // Shrink to actual size

  // Publish output
  pub_.publish(msg);
}

void GazeboOS1Plugin::LaserQueueThread() {
  while (nh_->ok()) {
    laser_queue_.callAvailable(ros::WallDuration(0.01));
  }
}
double GazeboOS1Plugin::GaussianKernel(double mu, double sigma) {
  // using Box-Muller transform to generate two independent standard normally
  // distributed normal variables
  // see wikipedia
  double U =
      (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
  double V =
      (double)rand() / (double)RAND_MAX;  // normalized uniform random variable
  return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboOS1Plugin)

}  // namespace gazebo
