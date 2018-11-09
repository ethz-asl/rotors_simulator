#include "rotors_gazebo_plugins/gazebo_ros_laser_plugin.h"
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

ROSLaserPlugin::ROSLaserPlugin()
    : nh_(NULL), gaussian_noise_(0), min_range_(0), max_range_(0) {}

ROSLaserPlugin::~ROSLaserPlugin() {
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

void ROSLaserPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  RayPlugin::Load(_parent, _sdf);

  std::string model_name_;
  std::string joint_name_;

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
    gzthrow("GazeboRosVelodyne" << STR_Gpu << "Laser controller requires a "
                                << STR_Gpu << "Ray Sensor as its parent");
  }
  // Get the parameters
  getSdfParam<std::string>(_sdf, "robot_namespace", robot_namespace_, "/");
  getSdfParam<std::string>(_sdf, "frame_name", frame_name_, "/world");
  getSdfParam<std::string>(_sdf, "topic_name", topic_name_, "/points");
  getSdfParam<double>(_sdf, "gaussian_noise", gaussian_noise_, 0.0);
  getSdfParam<double>(_sdf, "min_range", min_range_, 0.0);
  getSdfParam<double>(_sdf, "max_range", max_range_, INFINITY);

  getSdfParam<std::string>(_sdf, "model_name", model_name_, "lidar_sensor");
  getSdfParam<std::string>(_sdf, "joint_name", joint_name_, "joint");
  joint_ = world->GetModel(model_name_)->GetJoint(joint_name_);

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
            topic_name_, 1, boost::bind(&ROSLaserPlugin::ConnectCb, this),
            boost::bind(&ROSLaserPlugin::ConnectCb, this), ros::VoidPtr(),
            &laser_queue_);
    pub_ = nh_->advertise(ao);
  }

  // Sensor generation off by default
  parent_ray_sensor_->SetActive(false);

  // Start custom queue for laser
  callback_laser_queue_thread_ =
      boost::thread(boost::bind(&ROSLaserPlugin::LaserQueueThread, this));
#if GAZEBO_MAJOR_VERSION >= 7
  ROS_INFO("Lidar %s plugin ready, %i lasers", STR_GPU_,
           parent_ray_sensor_->VerticalRangeCount());
#else
  ROS_INFO("Lidar %s plugin ready, %i lasers", STR_GPU_,
           parent_ray_sensor_->GetVerticalRangeCount());
#endif
}

void ROSLaserPlugin::ConnectCb() {
  boost::lock_guard<boost::mutex> lock(lock_);
  if (pub_.getNumSubscribers()) {
    if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                     &ROSLaserPlugin::OnScan, this);
#else
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
                                     &ROSLaserPlugin::OnScan, this);
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

double ROSLaserPlugin::ConstrainAngle(double angle_rad) {
  angle_rad = fmod(angle_rad + M_PI, M2_PI);
  if (angle_rad < 0) angle_rad += M2_PI;
  return angle_rad - M_PI;
}

void ROSLaserPlugin::OnScan(const ConstLaserScanStampedPtr& _msg) {
  const double rotational_angle = ConstrainAngle(joint_->GetAngle(0).Radian());

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

  const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

  const math::Angle verticalMaxAngle =
      parent_ray_sensor_->GetVerticalAngleMax();
  const math::Angle verticalMinAngle =
      parent_ray_sensor_->GetVerticalAngleMin();
#endif
  const double vertical_diff =
      verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(6);
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
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "rotational_angle";
  msg.fields[5].offset = 24;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;
  msg.data.resize(verticalRangeCount * POINT_STEP);

  uint8_t* ptr = msg.data.data();
  for (int j = 0; j < verticalRangeCount; j++) {
    // Range
    double r = _msg->scan().ranges(j);
    if ((MIN_RANGE >= r) || (r >= MAX_RANGE)) {
      continue;
    }

    // Noise
    if (gaussian_noise_ != 0.0) {
      r += GaussianKernel(0, gaussian_noise_);
    }

    // Intensity
    double intensity = _msg->scan().intensities(j);

    // Get angle to compute xyz for each point
    double vertical_angle;

    if (verticalRayCount > 1) {
      vertical_angle = j * vertical_diff / (verticalRangeCount - 1) +
                       verticalMinAngle.Radian();
    } else {
      vertical_angle = verticalMinAngle.Radian();
    }

    // vertical_angle is rotated by rotational_angle:
    if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
      *((float*)(ptr + 0)) = r * cos(vertical_angle) * cos(rotational_angle);
      *((float*)(ptr + 4)) = r * cos(vertical_angle) * sin(rotational_angle);
#if GAZEBO_MAJOR_VERSION > 2
      *((float*)(ptr + 8)) = r * sin(vertical_angle);
#else
      *((float*)(ptr + 8)) = -r * sin(vertical_angle);
#endif
      *((float*)(ptr + 16)) = intensity;
#if GAZEBO_MAJOR_VERSION > 2
      *((uint16_t*)(ptr + 20)) = j;  // ring
#else
      *((uint16_t*)(ptr + 20)) = verticalRangeCount - 1 - j;  // ring
#endif
      *((float*)(ptr + 24)) = rotational_angle;
      ptr += POINT_STEP;
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

void ROSLaserPlugin::LaserQueueThread() {
  while (nh_->ok()) {
    laser_queue_.callAvailable(ros::WallDuration(0.01));
  }
}
double ROSLaserPlugin::GaussianKernel(double mu, double sigma) {
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
GZ_REGISTER_SENSOR_PLUGIN(ROSLaserPlugin)

}  // namespace gazebo
