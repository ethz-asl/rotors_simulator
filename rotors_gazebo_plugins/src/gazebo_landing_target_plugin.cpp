#include <functional>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace gazebo
{

struct GazeboLandingTargetPlugin : ModelPlugin
{
    // Gazebo
    physics::ModelPtr model_;
    event::ConnectionPtr conn_;
    sdf::ElementPtr sdf_;
    double prev_sim_time_ = 0; // [seconds]

    // ROS
    std::thread ros_th_;
    ros::NodeHandle *ros_nh_;
    ros::Subscriber pos_sub_;
    ros::Subscriber vel_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    size_t ros_seq_ = 0;

    GazeboLandingTargetPlugin() {}
    ~GazeboLandingTargetPlugin() { delete ros_nh_; }

    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        // Gazebo things
        model_ = parent;
        sdf_ = sdf;
        conn_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboLandingTargetPlugin::on_update, this));
        prev_sim_time_ = model_->GetWorld()->GetSimTime().Double();

        // Start ROS thread
        ros_th_ = std::thread(&GazeboLandingTargetPlugin::ros_thread, this);
        sleep(1);
    }

    void ros_thread()
    {
        // Initialize ros node
        if (ros::isInitialized() == false)
        {
            ROS_FATAL("ROS is not initialized!");
        }
        ros_nh_ = new ros::NodeHandle();

        // Register pos subscriber
        {
            std::string topic_name = "landing_target/set_position";
            if (sdf_->HasElement("set_position_topic"))
            {
                topic_name = sdf_->Get<std::string>("set_position_topic");
            }
            pos_sub_ = ros_nh_->subscribe(topic_name,
                                          1,
                                          &GazeboLandingTargetPlugin::position_callback,
                                          this);
        }

        // Register vel subscriber
        {
            std::string topic_name = "landing_target/set_velocity";
            if (sdf_->HasElement("set_velocity_topic"))
            {
                topic_name = sdf_->Get<std::string>("set_velocity_topic");
            }
            vel_sub_ = ros_nh_->subscribe(topic_name,
                                          1,
                                          &GazeboLandingTargetPlugin::velocity_callback,
                                          this);
        }

        // Register pose publisher
        {
            std::string topic_name = "landing_target/pose";
            if (sdf_->HasElement("pose_topic"))
            {
                topic_name = sdf_->Get<std::string>("pose_topic");
            }
            pose_pub_ = ros_nh_->advertise<geometry_msgs::PoseStamped>(topic_name, 1);
        }

        // Register twist publisher
        {
            std::string topic_name = "landing_target/twist";
            if (sdf_->HasElement("twist_topic"))
            {
                topic_name = sdf_->Get<std::string>("twist_topic");
            }
            twist_pub_ =
                ros_nh_->advertise<geometry_msgs::TwistStamped>(topic_name, 1);
        }
    }

    void on_update()
    {
        publish_pose();
        publish_twist();
        ros_seq_++;
    }

    void position_callback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        math::Pose T_WR = model_->GetWorldPose();
        math::Vector3 pos{msg->x, msg->y, msg->z};
        T_WR.pos = pos;
        model_->SetWorldPose(T_WR);
    }

    void velocity_callback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        math::Vector3 vel{msg->x, msg->y, msg->z};
        model_->SetLinearVel(vel);
    }

    void publish_pose()
    {
        const double time = model_->GetWorld()->GetSimTime().Double();
        const math::Pose T_WR = model_->GetWorldPose();
        const math::Vector3 r_WR = T_WR.pos;
        const math::Quaternion q_WR = T_WR.rot;

        geometry_msgs::PoseStamped msg;
        msg.header.seq = ros_seq_;
        msg.header.stamp = ros::Time(time);
        msg.header.frame_id = "world";
        msg.pose.position.x = r_WR.x;
        msg.pose.position.y = r_WR.y;
        msg.pose.position.z = r_WR.z;
        msg.pose.orientation.w = q_WR.w;
        msg.pose.orientation.x = q_WR.x;
        msg.pose.orientation.y = q_WR.y;
        msg.pose.orientation.z = q_WR.z;

        pose_pub_.publish(msg);
    }

    void publish_twist()
    {
        const double time = model_->GetWorld()->GetSimTime().Double();
        const math::Vector3 linear_velocity = model_->GetWorldLinearVel();
        const math::Pose T_WR = model_->GetWorldPose();
        const math::Vector3 angular_velocity = model_->GetWorldAngularVel();

        geometry_msgs::TwistStamped msg;
        msg.header.seq = ros_seq_;
        msg.header.stamp = ros::Time(time);
        msg.header.frame_id = "world";
        msg.twist.linear.x = linear_velocity.x;
        msg.twist.linear.y = linear_velocity.y;
        msg.twist.linear.z = linear_velocity.z;
        msg.twist.angular.x = angular_velocity.x;
        msg.twist.angular.y = angular_velocity.y;
        msg.twist.angular.z = angular_velocity.z;

        twist_pub_.publish(msg);
    }
};

GZ_REGISTER_MODEL_PLUGIN(GazeboLandingTargetPlugin)
} // namespace gazebo
