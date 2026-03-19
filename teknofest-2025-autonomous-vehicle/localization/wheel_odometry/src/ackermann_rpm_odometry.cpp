#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>

#include <smart_can_msgs/FB_MotorDriver.h>
#include <smart_can_msgs/FeedbackSteeringAngle.h>

class AckermannOdometryPublisher {
private:
    ros::NodeHandle nh_;
    
    ros::Publisher odom_pub_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber steering_sub_;

    const double WHEEL_DIAMETER_M = 0.51;
    const double WHEEL_BASE = 1.84;
    const double TIMEOUT_SEC = 0.5;

    double current_rpm_;
    double current_steering_deg_;
    ros::Time last_rpm_time_;
    ros::Time last_steering_time_;

    double x_, y_, theta_;
    ros::Time last_time_;

public:
    AckermannOdometryPublisher() : 
        x_(0.0), y_(0.0), theta_(0.0), 
        current_rpm_(0.0), current_steering_deg_(0.0) 
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/wheel_odometry", 10);

        rpm_sub_ = nh_.subscribe("/beemobs/FB_MotorDriver", 10, &AckermannOdometryPublisher::rpmCallback, this);
        steering_sub_ = nh_.subscribe("/beemobs/FeedbackSteeringAngle", 10, &AckermannOdometryPublisher::steeringCallback, this);

        last_time_ = ros::Time::now();
        last_rpm_time_ = ros::Time::now();
        last_steering_time_ = ros::Time::now();

        ROS_INFO("Ackermann Odometry Node Initialized (C++).");
    }

    double rpmToMps(double rpm) {
        return (M_PI * WHEEL_DIAMETER_M * rpm) / 60.0;
    }

    void rpmCallback(const smart_can_msgs::FB_MotorDriver::ConstPtr& msg) {
        current_rpm_ = msg->VehicleRPM;
        last_rpm_time_ = ros::Time::now();
    }

    void steeringCallback(const smart_can_msgs::FeedbackSteeringAngle::ConstPtr& msg) {
        current_steering_deg_ = msg->FeedBackSteeringAngle;
        last_steering_time_ = ros::Time::now();
    }

    void checkTimeouts(ros::Time current_time) {
        if ((current_time - last_rpm_time_).toSec() > TIMEOUT_SEC) {
            current_rpm_ = 0.0;
        }
        if ((current_time - last_steering_time_).toSec() > TIMEOUT_SEC) {
            current_steering_deg_ = 0.0;
        }
    }

    boost::array<double, 36> getPoseCovariance() {
        boost::array<double, 36> cov = {0.0};
        cov[0]  = 0.1;   // x
        cov[7]  = 0.1;   // y
        cov[14] = 1e-9;  // z
        cov[21] = 1e-9;  // roll
        cov[28] = 1e-9;  // pitch
        cov[35] = 0.5;   // yaw 
        return cov;
    }

    boost::array<double, 36> getTwistCovariance() {
        boost::array<double, 36> cov = {0.0};
        cov[0]  = 0.05;  // linear x
        cov[7]  = 1e-9;  // linear y
        cov[14] = 1e-9;  // linear z
        cov[21] = 1e-9;  // angular x
        cov[28] = 1e-9;  // angular y
        cov[35] = 0.5;   // angular z
        return cov;
    }

    void run() {
        ros::Rate rate(20); 

        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time_).toSec();

            checkTimeouts(current_time);

            double v_x = rpmToMps(current_rpm_);
            double v_y = 0.0;
            double v_yaw = 0.0;

            double steering_rad = -current_steering_deg_ * (M_PI / 180.0);

            if (std::abs(steering_rad) > 1e-4) {
                double turning_radius = WHEEL_BASE / std::tan(steering_rad);
                v_yaw = v_x / turning_radius;
            }

            x_ += v_x * std::cos(theta_) * dt;
            y_ += v_x * std::sin(theta_) * dt;
            theta_ += v_yaw * dt;

            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "wheel_odom";
            odom.child_frame_id = "base_link";

            
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            odom.pose.covariance = getPoseCovariance();

            odom.twist.twist.linear.x = v_x;
            odom.twist.twist.linear.y = v_y;
            odom.twist.twist.angular.z = v_yaw;
            odom.twist.covariance = getTwistCovariance();

            odom_pub_.publish(odom);

            last_time_ = current_time;
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_odometry_publisher");
    AckermannOdometryPublisher node;
    node.run();
    return 0;
}
