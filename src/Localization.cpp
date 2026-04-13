#include "Localization.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


//LocalizationNode::LocalizationNode() : rclcpp::Node("localization_node")
LocalizationNode::LocalizationNode() : rclcpp::Node("localization_node"), last_time_(this->get_clock()->now()) 
{
    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
	    "/joint_states", 
	    10, //velikost fronty
	    std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
	);
    // add code here

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    // add code here

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
    
    //inicializace polohy robota
    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.;
    odometry_.pose.pose.position.z = 0.1;

    //theta = 0;
    
    //publikace puvodni polohy
    publishOdometry();
    publishTransform();
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here

	//RCLCPP_INFO(this->get_logger(), "CallbackCalled");
	//rychlosti
	double left_wheel = msg.velocity[1];
	double right_wheel = msg.velocity[0];
	
	//dts
	auto current_time_ = this->get_clock()->now();
	double dt = (current_time_ - last_time_).seconds();
	
	//odometrie
	updateOdometry(left_wheel, right_wheel, dt);
	odometry_.header.stamp = current_time_; //cas publikace
	
	//publikace
	publishOdometry();
	publishTransform();
	//novy cas
	last_time_ = current_time_;
	
	
    // ********
    // * Help *
    // ********
    /*
    auto current_time = this->get_clock()->now();

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    */
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here
    
    //rychlost kol
    double v_left = left_wheel_vel * robot_config::WHEEL_RADIUS;
    double v_right = right_wheel_vel * robot_config::WHEEL_RADIUS;
    
    //rychlost robota
    double linear = (v_right + v_left) / 2.0; //linearni posuv
    double w = (v_right - v_left) / (2*robot_config::HALF_DISTANCE_BETWEEN_WHEELS); //uhlova rychlost robota
    
    //rotace do kvaternionu
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat); //dekodovani ze zpravy do tf_quat
    double roll, pitch, theta;
    //naklony robota, roll a pitch nepotrebujem
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta); 
   
    
    //prirustek polohy
    double delta_x = linear * std::cos(theta) * dt;
    double delta_y = linear * std::sin(theta) * dt;
    //prirustek uhlu
    double delta_theta = w * dt;


    //integrace: rychlost -> poloha 
    //prirustek pricteme k poloze a dame do zpravy
    odometry_.pose.pose.position.x += delta_x;
    odometry_.pose.pose.position.y += delta_y;
    theta += delta_theta;
    
     //normalizace do uhlu od -pi do +pi
    theta = std::atan2(std::sin(theta), std::cos(theta));
    
    if (!std::isfinite(theta)) {
    	theta = 0.0;
    }
    //zprava, uhel
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    
    if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w())) {
    	q.setRPY(0, 0, 0);
    }
    
    odometry_.pose.pose.orientation = tf2::toMsg(q); //dame uhel do zpravy, v kvaternionu

    //vypoctene rychlosti z odometrie
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = w;
}

void LocalizationNode::publishOdometry() {
    // add code here
    //RCLCPP_INFO(this->get_logger(), "Publish data");
    //odometry_.header.stamp = this->get_clock()->now(); //cas publikace
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    // add code here
    
    //toto bude zprava
    geometry_msgs::msg::TransformStamped t;

    //stamp, ids
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map"; // map
    t.child_frame_id = "base_link";   // base_link

    //pozice x, y, uhel
    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0; 
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
    
    // ********
    // * Help *
    // ********
    //tf_broadcaster_->sendTransform(t);
}
