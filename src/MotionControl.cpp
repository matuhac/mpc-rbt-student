#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>

double normalizeAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
    if(angle == -M_PI) {angle = M_PI;}
    return angle;
}

MotionControlNode::MotionControlNode() :
	rclcpp::Node("motion_control_node") {
    
        uint32_t waypoint_pointer = 0;
        double col_threshold = 0.3;
        bool collisionDetected = false;
        
        double goal_threshold = 0.25;
        bool plan_received = 0;
        
        double prev_distance_err = 0.0;

        double x_vel = 0.0;
        
        double current_time = 0.0;
        double prev_time = 0.0;
        double dt = 0.0;
        
        
        //pure pursuit
        double lookahead_distance_ = 0.3;
	double max_linear_speed_ = 0.2;
	double max_angular_speed_ = 1.5;
	double min_linear_speed_ = 0.00;
	
	bool found = 0;

        // Subscribers for odometry and laser scans
	odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/odom", 
		10, //velikost fronty
		std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1)
		);
	    
	lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"/tiago_base/Hokuyo_URG_04LX_UG01", //mozna spatne
		10, //velikost fronty
		std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1)
		);
        
        // Publisher for robot control
        twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Client for path planning
	plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

        // Action server
	nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
			this,
			"/go_to_goal",

			std::bind(&MotionControlNode::navHandleGoal, this,
			  std::placeholders::_1, std::placeholders::_2),

			std::bind(&MotionControlNode::navHandleCancel, this,
			  std::placeholders::_1),

			std::bind(&MotionControlNode::navHandleAccepted, this,
			  std::placeholders::_1)
	);

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        
	while (!plan_client_->wait_for_service(std::chrono::seconds(5))) {
		RCLCPP_WARN(this->get_logger(), "Waiting for planner");
	}
	
    }

void MotionControlNode::checkCollision() {
	//projdi vsechny body laser scanu a zkontroluj jestli vzdalenost je mensi nez threhold
	
	//RCLCPP_INFO(this->get_logger(), "Col. check"); //funguje
	
	if (laser_scan_.ranges.empty()) {
		//RCLCPP_INFO(this->get_logger(), "Lidar empty");
		return;
	}
	
	int laser_range = 25;
	int center = laser_scan_.ranges.size()/2;
	col_threshold = 0.35;
	
	for (int i = center - laser_range; i < center + laser_range; i++) {
		if (i < 0 || i >= (int)laser_scan_.ranges.size()) {continue;}
		if (laser_scan_.ranges[i] < col_threshold){
			collisionDetected = true;
			RCLCPP_INFO(this->get_logger(), "Collision detected");
			//RCLCPP_INFO(this->get_logger(), "range: %f", laser_scan_.ranges[i]);
		}
	
	}
	/*
	for(auto range : laser_scan_.ranges){
		if (range < col_threshold) {
			//kolize detekovana
			collisionDetected = true;
			RCLCPP_INFO(this->get_logger(), "range: %f", range);
			//RCLCPP_INFO(this->get_logger(), "Collision detected");
		}
	}
	*/
	//kolize detekovana, zastavit robota
	if(collisionDetected){
		geometry_msgs::msg::Twist stop;
		stop.linear.x = 0;
		stop.angular.z = 0;
		twist_publisher_->publish(stop);	
	}
}

bool MotionControlNode::getLookaheadPoint(geometry_msgs::msg::PoseStamped & lookahead) {
	//RCLCPP_INFO(this->get_logger(), "size: %d", path_.poses.size());
    for (size_t i = waypoint_pointer; i < path_.poses.size(); i++) {

        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;

        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist >= lookahead_distance_) {
            lookahead = path_.poses[i];
            waypoint_pointer = i;
            //RCLCPP_INFO(this->get_logger(), "way_pointer: %d", waypoint_pointer);
            return true;
        }
    }

    size_t i = path_.poses.size();
    lookahead = path_.poses[i-1];
    waypoint_pointer = i;
    //RCLCPP_INFO(this->get_logger(), "way_pointer: %d, size: %d", waypoint_pointer, i);
    
    //RCLCPP_INFO(this->get_logger(), "way_pointer: %d", waypoint_pointer);
    return true;
}


void MotionControlNode::transformToRobotFrame(const geometry_msgs::msg::PoseStamped & target, double & x_r, double & y_r) {
    double dx = target.pose.position.x - current_pose_.pose.position.x;
    double dy = target.pose.position.y - current_pose_.pose.position.y;
    
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    x_r =  cos(yaw)*dx + sin(yaw)*dy;
    y_r = -sin(yaw)*dx + cos(yaw)*dy;
}


void MotionControlNode::MotionControlNode::updateTwist() {
	//RCLCPP_INFO(this->get_logger(), "updateTwist"); //

	if (path_.poses.empty()) return;
	
	if(found){return;}
	
	const double Kp_lin = 2.5;
	const double Ti_lin = 1.0;
	const double Td_lin = 0.5;
	
	//RCLCPP_INFO(this->get_logger(), "updateTwist checks"); //

	geometry_msgs::msg::PoseStamped lookahead;
	geometry_msgs::msg::Twist twist;

	if (!getLookaheadPoint(lookahead)) return;
	
	//RCLCPP_INFO(this->get_logger(), "getLookAhead"); //

	double x_r, y_r;
	transformToRobotFrame(lookahead, x_r, y_r);
	

	//RCLCPP_INFO(this->get_logger(), "TransformToRobotFrame"); //

	double Ld = std::sqrt(x_r*x_r + y_r*y_r);
	//jsme blizko cili - zastavit
	if (Ld < 1e-3) {
		twist.linear.x = 0;
		twist.angular.z = 0;
		twist_publisher_->publish(twist);
	return;
	}

	//zakriveni trasy
	double kappa = 2.0 * y_r / (Ld * Ld);
	
	double dx = lookahead.pose.position.x - current_pose_.pose.position.x;
    	double dy = lookahead.pose.position.y - current_pose_.pose.position.y;
    	double distance_err = sqrt(dx*dx + dy*dy);
	
	
	//uhel o ktery se musime otocit
	double angle_error = atan2(y_r, x_r);
	
	while (angle_error > M_PI)  angle_error -= 2.0 * M_PI;
	while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
	
	double omega = 0;
	double v = 0;
	
	//cil je za nami
	if(x_r < 0){
		v = 0.0;
		omega = (angle_error > 0 ? 1 : -1) * max_angular_speed_;
	
	}
	else{
		double v_goal = 1.0 * distance_err;
		double v_curve = max_linear_speed_ / (1.0 + 2.0 * fabs(kappa));
		
		v = std::min(v_goal, v_curve);
		v = std::min(v, max_linear_speed_);

		//pokud je uhel moc velky, tak se jenom otacime
		//if (fabs(angle_error) > 0.5) {
		//	v = 0.0;
		//}

		//optimalizace rychlosti v zavislosti na uhlu pri 90 stupnich je nula
		 v *= std::cos(angle_error);

		//rychlost otaceni
		omega = v * kappa;
	}

	//limity
	v = std::max(v, 0.0);
	omega = std::max(std::min(omega, max_angular_speed_), -max_angular_speed_);
	
	//zprava
	twist.linear.x = v;
	twist.angular.z = omega;
	
	twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
	//pridat nejakou logiku na odmitnuti cile
	RCLCPP_INFO(this->get_logger(), "navHandleGoal Called");
	goal_pose_ = goal->pose;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here


	return rclcpp_action::CancelResponse::ACCEPT;
    // ********
    // * Help *
    // ********
    /*
    (void)goal_handle;
    ...
    return ...;
    */
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {

	RCLCPP_INFO(this->get_logger(), "Goal accepted");
	
	goal_handle_ = goal_handle;
	//start execute, vytvori novy thread, vytvori path callback
	//goal_handle_->execute();
	
	//zahaji thread s funkci execute
	std::thread(&MotionControlNode::execute, this).detach();
	//std::thread{std::bind(&MotionControlNode::execute, this)}.detach();

    
}

void MotionControlNode::execute() {
    	//pozadej o cestu, to ma byt v navHandleAccepted?
    	RCLCPP_INFO(this->get_logger(), "Execute called");
    	
	auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
	request->start = current_pose_;
	request->goal = goal_pose_;
	//send request - vrati cestu
	plan_client_->async_send_request(
		request,
		std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1)
	);
	
	rclcpp::Rate loop_rate(1); //
	waypoint_pointer = 1;
	lookahead_distance_ = 0.45;
	
	while(!plan_received){} //cekani dokud neprijde mapa
	
	auto target = path_.poses[waypoint_pointer];
	found = 0;
	
	while(rclcpp::ok()){
		
		//kontrola priblizeni k cili (waypointu)
		geometry_msgs::msg::PoseStamped lookahead;
		lookahead = path_.poses[waypoint_pointer-1];
		
		double x = lookahead.pose.position.x - current_pose_.pose.position.x;
		double y = lookahead.pose.position.y - current_pose_.pose.position.y;
		
		double dist = sqrt(x*x + y*y);
		
		RCLCPP_INFO(this->get_logger(), "Distance: %.4f, lookahead: %.4f", dist, lookahead_distance_);
		RCLCPP_INFO(this->get_logger(), "way_pointer: %d, size: %d", waypoint_pointer, path_.poses.size());
		
		if(waypoint_pointer >= path_.poses.size()) {
			auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
			goal_handle_->succeed(result);
			found = 1;
			
			geometry_msgs::msg::Twist twist;
			twist.linear.x = 0;
			twist.angular.z = 0;
			
			twist_publisher_->publish(twist);
			
			RCLCPP_INFO(this->get_logger(), "Goal Found 2");
			return;
		}
		/*
		//jsme uz dostatecne blizko k waypointu
		if(dist < lookahead_distance_){
			//kontrola zda soucasny waypoint je konecny cil
			if(waypoint_pointer >= path_.poses.size()){
			//if(target == goal_pose_){
				auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
				goal_handle_->succeed(result);
				RCLCPP_INFO(this->get_logger(), "Goal Found 1");
				found = 1;
				//cil nalezen
				break;
			}
		}
		*/
		
		if (goal_handle_->is_canceling()) {
			auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
			goal_handle_->canceled(result);
			geometry_msgs::msg::Twist twist;
			twist.linear.x = 0;
			twist.angular.z = 0;
			
			twist_publisher_->publish(twist);
			found = 1;
			RCLCPP_INFO(this->get_logger(), "Exec Return");
			return;
		}
		
		
		//RCLCPP_INFO(this->get_logger(), "Execute: sleep");
		loop_rate.sleep();
	}

}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {

	RCLCPP_INFO(this->get_logger(), "path Callback called");
	auto response = future.get();
	if(response) {
		//prijem naplanovane cesty
		
		if (response->plan.poses.empty()){
			RCLCPP_INFO(this->get_logger(), "Empty map");
			return;
		}
		
		RCLCPP_INFO(this->get_logger(), "Response received");
		path_ = response->plan;
		plan_received = 1;
		
		RCLCPP_INFO(this->get_logger(), "Velikost cesty: %d", sizeof(path_.poses.size()));
		
		//goal_handle_->execute();
		//std::thread(&MotionControlNode::execute, this).detach();
	}
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    //soucasna pozice z odometrie
        //RCLCPP_INFO(this->get_logger(), "odometry received"); //funguje
	current_pose_.pose = msg.pose.pose;
	
	current_time = rclcpp::Time(msg.header.stamp).seconds();
	dt = current_time - prev_time;
	prev_time = current_time;

	checkCollision();
	if(!collisionDetected){
		updateTwist();
	}
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    //prijem zpravy z lidaru
    //RCLCPP_INFO(this->get_logger(), "Lidar received");
    laser_scan_ = msg;
}
