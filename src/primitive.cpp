/**
  08-11-2018- Dream-Project
  Purpose: Provides primitives to move a robot's arm group.

  @author Oussama YAAKOUBI
  @version
*/

#include <primitive.hpp>

Primitive::Primitive() {
    init();
}


void Primitive::init() {
    connectToRos();
    ros::Duration(1).sleep();
	#ifdef ROBOT
		std::cout<<"CONTROLLER:: ROBOT "<<ROBOT<<std::endl;
	#else
		std::cout<<"CONTROLLER:: ROBOT NONE"<<std::endl;
	#endif
	std::cout<<"CONTROLLER:: ROBOT "<<ROBOT<<std::endl;
    // get the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    // get the robot state
    robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));

    // define left and right arms joints home positions
    left_arm_joints_start_positions_ = {1.95, 0.21, 1.08, -1.8, 1.48, -1.34, -2.87};
    right_arm_joints_start_positions_ = {-2.00, 0.36, -1.44, -2.12, -1.90, -1.00, 0.32};

    // initializing left and right arms
    left_arm_group_.reset(new  moveit::planning_interface::MoveGroup(moveit::planning_interface::MoveGroup::Options("left_arm",
                          moveit::planning_interface::MoveGroup::ROBOT_DESCRIPTION, nh)));
    right_arm_group_.reset(new moveit::planning_interface::MoveGroup(moveit::planning_interface::MoveGroup::Options("right_arm",
                           moveit::planning_interface::MoveGroup::ROBOT_DESCRIPTION, nh)));

    // setting planner id for both arms
    left_arm_group_->setPlannerId(static_cast<std::string>(planner_params_["planner_id"]));
    right_arm_group_->setPlannerId(static_cast<std::string>(planner_params_["planner_id"]));

    // setting planning time for both arms
    left_arm_group_->setPlanningTime(std::stod(planner_params_["planning_time"]));
    right_arm_group_->setPlanningTime(std::stod(planner_params_["planning_time"]));

    // setting approach radius: approach distance before thouching the target
    approach_radius_ = std::stod(planner_params_["approach_radius"]);

}

void Primitive::connectToRos() {
    tf_subscriber_.reset(new ros::Subscriber(nh.subscribe("/tf", 10, &Primitive::tfCallback, this)));
    planning_scene_pub_.reset(new ros::Publisher(nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1)));
    planning_scene_serv_.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene", 1)));
    right_gripper_pub_ = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/r_gripper_controller/command", true);
    left_gripper_pub_  = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", true);

    // getting planner params
    nh.getParam("X", x_);
    nh.getParam("Y", y_);
    nh.getParam("Z", z_);
    std::cout<<"z "<<z_<<std::endl;
	std::cout<<"CONTROLLER:: ROBOT "<<ROBOT<<std::endl;
    nh.getParam( "/planner_parameters", planner_params_);
    nh.getParam("right_gripper_id", right_gripper_id_);
    nh.getParam("left_gripper_id", left_gripper_id_);
    asyn_spinner_.reset(new ros::AsyncSpinner(1));
    asyn_spinner_->start();
}

void Primitive::tfCallback(const tf2_msgs::TFMessageConstPtr& msg) {
    if(!goal_.empty() && !goal_normal_.empty()) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        //Set and publish a frame for the goal position
        transform.setOrigin( tf::Vector3(goal_[0], goal_[1], goal_[2]) );
        tf::Quaternion q;
        q.setRPY(goal_normal_[0], goal_normal_[1], goal_normal_[2]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "goal_frame"));


        //Set and publish a frame for all approach poses
        for(size_t i = 0; i < approach_frames_vector_.size(); i++) {
            transform.setOrigin( tf::Vector3(approach_frames_vector_[i].pose.position.x, approach_frames_vector_[i].pose.position.y, approach_frames_vector_[i].pose.position.z) );
            q.setW(approach_frames_vector_[i].pose.orientation.w);
            q.setX(approach_frames_vector_[i].pose.orientation.x);
            q.setY(approach_frames_vector_[i].pose.orientation.y);
            q.setZ(approach_frames_vector_[i].pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "goal_frame", "approach_frame" + std::to_string(i)));
        }

        if(!goal_frames_vector_.empty()) {
            for(size_t i = 0; i < goal_frames_vector_.size(); i++) {
                transform.setOrigin( tf::Vector3(goal_frames_vector_[i].pose.position.x, goal_frames_vector_[i].pose.position.y, goal_frames_vector_[i].pose.position.z) );
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "approach_frame" + std::to_string(i), "final_goal_frame" + std::to_string(i)));

            }
        }

    }
}


bool Primitive::transformFrames(std::vector<geometry_msgs::PoseStamped> poses_to_transform,
                     std::vector<geometry_msgs::PoseStamped>& output_transform,
                     std::string child_frame_name) {

    tf::StampedTransform transform;
    std::string base_frame = "odom_combined";
    std::string target_frame;

    output_transform.clear();
    //make sure the goal frame is at the current goal
    for(size_t jojo = 0; jojo < poses_to_transform.size(); jojo++) {
        target_frame = child_frame_name + std::to_string(jojo);
        base_frame = "odom_combined";
        try {
            tf_listener_.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        geometry_msgs::PoseStamped current_target;
        current_target.header.frame_id = base_frame;
        current_target.pose.position.x = transform.getOrigin().getX() ;
        current_target.pose.position.y = transform.getOrigin().getY() ;
        current_target.pose.position.z = transform.getOrigin().getZ() ;
        current_target.pose.orientation.w = transform.getRotation().getW();
        current_target.pose.orientation.x = transform.getRotation().getX();
        current_target.pose.orientation.y = transform.getRotation().getY();
        current_target.pose.orientation.z = transform.getRotation().getZ();
        output_transform.push_back(current_target);
    }

    if(output_transform.size() == poses_to_transform.size())
        return true;
    else
        return false;
}


std::vector<geometry_msgs::PoseStamped> Primitive::getRandomApproachFrames(int number_target) {
    std::vector<geometry_msgs::PoseStamped> targets;
    goal_frames_vector_.clear();
    for(int i = 0; i < number_target; i++) {
        geometry_msgs::PoseStamped new_approach_point = getTargetPose();
        geometry_msgs::PoseStamped extended_pose;
        targets.push_back(new_approach_point);
        extended_pose.pose.position.z += approach_radius_ + kExtentionDistance;
        goal_frames_vector_.push_back(extended_pose);
    }
    return targets;
}


geometry_msgs::PoseStamped Primitive::getTargetPose() {
    Eigen::Vector3d approach_point;
    getApproachPoint(approach_point);
    // vector between the approach point(origin of the approach frame) and the goal(origin of the goal frame)
    tf::Vector3 V(-approach_point(0), -approach_point(1), -approach_point(2));
    V.normalize();
    // projection of the vector v in the xy plane of the approch frame
    tf::Vector3 VP(V.getX(), V.getY(), 0);
    // compute the rotations to make the z axis of the approach frame oriented towards the origin of the goal frame
    double yaw = atan2(V.getY(), V.getX());
    double pitch = atan2(VP.length(),V.getZ());
    tf::Quaternion q;
    q.setRPY(0, pitch -M_PI/2, yaw);

    geometry_msgs::PoseStamped approach_pose;
    approach_pose.header.frame_id = "goal_frame";
    approach_pose.pose.position.x = approach_point(0);
    approach_pose.pose.position.y = approach_point(1);
    approach_pose.pose.position.z = approach_point(2);
    approach_pose.pose.orientation.w = q.w();
    approach_pose.pose.orientation.x = q.x();
    approach_pose.pose.orientation.y = q.y();
    approach_pose.pose.orientation.z = q.z();
    return approach_pose;
}


void Primitive::getApproachPoint(Eigen::Vector3d& approach_point) {
    double u = 2*(rand() / (RAND_MAX + 1.0)) - 1;
    double v =2* (rand() / (RAND_MAX + 1.0)) - 1;
    double theta = M_PI/3 * u;
    double phi = M_PI/3 * v;
    double x = approach_radius_*sin(phi)*cos(theta);
    double y = approach_radius_*sin(phi)*sin(theta);
    double z = approach_radius_*cos(phi);
    approach_point(0) = x;
    approach_point(1) = y;
    approach_point(2) = z;
}



bool Primitive::goToStartingPosition(std::string arm_group_name) {
    moveit::planning_interface::MoveGroup::Plan go_to_starting_position_plan;
    std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group;
    std::vector<double> arm_joints_start_positions;
    bool in_start_position =false;
    // set the targeted arm joints group and its starting joints values
    if (strcmp(arm_group_name.c_str(),"left")) {
        arm_group = left_arm_group_;
        arm_joints_start_positions = left_arm_joints_start_positions_;
    } else {
        arm_group = right_arm_group_;
        arm_joints_start_positions = right_arm_joints_start_positions_;
    }
    // set the starting and targeted joints values for the left arm
    arm_group->setStartState(*left_arm_group_->getCurrentState());
    arm_group->setJointValueTarget(arm_joints_start_positions);
    // if the planning succeded, execute
    if (arm_group->plan(go_to_starting_position_plan)) {
        ROS_INFO_STREAM("CONTROLLER:: The planner has found a plan to bring the left arm to starting position");
        in_start_position = static_cast<bool>(arm_group->execute(go_to_starting_position_plan));
    } else ROS_WARN_STREAM("CONTROLLER:: The planner has not found a plan to bring the left arm to starting position");

    if (in_start_position) ROS_INFO_STREAM("CONTROLLER:: "<< arm_group_name.c_str() << " arm in start position");
    else ROS_WARN_STREAM("CONTROLLER:: "<< arm_group_name.c_str() <<  " arm has FAILED to go to the starting position");
    return in_start_position;
}


bool Primitive::reverseBackTrajectory(moveit::planning_interface::MoveGroup::Plan& traj_plan, std::string arm_group_name,
                           std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group) {

    moveit::planning_interface::MoveGroup::Plan reverse_plan;
    reverse_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
    reverse_plan.trajectory_.joint_trajectory.header.frame_id = traj_plan.trajectory_.joint_trajectory.header.frame_id;
    reverse_plan.trajectory_.joint_trajectory.joint_names = traj_plan.trajectory_.joint_trajectory.joint_names;

    int j = traj_plan.trajectory_.joint_trajectory.points.size() - 1;
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    robot_trajectory::RobotTrajectory robot_traj(robot_model_, arm_group_name);
    robot_state::RobotState r_state(robot_model_);
    for(size_t i = 0; i < traj_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++) {
        moveit::core::jointTrajPointToRobotState(traj_plan.trajectory_.joint_trajectory, j, r_state);
        robot_traj.insertWayPoint(i, r_state, 0.1);
        j--;
    }

    if(!time_param.computeTimeStamps(robot_traj))
        ROS_WARN("TEST : Time parametrization for the solution path failed.");
    robot_traj.getRobotTrajectoryMsg(reverse_plan.trajectory_);
    moveit::core::robotStateToRobotStateMsg(*arm_group->getCurrentState(), reverse_plan.start_state_);
    return(static_cast<bool>(arm_group->execute(reverse_plan)));

}

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
    std::string key;
    std::string value;
    std::string temp_str;
    std::string::size_type res;

    key = "__name:=";
    for (unsigned short i = 0; i < argc; ++i) {
        temp_str = argv[i];
        res = temp_str.find(key);

        if (res != std::string::npos) {
            value = temp_str.erase(res, key.length());
            break;
        }
        else if (i == argc - 1) {
            value = default_val;
        }
    }
    return value;
}

int main(int argc, char** argv){
    std::string node_name;
    node_name = parse_arg(argc, argv, "push_button_primitive_node");
    ros::init(argc, argv, node_name);

    Primitive controller;

    ROS_INFO_STREAM("CONTROLLER:: NameSpace: "<<controller.nh.getNamespace());

    ROS_INFO_STREAM("CONTROLLER:: Robot controller ready !");
    while (ros::ok()) {
        controller.init();
        usleep(1000);
        ros::spinOnce();
    }
    return 0;
}
