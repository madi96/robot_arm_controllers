/**
  08-11-2018- Dream-Project
  Purpose: Provides primitives to move a robot's arm group ,for the babbling experiment.

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

    // get the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    // get the robot state
    robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));

    // For BAXTER
#ifdef ROBOT_IS_BAXTER
    // define left and right arms joints starting positions
    left_arm_joints_start_positions_ = {1.47, -0.67, -1.06, 1.42, 0.75, 1.24, 1.43};
    right_arm_joints_start_positions_ = {-1.52, -0.79, 1.17, 1.73, -0.74, 1.13, -1.21};
    // grippers link name
    left_gripper_link_name_ = "left_gripper";
    right_gripper_link_name_ = "right_gripper";
    // define base frame
    base_frame_ = "base";
#endif

    // For PR2
#ifdef ROBOT_IS_PR2
    // define left and right arms joints starting positions
    left_arm_joints_start_positions_ = {1.95, 0.21, 1.08, -1.8, 1.48, -1.34, -2.87};
    right_arm_joints_start_positions_ = {-2.00, 0.36, -1.44, -2.12, -1.90, -1.00, 0.32};
    // grippers link name
    left_gripper_link_name_ = "l_gripper_tool_frame";
    right_gripper_link_name_ = "r_gripper_tool_frame";
    // define base frame
    base_frame_ = "odom_combined";
#endif

    // initializing left and right arms
    left_arm_group_.reset(new  moveit::planning_interface::MoveGroupInterface(moveit::planning_interface::MoveGroupInterface::Options("left_arm",
                          moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION, nh)));
    right_arm_group_.reset(new moveit::planning_interface::MoveGroupInterface(moveit::planning_interface::MoveGroupInterface::Options("right_arm",
                           moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION, nh)));

    // setting planner id for both arms
    left_arm_group_->setPlannerId(static_cast<std::string>(planner_params_["planner_id"]));
    right_arm_group_->setPlannerId(static_cast<std::string>(planner_params_["planner_id"]));

    // setting planning time for both arms
    left_arm_group_->setPlanningTime(std::stod(planner_params_["planning_time"]));
    right_arm_group_->setPlanningTime(std::stod(planner_params_["planning_time"]));

    // setting approach radius: approach distance before thouching the target
    approach_radius_ = std::stod(planner_params_["approach_radius"]);
    
    goToStartingPosition("left");
    goToStartingPosition("right");
}

void Primitive::connectToRos() {
    //~ tf_subscriber_.reset(new ros::Subscriber(nh.subscribe("/tf", 10, &Primitive::tfCallback, this)));
    planning_scene_pub_.reset(new ros::Publisher(nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1)));
    planning_scene_serv_.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene", 1)));

    // For BAXTER
#ifdef ROBOT_IS_BAXTER
    right_gripper_pub_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", true);
    left_gripper_pub_  = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", true);
    nh.getParam("right_gripper_id", right_gripper_id_);
    nh.getParam("left_gripper_id", left_gripper_id_);
#endif

    // For PR2
#ifdef ROBOT_IS_PR2
    right_gripper_pub_ = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/r_gripper_controller/command", true);
    left_gripper_pub_  = nh.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", true);
#endif

    // getting planner params
    nh.getParam("X", x_);
    nh.getParam("Y", y_);
    nh.getParam("Z", z_);
    std::cout<<"z "<<z_<<std::endl;
    nh.getParam( "/planner_parameters", planner_params_);
    asyn_spinner_.reset(new ros::AsyncSpinner(1));
    asyn_spinner_->start();
}

void Primitive::defineApproachAndGoalFrames() {
	approach_pose_ = getTargetPose();
	// For PR2
	#ifdef ROBOT_IS_PR2
        goal_pose_.pose.position.x = approach_radius_ + kExtentionDistance;
	#endif
	// For BAXTER
	#ifdef ROBOT_IS_BAXTER
        goal_pose_.pose.position.z = approach_radius_ + kExtentionDistance;
	#endif
		
	}


geometry_msgs::PoseStamped Primitive::getTargetPose() {
    Eigen::Vector3d approach_point;
    setApproachPoint(approach_point);
    // vector between the approach point(origin of the approach frame) and the goal(origin of the goal frame)
    tf::Vector3 V(-approach_point(0), -approach_point(1), -approach_point(2));
    V.normalize();
    // projection of the vector v in the xy plane of the approch frame
    tf::Vector3 VP(V.getX(), V.getY(), 0);
    // compute the rotations to make the z axis of the approach frame oriented towards the origin of the goal frame
    double yaw = atan2(V.getY(), V.getX());

    // For PR2
#ifdef ROBOT_IS_PR2
    double pitch = atan2(VP.length(),V.getZ())- M_PI/2;
#endif
    // For BAXTER
#ifdef ROBOT_IS_BAXTER
    double pitch = atan2(VP.length(),V.getZ());
#endif
    tf::Quaternion q;
    q.setRPY(0, pitch, yaw);

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


void Primitive::setApproachPoint(Eigen::Vector3d& approach_point) {
	double u=rand() / (RAND_MAX + 1.0);
	double v=rand() / (RAND_MAX + 1.0);
    double theta = M_PI/2- (M_PI/3 * v);
    double phi = M_PI/2 -(M_PI/3 * u);
    double x = approach_radius_*sin(phi)*cos(theta);
    double y = approach_radius_*sin(phi)*sin(theta);
    double z = approach_radius_*cos(phi);
    approach_point(0) = x;
    approach_point(1) = y;
    approach_point(2) = z;
}

        //~ void Primitive::getApproachPoint(Eigen::Vector3d& approach_point) {

            //~ double u = 2 * (rand() / (RAND_MAX + 1.0)) - 1;
            //~ double theta = M_PI/4 * u;
            //~ double x = 0;
            //~ double y = approach_radius_*sin(theta);
            //~ double z = approach_radius_*cos(theta);
            //~ approach_point(0) = x;
            //~ approach_point(1) = y;
            //~ approach_point(2) = z;
        //~ }

bool Primitive::goToStartingPosition(std::string arm_group_name) {
    moveit::planning_interface::MoveGroupInterface::Plan go_to_starting_position_plan;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group;
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


// this function is used for enabling/disabling collision detection
void Primitive::manipulateOctomap(bool add_octomap_to_acm) {

    moveit_msgs::GetPlanningScene::Request ps_req;
    moveit_msgs::GetPlanningScene::Response ps_res;
    moveit_msgs::PlanningScene ps_msg;


    ps_req.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    // get the planning scene response
    planning_scene_serv_->call(ps_req, ps_res);
    if(add_octomap_to_acm) {
        ps_res.scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
        ps_res.scene.allowed_collision_matrix.default_entry_values.push_back(true);
    }
    else {
        ps_res.scene.allowed_collision_matrix.default_entry_names.clear();
        ps_res.scene.allowed_collision_matrix.default_entry_values.clear();
    }

    // publish planning scene message
    ps_msg.is_diff = true;
    ps_msg.allowed_collision_matrix = ps_res.scene.allowed_collision_matrix;
    planning_scene_pub_->publish(ps_msg);
}



bool Primitive::rotateWrist(moveit::planning_interface::MoveGroupInterface::Plan wrist_rotation_plan){

	//arm_group->setStartState(*arm_group->getCurrentState());
	std::vector< double >  joints_values = chosen_arm_group_->getCurrentJointValues ();
	joints_values[joints_values.size()-1] = wrist_rot_array_[rand_rotation_value_index_];
	chosen_arm_group_->setJointValueTarget(joints_values);
	return static_cast<bool>(chosen_arm_group_->execute(wrist_rotation_plan));
}

bool Primitive::reverseBackTrajectory(moveit::planning_interface::MoveGroupInterface::Plan& traj_plan) {

    moveit::planning_interface::MoveGroupInterface::Plan reverse_plan;
    reverse_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
    reverse_plan.trajectory_.joint_trajectory.header.frame_id = traj_plan.trajectory_.joint_trajectory.header.frame_id;
    reverse_plan.trajectory_.joint_trajectory.joint_names = traj_plan.trajectory_.joint_trajectory.joint_names;

    int j = traj_plan.trajectory_.joint_trajectory.points.size() - 1;
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    robot_trajectory::RobotTrajectory robot_traj(robot_model_, chosen_arm_group_name_);
    robot_state::RobotState r_state(robot_model_);
    for(size_t i = 0; i < traj_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++) {
        moveit::core::jointTrajPointToRobotState(traj_plan.trajectory_.joint_trajectory, j, r_state);
        robot_traj.insertWayPoint(i, r_state, 0.1);
        j--;
    }

    if(!time_param.computeTimeStamps(robot_traj))
        ROS_WARN("TEST : Time parametrization for the solution path failed.");
    robot_traj.getRobotTrajectoryMsg(reverse_plan.trajectory_);
    moveit::core::robotStateToRobotStateMsg(*chosen_arm_group_->getCurrentState(), reverse_plan.start_state_);
    return(static_cast<bool>(chosen_arm_group_->execute(reverse_plan)));

}

void Primitive::publishTransformToRobotFrame(std::vector<double> goal,
											 std::vector<double> goal_normal) {

    static tf::TransformBroadcaster br;
    tf::Transform goalToApprochtransform, finalToApproachTransform,
    goalToRobotTransform, approachToRobotTransform, extendedGoalToRobotTransform;
    tf::Quaternion q;

    //~ if (chosen_arm_group_name_ =="left_arm" )
    //~ q.setRPY(0,0,M_PI/2);
    //~ else if (chosen_arm_group_name_ =="right_arm" )
    //~ q.setRPY(0,0,-M_PI);
    //~ else q.setRPY(goal_normal_[0],goal_normal_[1],goal_normal_[2]);

    // set a transform from the goal frame to the robot frame
    q.setRPY(goal_normal[0],goal_normal[1],goal_normal[2]);

    goalToRobotTransform.setOrigin( tf::Vector3(goal[0], goal[1], goal[2]) );
    goalToRobotTransform.setRotation(q);

    defineApproachAndGoalFrames();

    // set a transform from the approach frame to the robot frame
    goalToApprochtransform.setOrigin( tf::Vector3(approach_pose_.pose.position.x, approach_pose_.pose.position.y, approach_pose_.pose.position.z) );
    q.setW(approach_pose_.pose.orientation.w);
    q.setX(approach_pose_.pose.orientation.x);
    q.setY(approach_pose_.pose.orientation.y);
    q.setZ(approach_pose_.pose.orientation.z);
    goalToApprochtransform.setRotation(q);
    approachToRobotTransform = goalToRobotTransform*goalToApprochtransform;

    transformed_approach_frame_.header.frame_id = base_frame_;
    transformed_approach_frame_.pose.position.x = approachToRobotTransform.getOrigin().getX() ;
    transformed_approach_frame_.pose.position.y = approachToRobotTransform.getOrigin().getY() ;
    transformed_approach_frame_.pose.position.z = approachToRobotTransform.getOrigin().getZ() ;
    transformed_approach_frame_.pose.orientation.w = approachToRobotTransform.getRotation().getW();
    transformed_approach_frame_.pose.orientation.x = approachToRobotTransform.getRotation().getX();
    transformed_approach_frame_.pose.orientation.y = approachToRobotTransform.getRotation().getY();
    transformed_approach_frame_.pose.orientation.z = approachToRobotTransform.getRotation().getZ();

    // set a transform from extented goal frame to the robot frame
    finalToApproachTransform.setOrigin( tf::Vector3(goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z) );
    q.setRPY(0, 0, 0);
    finalToApproachTransform.setRotation(q);
    extendedGoalToRobotTransform= approachToRobotTransform*finalToApproachTransform ;

    transformed_goal_frame_.header.frame_id = base_frame_;
    transformed_goal_frame_.pose.position.x = extendedGoalToRobotTransform.getOrigin().getX() ;
    transformed_goal_frame_.pose.position.y = extendedGoalToRobotTransform.getOrigin().getY() ;
    transformed_goal_frame_.pose.position.z = extendedGoalToRobotTransform.getOrigin().getZ() ;
    transformed_goal_frame_.pose.orientation.w = extendedGoalToRobotTransform.getRotation().getW();
    transformed_goal_frame_.pose.orientation.x = extendedGoalToRobotTransform.getRotation().getX();
    transformed_goal_frame_.pose.orientation.y = extendedGoalToRobotTransform.getRotation().getY();
    transformed_goal_frame_.pose.orientation.z = extendedGoalToRobotTransform.getRotation().getZ();

    // publish transforms
    br.sendTransform(tf::StampedTransform(goalToRobotTransform, ros::Time::now(), base_frame_, "goal_frame"));
    br.sendTransform(tf::StampedTransform(approachToRobotTransform, ros::Time::now(), base_frame_,"approach_frame"));
    br.sendTransform(tf::StampedTransform(extendedGoalToRobotTransform, ros::Time::now(), base_frame_, "final_goal_frame"));

}

bool Primitive::planForTrajectory(moveit::planning_interface::MoveGroupInterface::Plan& to_approach_point_plan,
                                  moveit::planning_interface::MoveGroupInterface::Plan& to_goal_plan,
                                  int iteration) {

    bool planning_result = false;
    double cartesian_path_success_threshold = approach_radius_ /(approach_radius_+ kExtentionDistance) ;
    robot_state::RobotStatePtr start_state_for_goal_trajectory;

    ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the selected approach point"<< " for the "<<chosen_arm_group_name_<< " Random frame number "<<iteration);
    chosen_arm_group_->clearPoseTargets();
    chosen_arm_group_->setStartState(*chosen_arm_group_->getCurrentState());
    chosen_arm_group_->setPoseTarget(transformed_approach_frame_);
    bool to_approach_point_plan_result = static_cast<bool>(chosen_arm_group_->plan(to_approach_point_plan));


    // plan a trajectory to the approach point
    if (to_approach_point_plan_result) {
        start_state_for_goal_trajectory = chosen_arm_group_->getCurrentState();
        ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the approach point"<< " for the "<<chosen_arm_group_name_);

        // set the starting position for the trajectory to the approach point
        moveit::core::jointTrajPointToRobotState(to_approach_point_plan.trajectory_.joint_trajectory,
                to_approach_point_plan.trajectory_.joint_trajectory.points.size() - 1,
                *start_state_for_goal_trajectory);

        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose selected_approach_pose;

        //make sure that the octomap is in the collision world
        manipulateOctomap(true);

        waypoints.push_back(transformed_approach_frame_.pose);
        waypoints.push_back(transformed_goal_frame_.pose);

        moveit_msgs::RobotTrajectory robot_trajectory;
        chosen_arm_group_->setStartState(*start_state_for_goal_trajectory);
        double fraction = chosen_arm_group_->computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);

        // compute the success threshold: a computed cartesian path is considered a success
        // if it return a faction greater than this threshold
        // cartesian_path_success_threshold = approach_radius_/(- kExtentionDistance + approach_radius_ );
        ROS_WARN_STREAM("CONTROLLER:: fraction "<<fraction<< " for the "<<chosen_arm_group_name_);
        if (fraction >= cartesian_path_success_threshold) {
            planning_result = true;
            to_goal_plan.trajectory_ = robot_trajectory;
        } else {
            ROS_WARN_STREAM("CONTROLLER:: Failed to find a straight line motion from the approach point to the goal "<< " for the "<<chosen_arm_group_name_);
        }

    } else {
        ROS_WARN_STREAM("CONTROLLER:: Failed to find a plan from the starting position to the approach point"<< " for the "<<chosen_arm_group_name_);
    }
    return planning_result;
}


bool Primitive::executePlannedMotion(moveit::planning_interface::MoveGroupInterface::Plan to_approach_point_plan,
                                     moveit::planning_interface::MoveGroupInterface::Plan to_goal_plan) {

    bool execution_result = false;

    ROS_INFO_STREAM("CONTROLLER:: Executing planned motion"<< " for the "<<chosen_arm_group_name_);

    // Execute plan: from starting position to the approach position
    bool to_approach_point_motion_result = static_cast<bool>(chosen_arm_group_->execute(to_approach_point_plan));
    ros::Duration(1).sleep();

    // Execute plan: from approach position to the specified goal
    moveit::core::robotStateToRobotStateMsg(*chosen_arm_group_->getCurrentState(), to_goal_plan.start_state_);
    bool to_goal_motion_result= static_cast<bool>(chosen_arm_group_->execute(to_goal_plan));

    // reverse back motion from approach position to the specified goal
    reverseBackTrajectory(to_goal_plan);

    // reverse back motion from starting position to the approach position
    reverseBackTrajectory(to_approach_point_plan);

    if (to_approach_point_motion_result && to_goal_motion_result) {
        execution_result = true;
    }
    
    return execution_result;
}


bool Primitive::planAndExecute(std::vector<double> goal,
                               std::vector<double> goal_normal) {

    moveit::planning_interface::MoveGroupInterface::Plan to_approach_point_plan, to_goal_plan;
    bool planning_result = false;
    bool execution_result = false;
    int  random_frame_budget = 10;
    int random_frame_count = 0;
    while(!planning_result && random_frame_count < random_frame_budget ) {
        publishTransformToRobotFrame(goal, goal_normal);
        planning_result = planForTrajectory(to_approach_point_plan,to_goal_plan, random_frame_count);
        random_frame_count +=1;
    }
    if (planning_result) {
        execution_result = executePlannedMotion(to_approach_point_plan, to_goal_plan);
        if (execution_result) {
            ROS_INFO_STREAM("CONTROLLER:: Planned motion executed successfully"<< " for the "<<chosen_arm_group_name_);
            //~ action_serv_->setSucceeded();
        } else {
            ROS_INFO_STREAM("CONTROLLER:: Planned motion execution FAILED"<< " for the "<<chosen_arm_group_name_);
            //~ action_serv_->setAborted();
        }
    }
    return planning_result;
}

void Primitive::execute() {

    std::string chosen_arm_group_name_;
    std::vector<double> goal, goal_normal;
    ros::Publisher gripper_pub;
    bool result = false;

    //make sure that the octomap is in the collision world
    manipulateOctomap(true);

    // put left and right arms in the starting positions
    goToStartingPosition("left");
    goToStartingPosition("right");

    // set the goal
    goal = {x_,y_,z_};
    goal_normal = {0,0,0};


    //always plan for the approach with the octomap in the obstacle domain
    manipulateOctomap(false);

    // Choose a random arm group
    chosen_arm_group_name_ = group_names_array_[rand()%2];

    ROS_INFO_STREAM("CONTROLLER:: chosen group name "<<chosen_arm_group_name_);

    // set the targeted arm joints group and its starting joints values
    if(chosen_arm_group_name_ == "left_arm") {
        ROS_INFO_STREAM("CONTROLLER:: chosen group name left_arm ");
        chosen_arm_group_ = left_arm_group_;
    } else {
        ROS_INFO_STREAM("CONTROLLER:: chosen group name right_arm ");
        chosen_arm_group_ = right_arm_group_;
    }

    // plan and execute for the the chosen arm
    result= planAndExecute(goal, goal_normal);
    if (! result) { // plan for the other arm
        ROS_INFO_STREAM("CONTROLLER:: planning for the other group, current group_name "<<chosen_arm_group_name_);
        if (chosen_arm_group_name_ == "left_arm") {
            ROS_INFO_STREAM("CONTROLLER:: second group name right_arm ");
            chosen_arm_group_name_ = "right_arm";
            chosen_arm_group_ = right_arm_group_;
        } else if (chosen_arm_group_name_ == "right_arm") {
            ROS_INFO_STREAM("CONTROLLER:: second group name left_arm ");
            chosen_arm_group_name_ = "left_arm";
            chosen_arm_group_ = left_arm_group_;
        }
        // plan and execute for the other arm if the chosen arm fails
        result= planAndExecute(goal, goal_normal);
        if (! result) {
            //~ action_serv_->setAborted();
            ROS_WARN_STREAM("CONTROLLER:: Failed to plan for the second arm: "<<chosen_arm_group_name_);
        }
    }
    ROS_WARN_STREAM("CONTROLLER:: ------------------------------");
}
