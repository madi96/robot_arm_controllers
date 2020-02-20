/**
  08-11-2018- Dream-Project
  Purpose: Provides primitives to move a robot's arm group ,for the babbling experiment.

  @author Oussama YAAKOUBI
  @version
*/

#include <primitive.hpp>

class JoystickPrimitive: public Primitive {
    public:

        void setApproachPoint(Eigen::Vector3d& approach_point) {

            double u = 2 * (rand() / (RAND_MAX + 1.0)) - 1;
            double theta = M_PI/4 * u;
            double x = 0;
            double y = approach_radius_*sin(theta);
            double z = approach_radius_*cos(theta);
            approach_point(0) = x;
            approach_point(1) = y;
            approach_point(2) = z;
        }
};

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

int main(int argc, char** argv) {
    std::string node_name;
    node_name = parse_arg(argc, argv, "push_primitive_node");
    ros::init(argc, argv, node_name);

    PushPrimitive controller;

    ROS_INFO_STREAM("CONTROLLER:: NameSpace: "<<controller.nh.getNamespace());

    ROS_INFO_STREAM("CONTROLLER:: Robot controller ready !");
    while (ros::ok()) {
        controller.execute();
        usleep(1000);
        ros::spinOnce();
    }
    return 0;
}
