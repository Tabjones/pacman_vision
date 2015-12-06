#include <pacman_vision/common.h>
#include <pacman_vision/estimator.hpp>

int main (int argc, char *argv[])
{
    std::string node_namespace ("pacman_vision");
    ros::init(argc, argv, node_namespace);
    ros::Time::init();
    std::shared_ptr<Storage> stor;
    Estimator module(node_namespace, "estimator", stor, 10);
    ros::Rate main_rate(30);
    module.spawn();
    while (ros::ok())
    {
        ros::spinOnce();
        main_rate.sleep();
    }
    return 0;
}
