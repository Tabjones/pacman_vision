#include <pacman_vision/module_config.h>
#include <pacman_vision/basic_node.hpp>
#include <pacman_vision/sensor_processor.hpp>
#include <pacman_vision/estimator.hpp>

int main (int argc, char *argv[])
{
    std::string node_namespace ("pacman_vision");
    ros::init(argc, argv, node_namespace);
    ros::Time::init();
    std::shared_ptr<Storage> storage (new Storage);
    BasicNode node(node_namespace, storage, 50);
    SensorProcessor sensor(node_namespace, "sensor_processor", storage, 50);
    Estimator estimator(node_namespace, "estimator", storage, 2);
    //node and sensor always spawn
    node.spawn();
    sensor.spawn();
    while (ros::ok())
    {
        //no op until GUI is here
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
