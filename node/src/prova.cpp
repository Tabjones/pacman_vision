//TODO:
//1) Rename pose scanner to in-hand-scanner, and modify it
//1)when tracker re-finds object in scene: make use of hand-obj rel trans
//  as a starting condition
//2)Fill tracker service grasp verification, also design a service that fills
//  which hand is grasping
//3)Make a separated thread for Kinect2Processor ?! (wait until libfreenect2 is more developed)

#include <pacman_vision/common.h>
#include <pacman_vision/base_ros_node_helpers.hpp>

class M : public Module
{
    public:
        M()=delete;
        M(const ros::NodeHandle &n, const std::string ns, std::shared_ptr<Storage> &stor, const ros::Rate &rate):
            Module(n,ns,stor,rate){ std::cout<<"M created\n";}
        virtual void spin()
        {
            while(nh.ok())
            {
                spinOnce();
                spin_rate.sleep();
                std::cout<<" spin ";
            }
        }
        typedef std::shared_ptr<M> Ptr;
};

class Node : public MasterNode
{
    public:
    Node()=delete;
    Node(std::string ns, ros::Rate rate):MasterNode(ns,rate)
    {}
    virtual void checkModules()
    {
        std::string child("module");
        M::Ptr mod(std::make_shared<M>(nh, child, storage, ros::Rate(5)));
        module_drivers->spawn(mod);
    }
    void killMod()
    {
        module_drivers->kill("module");
    }
};
int main (int argc, char *argv[])
{
    std::string ns("master");
    ros::init(argc, argv, ns);
    ros::Time::init();
    Node node(ns,10);
    int count(0);
    while (node.ok())
    {
        node.spinOnce();
        node.sleep();
        std::cout<<" main ";
        ++count;
        if (count == 100)
        {
            node.checkModules();
        }
        else if (count == 300)
        {
            node.killMod();
        }
    }
    return 0;
}
