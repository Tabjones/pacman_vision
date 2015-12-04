#ifndef _BASE_ROS_NODE_HELPERS_HPP_
#define _BASE_ROS_NODE_HELPERS_HPP_
//this header is hpp cause it also contains implementations

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <pacman_vision/common.h>
#include <pacman_vision/storage.h>

///Base class module, should be inherited by modules in CRPT
template<typename Derived>
class Module
{
    public:
        //no empty module creation
        Module()=delete;
        //ctor with node handle of masternode, namespace, pointer to storage and desired rate
        Module(const ros::NodeHandle n, const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate):
            is_running(false), spin_rate(rate)
        {
            nh = ros::NodeHandle (n, ns);
            storage=stor;
            name_space = ns;
            queue_ptr.reset(new ros::CallbackQueue);
            nh.setCallbackQueue(&(*queue_ptr));
        }
        virtual ~Module(){}
        //general spinOnce
        void spinOnce()
        {
            derived().queue_ptr->callAvailable(ros::WallDuration(0));
        }
        bool inline isRunning() const
        {
            return derived().is_running;
        }
        inline std::string getName() const
        {
            return derived().name_space;
        }
        //spawn with spin implemented here
        void default_spawn()
        {
            //subsequent calls to spawn are prohibited
            if (isRunning()){
                ROS_WARN("[%s]\tTried to spawn %s, but it is already spawned and running.",__func__,getName().c_str());
                return;
            }
            derived().is_running = true;
            derived().worker = std::thread(&Module::spin, this);
        }
        void kill()
        {
            if (!isRunning()){
                ROS_WARN("[%s]\tTried to kill %s, but it is not running.",__func__,getName().c_str());
                return;
            }
            derived().is_running = false;
            derived().worker.join();
        }
        //spawn with spin implemented in Derived
        void spawn ()
        {
            if(isRunning()){
                ROS_WARN("[%s]\tTried to spawn %s, but it is already spawned and running.",__func__,getName().c_str());
                return;
            }
            derived().is_running = true;
            derived().worker = std::thread(&Derived::spin, derived_ptr());
        }
    protected:
        bool is_running;
        ros::NodeHandle nh;
        ros::Rate spin_rate;
        std::string name_space;
        std::shared_ptr<Storage> storage;
        std::shared_ptr<ros::CallbackQueue> queue_ptr;
        std::thread worker;
    private:
        //return derived ref
        Derived& derived() { return *static_cast<Derived*>(this); }
        const Derived& derived() const { return *static_cast<const Derived*>(this); }
        //return derived_ptr
        Derived* derived_ptr() {return static_cast<Derived*>(this); }
        const Derived* derived_ptr() const {return static_cast<const Derived*>(this); }
        //general spin
        void spin()
        {
            while(derived().nh.ok() && derived().is_running)
            {
                spinOnce();
                derived().spin_rate.sleep();
            }
        }
};

/**\brief Class MasterNode
 * {:Brief Base Class implementing a ROS master node that uses Modules with
 * separate threads.}
*/
class MasterNode
{
    public:
        ///empty obj creation is not allowed
        MasterNode()=delete;
        ///Only this ctor is allowed
        MasterNode(const std::string ns, const ros::Rate rate) : spin_rate(rate), name_space(ns)
        {
            nh = ros::NodeHandle(ns);
            storage = std::make_shared<Storage>();
        }
        ///Destructor
        virtual ~MasterNode(){}
        ///Main loop spin, crude implementation
        virtual void spin()
        {
            while (ok())
            {
                spinOnce();
                sleep();
            }
        }
        ///Custom spin method
        virtual void spinOnce()
        {
            ros::spinOnce();
        }
        ///wait for rate
        virtual void sleep()
        {
            spin_rate.sleep();
        }
        //Check OK-ness
        virtual inline bool ok() const
        {
            return nh.ok();
        }
        inline std::shared_ptr<Storage> getStorage() const
        {
            return storage;
        }
        inline ros::NodeHandle getNodeHandle() const
        {
            return nh;
        }
        inline std::string getName() const
        {
            return name_space;
        }
    protected:
        ros::NodeHandle nh;
        ros::Rate spin_rate;
        std::string name_space;
        std::shared_ptr<Storage> storage;
};
#endif // _BASE_ROS_NODE_HELPERS_HPP_

/** Use this as:
 *
 * Module Foo
class Foo: public Module<Foo>
{
    public:
        friend class Module<Foo>;
        Foo ()=delete;
        Foo (const ros::NodeHandle n, const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate):
            Module<Foo>(n,ns,stor,rate)
        {}
    private:
        void spin()
        {
            std::cout<<"Foo start\n";
            while(nh.ok() && is_running)
            {
                std::cout<<"Foo spinning\n";
                spinOnce();
                spin_rate.sleep();
            }
            std::cout<<"Foo stop\n";
        }
};

* Master node
class Master: public MasterNode
{
    public:
        Master()=delete;
        Master(const std::string ns, const ros::Rate rate): MasterNode(ns,rate)
        {}
        virtual ~Master(){}
};

* main
int main (int argc, char *argv[])
{
    std::string ns("master");
    ros::init(argc, argv, ns);
    ros::Time::init();
    Master node(ns,20);
    Foo module(ns,"module",node.getStorage(), 40);
    int count(0);
    while (node.ok())
    {
        std::cout<<"Master spinning\n";
        node.spinOnce();
        node.sleep();
        ++count;
        if (count == 100)
        {
            module.spawn();
            //start spamming Foo spinning at double rate
        }
        if (count == 200)
        {
            module.kill();
            //stop spamming Foo spinning at double rate
        }
    }
    return 0;
}
*/

