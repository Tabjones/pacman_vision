#ifndef _DYNAMIC_MODULES_HPP_
#define _DYNAMIC_MODULES_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <common/common_ros.h>
#include <common/common_std.h>
#include <common/storage.h>

#include <thread>

namespace pacv
{
//The magic starts here
//
///Base  class for  dynamic  modules, should  be inherited  by  modules in  CRPT
//fashion. See the end of the file for an example usage!
template<typename Derived>
class Module
{
    //////Methods
    public:
        //no empty module creation allowed
        Module()=delete;
        //ctor with  node handle to  create named namespace, pointer  to storage
        //This is for master node
        Module(const std::string ns, const Storage::Ptr stor):
            is_running(false), disabled(false)
        {
            derived().storage=stor;
            derived().name = ns;
            //father top level node version
            derived().nh = std::make_shared<ros::NodeHandle>(name);
        }
        //ctor with  node handle of  masternode, namespace, pointer  to storagee
        //This is for modules, dependant of master node
        Module(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor):
            is_running(false), disabled(false)
        {
            // nh = std::make_shared<ros::NodeHandle>(n, ns);
            derived().storage=stor;
            derived().name = ns;
            //copy construct father node handle
            derived().father_nh = std::make_shared<ros::NodeHandle>(n);
            // nh->setCallbackQueue(&(*queue_ptr));
        }
        virtual ~Module()
        {
            derived().is_running=false;
            derived().worker.join();
            derived().nh->shutdown();
        }
        //set desired spin rate (mandatory call)
        void setRate(const double freq)
        {
            derived().spin_rate = std::make_shared<ros::Rate>(freq);
        }
        //general spinOnce
        void spinOnce()
        {
            derived().queue_ptr->callAvailable(ros::WallDuration(0));
        }
        bool inline isRunning() const
        {
            return derived().is_running;
        }
        bool inline isDisabled() const
        {
            return derived().disabled;
        }
        inline const std::string getNamespace() const
        {
            if(nh)
                return derived().nh->getNamespace();
            else{
                ROS_WARN("[Module::%s]\tModule has no namespace, it is not spawned yet...",__func__);
                return std::string();
            }
        }
        inline const std::string getFatherNamespace() const
        {
            if(derived().father_nh)
                return derived().father_nh->getNamespace();
            else{
                ROS_WARN("[Module::%s]\tModule has no father namespace, most likely it's the top level one...",__func__);
                return std::string();
            }
        }
        ///wait for rate
        inline void sleep()
        {
            if (derived().spin_rate)
                derived().spin_rate->sleep();
        }
        //Check OK-ness
        inline bool isOk() const
        {
            if (derived().nh)
                return derived().nh->ok();
            else{
                ROS_WARN("[Module::%s]\tModule has no namespace, it is not spawned yet...",__func__);
                return false;
            }
        }
        inline Storage::Ptr getStorage() const
        {
            return derived().storage;
        }
        inline ros::NodeHandle getNodeHandle() const
        {
            if (derived().nh)
                return *(derived().nh);
            else{
                ROS_WARN("[Module::%s]\tModule has no defined node handle, it is not spawned yet...",__func__);
                return ros::NodeHandle();
            }
        }
        inline ros::NodeHandle getFatherNodeHandle() const
        {
            if (derived().father_nh)
                return *(derived().father_nh);
            else{
                ROS_WARN("[Module::%s]\tModule has no father node handle, most likely it is the top level one...",__func__);
                return ros::NodeHandle();
            }
        }
        void kill()
        {
            if (!derived().isRunning()){
                ROS_WARN("[Module::%s]\tTried to kill %s, but it is not running.",__func__,derived().name.c_str());
                return;
            }
            derived().is_running = false;
            derived().worker.join();
            derived().nh->shutdown();
            derived().nh.reset();
            derived().queue_ptr.reset();
            derived().deInit();
            if (!derived().father_nh)
                //This is the master node, killing it means goodbye
                ros::shutdown();
        }
        //spawn with spin implemented in Derived, or if it doesnt exist use this spin
        void spawn ()
        {
            if(derived().isRunning()){
                ROS_WARN("[Module::%s]\tTried to spawn %s, but it is already spawned and running.",__func__,derived().name.c_str());
                return;
            }
            if(!derived().spin_rate)
                ROS_WARN("[Module::%s]\tSpawning %s without a defined spin rate, thus this node will spin at maximum speed! Call setRate() if you don't want this!",__func__,derived().name.c_str());
            if(derived().father_nh){
                //module version
                derived().nh = std::make_shared<ros::NodeHandle>(*derived().father_nh, name);
                derived().queue_ptr = std::make_shared<ros::CallbackQueue>();
                derived().nh->setCallbackQueue(&(*queue_ptr));
            }
            else if(!derived().nh){
                ROS_ERROR("[Module::%s]\t%s doesn't have neither a node handle nor a father nodehandle to depend onto, thus this module is not validly constructed. Cannor spawn it...",__func__,derived().name.c_str());
            }
            //Derived MUST implement init()
            derived().init();
            derived().is_running = true;
            derived().worker = std::thread(&Derived::spin, derived_ptr());
        }
        inline void disable()
        {
            derived().disabled=true;
        }
        inline void enable()
        {
            derived().disabled=false;
        }
    protected:
        //general spin, cannot be called from outside, only via spawn
        void spin()
        {
            while(derived().nh->ok() && derived().is_running)
            {
                if(!derived().disabled)
                    derived().spinOnce();
                derived().queue_ptr->callAvailable(ros::WallDuration(0));
                derived().sleep();
            }
        }
        void init()
        {
            ROS_WARN("[Module::%s]\tLooks like this module does not implement an init() function. The one provided does nothing. Please implement one to suppress this warning!",__func__);
        }
        void deInit()
        {
            ROS_WARN("[Module::%s]\tLooks like this module does not implement a deInit() function. The one provided does nothing. Please implement one to suppress this warning!",__func__);
        }
    ///////Members
    protected:
        bool is_running;
        bool disabled;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::NodeHandle> father_nh;
        std::shared_ptr<ros::Rate> spin_rate;
        std::string name;
        Storage::Ptr storage;
        std::shared_ptr<ros::CallbackQueue> queue_ptr;
        std::thread worker;
    private:
        //return derived ref
        Derived& derived() { return *static_cast<Derived*>(this); }
        const Derived& derived() const { return *static_cast<const Derived*>(this); }
        //return derived_ptr
        Derived* derived_ptr() {return static_cast<Derived*>(this); }
        const Derived* derived_ptr() const {return static_cast<const Derived*>(this); }
};
}
#endif

/** Example usage:
 *
 * Module Foo
class Foo: public Module<Foo>
{
    public:
        friend class Module<Foo>; //this lets the base class access all of our members, even private ones
        //Foo ()=delete; no need to delete it since its base is deleted
        Foo (const ros::NodeHandle n, const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate):
            Module<Foo>(n,ns,stor,rate)
        {}
    private:
        void spin()
        {
            while(nh.ok() && is_running)
            {
                std::cout<<"Foo is spinning\n";
                spinOnce();
                spin_rate.sleep();
            }
        }
};
* main
int main (int argc, char *argv[])
{
    std::string ns("main");
    ros::init(argc, argv, ns);
    ros::Time::init();
    ros::NodeHandle node(ns);
    ros::Rate rate(10);
    Storage::Ptr storage (new Storage);
    Foo module(node,"foo",storage, 40);
    int count(0);
    while (node.ok())
    {
        //main node is spinning at 10Hz, its namespace is "main"
        ros::spinOnce();
        rate.sleep();
        ++count;
        std::cout<<"Main is spinning\n";
        if (count == 100)
        {
            module.spawn();
            //start Foo spinning at 40Hz, its namespace is "main/foo"
        }
        if (count == 200)
        {
            module.kill();
            //Foo dies, now only main remains
        }
        if (count == 300)
        {
            break;
            //also main dies
        }
    }
    return 0;
}
*/

