#ifndef _DYNAMIC_MODULES_HPP_
#define _DYNAMIC_MODULES_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <pacman_vision/common.h>
#include <pacman_vision/storage.h>

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
        //and desired rate. This is for master node
        Module(const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate):
            is_running(false), spin_rate(rate), disabled(false)
        {
            nh = ros::NodeHandle (ns);
            storage=stor;
            name_space = ns;
        }
        //ctor with  node handle of  masternode, namespace, pointer  to storagee
        //and desired rate. This is for modules, dependant of master node
        Module(const ros::NodeHandle n, const std::string ns, const std::shared_ptr<Storage> stor, const ros::Rate rate):
            is_running(false), spin_rate(rate), disabled(false)
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
        inline const std::string getName() const
        {
            return derived().name_space;
        }
        ///wait for rate
        inline void sleep()
        {
            derived().spin_rate.sleep();
        }
        //Check OK-ness
        inline bool isOk() const
        {
            return derived().nh.ok();
        }
        inline Storage::ConstPtr getStorage() const
        {
            return derived().storage;
        }
        inline const ros::NodeHandle getNodeHandle() const
        {
            return derived().nh;
        }
        void kill()
        {
            if (!derived().isRunning()){
                ROS_WARN("[%s]\tTried to kill %s, but it is not running.",__func__,derived().name_space.c_str());
                return;
            }
            derived().is_running = false;
            derived().worker.join();
        }
        //spawn with spin implemented in Derived, or if it doesnt exist use this spin
        void spawn ()
        {
            if(derived().isRunning()){
                ROS_WARN("[%s]\tTried to spawn %s, but it is already spawned and running.",__func__,derived().name_space.c_str());
                return;
            }
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
            while(derived().nh.ok() && derived().is_running)
            {
                if(!derived().disabled){
                    derived().spinOnce();
                    derived().spin_rate.sleep();
                }
            }
        }
    ///////Members
    protected:
        bool is_running;
        bool disabled;
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
};
#endif // _BASE_ROS_NODE_HELPERS_HPP_

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

