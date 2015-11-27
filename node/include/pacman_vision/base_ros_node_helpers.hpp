#ifndef _INCL_BASE_ROS_NODE_HELPERS_HPP_
#define _INCL_BASE_ROS_NODE_HELPERS_HPP_

//this header is hpp cause it also contains some implementations with templates

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <pacman_vision/common.h>
#include <pacman_vision/storage.h>

// TODO: Add container of ModuleDriver masternode can inherit.(tabjones on Friday 27/11/2015)
// TODO: Think of using driver without template, i.e. pointer to Module base (tabjones on Friday 27/11/2015)
///Single Driver class
template <typename TMOD>
class ModuleDriver
{
    public:
        ModuleDriver():running(false){}
        virtual ~ModuleDriver(){}

        //spawn a thread "driving" the module
        void spawn(TMOD &mod)
        {
            //subsequent calls to spawn are prohibited
            if(!running){
                module.reset(&mod);
                worker = std::thread(module->spin());
                running = true;
            }
            ROS_WARN("[%s]\tTried to spawn %s, but it is already spawned and running.",__func__,mod.string().c_str());
        }
        //kill a running module
        void kill()
        {
            module->nh.shutdown();
            worker.join();
            module.reset();
            running = false;
        }
        bool isRunning()
        {
            return (running);
        }
        typedef std::shared_ptr<ModuleDriver<TMOD>> Ptr;
    protected:
        std::shared_ptr<TMOD> module;
        std::thread worker;
        bool running;
}

///Base class modules should inherit
class Module
{
    friend class ModuleDriver; //module driver could mess with this module
    public:
        //ctor with node handle of masternode, namespace, pointer to storage and desired rate
        Module(const ros::NodeHandle &n, const std::string ns, std::shared_ptr<Storage> &stor, const ros::Rate rate):
            spin_rate(rate), storage(stor)
        {
            this->nh = ros::NodeHandle (n, ns);
            this->queue_ptr.reset(new ros::CallbackQueue);
            this->nh.setCallbackQueue(&(*queue_ptr));
        }
        virtual ~Module()
        {
            this->nh.shutdown();
        }
        virtual void spinOnce()
        {
            this->queue_ptr->callAvailable(ros::WallDuration(0));
        }
        virtual void spin()
        {
            while(this->nh.ok())
            {
                this->spinOnce();
                spin_rate.sleep();
            }

        }
    protected:
        ///protected ctor, empy obj creation is not allowed
        Module(){}
        ros::NodeHandle nh;
        ros::Rate spin_rate;
        std::unique_ptr<ros::CallbackQueue> queue_ptr;
        std::shared_ptr<Storage> storage;
};

/**\brief Class MasterNode
 * {:Brief Base Class implementing a ROS master node that uses Modules with
 * separate threads.}
*/
class MasterNode
{
    public:
        ///Only this ctor is allowed
        MasterNode(std::string ns, ros::Rate rate)
        {
            nh = ros::NodeHandle(ns);
            storage.reset(new Storage);
            spin_rate = rate;
        }
        ///Destructor
        virtual ~MasterNode ();
        ///Main loop spin, crude implementation
        virtual void spin()
        {
            while (this->nh.ok())
            {
                this->spinOnce();
                this->spin_rate.sleep();
            }
        }
        ///Custom spin method
        virtual void spinOnce()
        {
            ros::spinOnce();
        }
        ///Shut it down
        virtual void reset()
        {
            modules.clear();
            storage.reset(new Storage);
        }
    protected:
        ///Constructor protected, empty obj creation is not allowed
        MasterNode (){}
        ///Node Handle
        ros::NodeHandle nh;
        /// desired spin rate
        ros::Rate spin_rate;
        /// Shared pointer to Storage Class
        std::shared_ptr<Storage> storage;
        //Container of modules this class uses
        std::vector<Module> modules;
        ///Check Modules Method
        virtual void checkModules()=0;
};

#endif // _INCL_BASE_ROS_NODE_HELPERS_HPP_
