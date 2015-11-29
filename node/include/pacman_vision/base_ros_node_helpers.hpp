#ifndef _INCL_BASE_ROS_NODE_HELPERS_HPP_
#define _INCL_BASE_ROS_NODE_HELPERS_HPP_
//this header is hpp cause it also contains implementations

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <pacman_vision/common.h>
// #include <pacman_vision/storage.h>
#include <unordered_map>

struct Storage{}; //TMP
///common members between master and modules
class BaseCommon{
    public:
        virtual ~BaseCommon()=default;
    protected:
        BaseCommon():spin_rate(10){};
        ros::NodeHandle nh;
        ros::Rate spin_rate;
        std::string name_space;
        std::shared_ptr<Storage> storage;
};
///Base class module, should be inherited by modules
class Module : public BaseCommon
{
    public:
        typedef std::shared_ptr<Module> Ptr;
        ///empy module creation is not allowed
        Module()=delete;
        //ctor with node handle of masternode, namespace, pointer to storage and desired rate
        Module(const ros::NodeHandle &n, const std::string ns, std::shared_ptr<Storage> &stor, const ros::Rate rate)
        {
            this->spin_rate=rate;
            this->storage=stor;
            this->nh = ros::NodeHandle (n, ns);
            this->name_space = ns;
            this->queue_ptr.reset(new ros::CallbackQueue);
            this->nh.setCallbackQueue(&(*queue_ptr));
        }
        virtual ~Module(){}
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
        virtual inline std::string getMyName() const
        {
            //Module identifier
            return name_space;
        }
    protected:
        std::unique_ptr<ros::CallbackQueue> queue_ptr;
};
///Multiple Driver class, this makes modules live or die
class DriverOfModules
{
    public:
        typedef std::shared_ptr<DriverOfModules> Ptr;
        DriverOfModules()=default;
        virtual ~DriverOfModules()=default;
        //spawn a thread "driving" the module
        std::string spawn(Module::Ptr mod)
        {
            std::string key = mod->getMyName();
            //subsequent calls to spawn are prohibited
            if (isRunning(key)){
                ROS_WARN("[%s]\tTried to spawn %s, but it is already spawned and running.",__func__,key.c_str());
                return std::string();
            }
            modules[key] = mod;
            workers[key] = std::thread(&Module::spin, mod);
            running[key] = true;
            return key;
        }
        //kill a running module
        void kill(std::string key)
        {
            if (!isRunning(key)){
                ROS_WARN("[%s]\tTried to kill %s, but it is not running.",__func__,key.c_str());
                return;
            }
            modules[key].reset();
            workers[key].join();
            running[key] = false;
        }
        bool isRunning(std::string key) const
        {
            auto search_key = running.find(key);
            if(search_key != running.end())
                //this key does exist
                return running.at(key);
            else
                return false;
        }
    protected:
        std::unordered_map<std::string,Module::Ptr> modules;
        std::unordered_map<std::string,std::thread> workers;
        std::unordered_map<std::string,bool> running;
};

/**\brief Class MasterNode
 * {:Brief Base Class implementing a ROS master node that uses Modules with
 * separate threads.}
*/
class MasterNode : public BaseCommon
{
    public:
        ///empty obj creation is not allowed
        MasterNode ()=delete;
        ///Only this ctor is allowed
        MasterNode(std::string ns, ros::Rate rate)
        {
            nh = ros::NodeHandle(ns);
            name_space = ns;
            storage = std::make_shared<Storage>();
            spin_rate = rate;
        }
        ///Destructor
        virtual ~MasterNode()=default;
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
        ///wait for rate
        virtual void sleep()
        {
            spin_rate.sleep();
        }
        //Check OK-ness
        virtual inline bool ok()
        {
            return nh.ok();
        }
        ///Reset everything
        virtual void reset()
        {
            module_drivers.reset();
            storage=std::make_shared<Storage>();
        }
    protected:
        ///Check Modules Method
        virtual void checkModules()=0;
        DriverOfModules::Ptr module_drivers;
};
#endif // _INCL_BASE_ROS_NODE_HELPERS_HPP_
