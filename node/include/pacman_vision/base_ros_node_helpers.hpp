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
#include <pacman_vision/storage.h>
#include <unordered_map>

///common members between master and modules
class BaseCommon{
    public:
        virtual ~BaseCommon()=default;
    protected:
        BaseCommon()=default;
        ros::NodeHandle nh;
        ros::Rate spin_rate;
        std::string name_space;
        std::shared_ptr<Storage> storage;
};
///Base class module, should be inherited by modules
class Module : public BaseCommon
{
    public:
        ///empy module creation is not allowed
        Module()=delete;
        //ctor with node handle of masternode, namespace, pointer to storage and desired rate
        Module(const ros::NodeHandle &n, const std::string ns, std::shared_ptr<Storage> &stor, const ros::Rate &rate):
            spin_rate(rate), storage(stor)
        {
            this->nh = ros::NodeHandle (n, ns);
            this->name_space = ns;
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
        virtual inline std::string getMyName() const
        {
            //Module identifier
            return ns;
        }
        virtual inline Module::Ptr getMyPtr() const =0;
        typedef std::shared_ptr<Module> Ptr
    protected:
        std::unique_ptr<ros::CallbackQueue> queue_ptr;
};
///Single Driver class, this makes a module live or die
class ModuleDriver
{
    public:
        ModuleDriver():running(false){}
        virtual ~ModuleDriver()=default;
        //spawn a thread "driving" the module
        void spawn(Module:Ptr &mod)
        {
            //subsequent calls to spawn are prohibited
            if(!running){
                module = mod;
                name = module->getMyName();
                worker = std::thread(module->spin());
                running = true;
                return;
            }
            ROS_WARN("[%s]\tTried to spawn %s, but it is already spawned and running.",__func__,module->getMyName().c_str());
        }
        //kill a running module
        void kill()
        {
            if (running){
                module->nh.shutdown();
                worker.join();
                module.reset();
                running = false;
                return;
            }
            ROS_WARN("[%s]\tTried to kill %s, but it is not running.",__func__,module->getMyName().c_str());
        }
        inline bool isRunning() const
        {
            return (running);
        }
        inline Module::Ptr getModule() const
        {
            //Derived classes from Module are forced to implement getMyPtr, so
            //with this function you could access a shared_ptr of Derived type.
            //i.e. by calling from outside auto <return_val>->getMyPtr();
            return (module);
        }
        typedef std::shared_ptr<ModuleDriver> Ptr;
    protected:
        Module::Ptr module;
        std::thread worker;
        bool running;
        std::string name;
};
//Container of ModuleDriver easy typing
typedef std::unordered_map<std::string,ModuleDriver::Ptr> ModuleMap;

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
        MasterNode(std::string ns, ros::Rate &rate)
        {
            nh = ros::NodeHandle(ns);
            name_space = ns;
            storage = std::make_shared<Storage>();
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
        ///Reset everything
        virtual void reset()
        {
            module_map.clear();
            storage=std::make_shared<Storage>();
        }
    protected:
        ///Check Modules Method
        virtual void checkModules()=0;
        ModuleMap module_map;
};
#endif // _INCL_BASE_ROS_NODE_HELPERS_HPP_
