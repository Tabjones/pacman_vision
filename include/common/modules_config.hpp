#ifndef _MODULES_CONFIG_HPP_
#define _MODULES_CONFIG_HPP_

#include <unordered_map>

#include <common/common_std.h>
#include <common/box.h>

#include <XmlRpcValue.h>

namespace pacv
{
template <typename Derived>
class Config
{
    public:
    virtual ~Config()=default;
    bool set(std::string key, bool value)
    {
        LOCK lock(mtx_config);
        size_t size = derived().map_bool.size();
        derived().map_bool[key] = value;
        if (size != derived().map_bool.size()){
            //key did not exist
            derived().map_bool.erase(key);
            return false;
        }
        return true;
    }
    bool set(std::string key, double value)
    {
        LOCK lock(mtx_config);
        size_t size = derived().map_double.size();
        derived().map_double[key] = value;
        if (size != derived().map_double.size()){
            //key did not exist
            derived().map_double.erase(key);
            return false;
        }
        return true;
    }
    bool set(std::string key, std::string value)
    {
        LOCK lock(mtx_config);
        size_t size =derived().map_string.size();
        derived().map_string[key] = value;
        if (size !=derived().map_string.size()){
            //key did not exist
            derived().map_string.erase(key);
            return false;
        }
        return true;
    }
    bool set(std::string key, int value)
    {
        LOCK lock(mtx_config);
        size_t size =derived().map_int.size();
        derived().map_int[key] = value;
        if (size !=derived().map_int.size()){
            //key did not exist
            derived().map_int.erase(key);
            return false;
        }
        return true;
    }
    bool set(std::string key, Box value)
    {
        LOCK lock(mtx_config);
        size_t size = derived().map_box.size();
        derived().map_box[key] = value;
        if (size != derived().map_box.size()){
            //key did not exist
            derived().map_box.erase(key);
            return false;
        }
        return true;
    }
    bool get(std::string key, bool& value)
    {
        LOCK lock(mtx_config);
        try
        {
            value =derived().map_bool.at(key);
        }
        catch (std::out_of_range)
        {
            return false;
        }
        return true;
    }
    bool get(std::string key, double& value)
    {
        LOCK lock(mtx_config);
        try
        {
            value =derived().map_double.at(key);
        }
        catch (std::out_of_range)
        {
            return false;
        }
        return true;
    }
    bool get(std::string key, std::string& value)
    {
        LOCK lock(mtx_config);
        try
        {
            value =derived().map_string.at(key);
        }
        catch (std::out_of_range)
        {
            return false;
        }
        return true;
    }
    bool get(std::string key, int& value)
    {
        LOCK lock(mtx_config);
        try
        {
            value = derived().map_int.at(key);
        }
        catch (std::out_of_range)
        {
            return false;
        }
        return true;
    }
    bool get(std::string key, Box& value)
    {
        LOCK lock(mtx_config);
        try
        {
            value = derived().map_box.at(key);
        }
        catch (std::out_of_range)
        {
            return false;
        }
        return true;
    }
    bool get(std::string key, XmlRpc::XmlRpcValue& val)
    {
        try
        {
            bool v;
            if (get(key,v)){
                XmlRpc::XmlRpcValue vb(v);
                val = vb;
                return (val.valid());
            }
            double vd;
            if (get(key,vd)){
                val = vd;
                return (val.valid());
            }
            int vi;
            if (get(key,vi)){
                val=vi;
                return (val.valid());
            }
            std::string vs;
            if (get(key, vs)){
                val=vs.c_str();
                return (val.valid());
            }
        }
        catch (...)
        {
            return false;
        }
        return false;
    }
    bool set(std::string key, XmlRpc::XmlRpcValue val)
    {
        try
        {
            if (val.getType() == XmlRpc::XmlRpcValue::TypeBoolean){
                bool value = static_cast<bool>(val);
                return (set(key,value));
            }
            if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble){
                double value = static_cast<double>(val);
                return (set(key,value));
            }
            if (val.getType() == XmlRpc::XmlRpcValue::TypeString){
                std::string value = static_cast<std::string>(val);
                return (set(key,value));
            }
            if (val.getType() == XmlRpc::XmlRpcValue::TypeInt){
                int value = static_cast<int>(val);
                return (set(key,value));
            }
        }
        catch (...)
        {
            return false;
        }
        return false;
    }
    protected:
    std::mutex mtx_config;
    //parameters map of bools
    std::unordered_map<std::string, bool> map_bool;
    //parameters map of doubles
    std::unordered_map<std::string, double> map_double;
    //parameters map of strings
    std::unordered_map<std::string, std::string> map_string;
    //parameters map of int
    std::unordered_map<std::string, int> map_int;
    //parameters map of boxes
    std::unordered_map<std::string, Box> map_box;
    private:
    //return derived ref
    Derived& derived() { return *static_cast<Derived*>(this); }
    const Derived& derived() const {return *static_cast<const Derived*>(this); }
    //return derived_ptr
    Derived* derived_ptr() {return static_cast<Derived*>(this); }
    const Derived* derived_ptr() const {return static_cast<const Derived*>(this); }
};
}
#endif

