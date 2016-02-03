// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

