/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "base-logging/Logging.hpp"
#include <sstream>

using namespace robot_frames;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    // Delete dynamic ports
    for(OutputPortMap::iterator it=out_ports.begin(); it!=out_ports.end(); ++it){
        ports()->removePort(it->second->getName());
        delete it->second;
    }
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    calc.output_only_valid(_output_only_valid.get());

    std::string urdf_file = _urdf_file.get();
    LOG_ERROR("Configuring with URDF file %s", urdf_file.c_str());
    try{
        calc.load_robot_model(urdf_file, _init_invalid.get());
    }
    catch(std::runtime_error ex){
        LOG_ERROR("Exception while initializing from urdf file: ", urdf_file.c_str());
        throw(ex);
    }
    LOG_DEBUG("Robot model loaded");

    output_static_transforms_ = _output_static_transforms.get();


    std::string j_name;
    std::string seg_name;
    std::string parent_seg_name;
    if(output_static_transforms_)
        joint_names_ = calc.get_all_joint_names();
    else
        joint_names_ = calc.get_moving_joint_names();

    std::stringstream ss;
    for(uint i=0; i<joint_names_.size(); i++){
        ss << joint_names_[i] << " ";
    }
    LOG_INFO("The following joint names where found in URDF: %s",ss.str().c_str());
    ss.str("");

    LOG_INFO("Creating output ports..");
    for(uint i=0; i<joint_names_.size(); i++){
        j_name = joint_names_[i];
        seg_name = calc.get_segment_name_from_joint_name(j_name);
        parent_seg_name = calc.get_parent_name_by_segment_name(seg_name);

        LOG_INFO("Adding transformation output port for joint %s. It transforms %s -> %s",
                 j_name.c_str(), parent_seg_name.c_str(), seg_name.c_str());
        RTT::OutputPort<base::samples::RigidBodyState>* output_port =
                new RTT::OutputPort<base::samples::RigidBodyState>(joint_names_[i]);
        ports()->addPort(joint_names_[i], *output_port);
        out_ports[j_name] = output_port;

        /*
         * I think adding properties like this (addProperty) does not work
         * as expected.
         * when changing the value of one property in rock-display, all
         * propertiesvalues changed.
         *
        ss.str("");
        ss << seg_name << "_frame";
        LOG_INFO("Adding property '%s' for frame renaming",
                 ss.str().c_str());
        addProperty(ss.str(), seg_name);
        */
    }
    LOG_INFO("Creating output ports.. done");

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::Joints joints;
    while (_input.read(joints, false) == RTT::NewData){
        calc.update(joints);
    }


    base::samples::RigidBodyState transform;
    bool success=true;
    for(uint i=0; i<joint_names_.size(); i++){
        success = calc.get_transform_by_joint_name(joint_names_[i], transform);
        if(success){
            OutputPortMap::iterator it = out_ports.find(joint_names_[i]);

            //This should never be triggered
            assert(it != out_ports.end());

            //Write output
            it->second->write(transform);
        }
        else{
            LOG_INFO("Could not get transform for joint %s.", joint_names_[i].c_str());
        }
    }
}


void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
