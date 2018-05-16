/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SingleChainPublisher.hpp"

#include <robot_frames/RobotFrames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <base-logging/Logging.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace std;
using namespace robot_frames;

SingleChainPublisher::SingleChainPublisher(std::string const& name)
    : SingleChainPublisherBase(name)
{
}

SingleChainPublisher::SingleChainPublisher(std::string const& name, RTT::ExecutionEngine* engine)
    : SingleChainPublisherBase(name, engine)
{
}

SingleChainPublisher::~SingleChainPublisher()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SingleChainPublisher.hpp for more detailed
// documentation about them.

bool SingleChainPublisher::configureHook()
{
    if (! SingleChainPublisherBase::configureHook())
        return false;

    pair<string, kdl_parser::ROBOT_MODEL_FORMAT> model =
        getRobotModelString(_robot_model, _robot_model_format);

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(model.first, tree, model.second))
    {
        LOG_ERROR("Failed to convert the robot model into a KDL tree");
        return false;
    }

    Chain chain = _chain.get();

    std::string root = chain.root_link;
    if(root == "__base__"){
        root = tree.getRootSegment()->first;
    }
    std::string tip = chain.tip_link;
    if(tip == "__base__"){
        tip = tree.getRootSegment()->first;
    }
    
    KDL::Chain kdl_chain;
    if (!tree.getChain(root, tip, kdl_chain)) {
        LOG_ERROR("Error extracting chain with root '%s' and tip '%s'.",
                  root.c_str(), tip.c_str());
        return false;
    }

    //Determine involved joints for each chain
    joints_names_.clear();
    for(uint s=0; s<kdl_chain.segments.size(); s++){
        KDL::Segment segment = kdl_chain.segments[s];

        if (segment.getJoint().getType() != KDL::Joint::None)
        {
            std::string jname = segment.getJoint().getName();
            joints_names_.push_back(jname);
        }
    }
    joints_array_.data.resize(joints_names_.size());
    joints_array_.data.setZero();

    //Prepare frames storage
    output_pose_.invalidate();
    if(chain.tip_link_renamed == ""){
        output_pose_.sourceFrame = chain.tip_link;
    }
    else{
        output_pose_.sourceFrame = chain.tip_link_renamed;
    }
    if(chain.root_link_renamed == ""){
        output_pose_.targetFrame = chain.root_link;
    }
    else{
        output_pose_.targetFrame = chain.root_link_renamed;
    }
    this->chain_ = kdl_chain;
    return true;
}
bool SingleChainPublisher::startHook()
{
    if (! SingleChainPublisherBase::startHook())
        return false;
    return true;
}

void SingleChainPublisher::updateHook()
{
    SingleChainPublisherBase::updateHook();

    while (_joints_samples.read(joints_samples_, false) == RTT::NewData){
        for(size_t i = 0; i<joints_names_.size(); i++){
            if (joints_names_[i].empty())
                continue;

            base::JointState js = joints_samples_.getElementByName(joints_names_[i]);
            if (!js.hasPosition())
                exception(JOINT_POSITION_INVALID);
            joints_array_(i) = js.position;
        }

        //Calculate
        KDL::ChainFkSolverPos_recursive chain_solver(chain_);
        KDL::Frame kdl_frame;
        int result = chain_solver.JntToCart(joints_array_, kdl_frame);
        if(result < 0){
            LOG_ERROR_S << "Something went wrong solving forward kineamtics for the chain " << chain_solver.strError(result) << endl;
            exception(FORWARD_KINEMATICS_FAILED);
            return;
        }

        //Convert and write to port
        convert(kdl_frame, output_pose_);
        output_pose_.time = joints_samples_.time;
        _tip_pose.write(output_pose_);
    }
}
void SingleChainPublisher::errorHook()
{
    SingleChainPublisherBase::errorHook();
}
void SingleChainPublisher::stopHook()
{
    SingleChainPublisherBase::stopHook();
}
void SingleChainPublisher::cleanupHook()
{
    SingleChainPublisherBase::cleanupHook();
}
