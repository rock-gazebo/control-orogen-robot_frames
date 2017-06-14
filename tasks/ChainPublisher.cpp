/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ChainPublisher.hpp"

using namespace robot_frames;

ChainPublisher::ChainPublisher(std::string const& name)
    : ChainPublisherBase(name)
{
}

ChainPublisher::ChainPublisher(std::string const& name, RTT::ExecutionEngine* engine)
    : ChainPublisherBase(name, engine)
{
}

ChainPublisher::~ChainPublisher()
{
}

void ChainPublisher::clear_and_resize_vectors()
{
    chains_.clear();
    chains_.resize(n_defined_chains_);

    chain_names_.clear();
    chain_names_.resize(n_defined_chains_);

    for(size_t i=0; i<pos_solvers_.size(); i++){
        delete pos_solvers_[i];
    }
    pos_solvers_.clear();
    pos_solvers_.resize(n_defined_chains_);

    kdl_frames_.clear();
    kdl_frames_.resize(n_defined_chains_);

    bt_frames_.clear();
    bt_frames_.resize(n_defined_chains_);

    joint_arrays_.clear();
    joint_arrays_.resize(n_defined_chains_);

    involved_active_joints_.clear();
    involved_active_joints_.resize(n_defined_chains_);

    for(size_t i=0; i<out_ports_.size(); i++){
        ports()->removePort(out_ports_[i]->getName());
        delete out_ports_[i];
    }
    out_ports_.clear();
    out_ports_.resize(n_defined_chains_);
}

bool ChainPublisher::unpack_joints(const base::samples::Joints& joint_state,
                                   const std::vector<std::string>& involved_joints,
                                   KDL::JntArray& joint_array)
{
    assert(involved_joints.size() == joint_array.rows());

    bool ok = true;
    for(size_t i = 0; i<involved_joints.size(); i++){
        try{
            base::JointState js = joint_state.getElementByName(involved_joints[i]);
            joint_array(i) = js.position;
        }
        catch(base::samples::Joints::InvalidName ex){
            LOG_ERROR("Could not find joint %s in joint state vector.", involved_joints[i].c_str());
            ok = false;
        }
    }

    return ok;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ChainPublisher.hpp for more detailed
// documentation about them.

bool ChainPublisher::configureHook()
{
    if (! ChainPublisherBase::configureHook())
        return false;

    std::vector<robot_frames::Chain> chain_definitions = _chains.get();
    n_defined_chains_ = chain_definitions.size();
    if(!n_defined_chains_){
        LOG_WARN_S << "No chains are defined! This component will do nothing..." << std::endl;
    }

    std::string urdf_file_path = _urdf_file.get();

    bool st;
    st = kdl_parser::treeFromFile(urdf_file_path, tree_);
    if(!st){
        LOG_ERROR_S << "Error parsing urdf file: " << urdf_file_path << std::endl;
        return false;
    }

    //Resize stuff
    clear_and_resize_vectors();

    //Receive Chains from urdf
    for(int i=0; i<n_defined_chains_; i++){
        std::string root = chain_definitions[i].root_link;

        //By entering '__base__' as root or tip link, one could refer to the actual base-link of the robot model
        if(root == "__base__"){
            root = tree_.getRootSegment()->first;
        }
        std::string tip = chain_definitions[i].tip_link;
        if(tip == "__base__"){
            tip = tree_.getRootSegment()->first;
        }

        std::string name = chain_definitions[i].name;

        st = tree_.getChain(root, tip, chains_[i]);
        if(!st){
            LOG_ERROR("Error extracting chain '%s' with root '%s' and tip '%s'. The urdf_file is: %s",
                      name.c_str(), root.c_str(), tip.c_str(), urdf_file_path.c_str());
            return false;
        }
        chain_names_[i] = name;
    }

    //Determine invloved joits, prepare frames storage and solvers
    for(int i=0; i<n_defined_chains_; i++){
        KDL::Chain chain = chains_[i];
        //Determine involved joints for each chain
        for(uint s=0; s<chain.segments.size(); s++){
            KDL::Segment segment = chain.segments[s];
            std::string jname = segment.getJoint().getName();

            if(segment.getJoint().getType() == KDL::Joint::None){
                LOG_DEBUG("Skipping joint %s of chain %s. Joint is fixed.",
                          jname.c_str(),
                          chain_names_[i].c_str());
                continue;
            }

            involved_active_joints_[i].push_back(jname);
        }
        //Resize joint arrays
        joint_arrays_[i].resize(involved_active_joints_[i].size());
        joint_arrays_[i].data.setZero();

        //Prepare frames storage
        base::samples::RigidBodyState rbs;
        if(chain_definitions[i].tip_link_renamed == ""){
            rbs.sourceFrame = chain_definitions[i].tip_link;
        }
        else{
            rbs.sourceFrame = chain_definitions[i].tip_link_renamed;
        }
        if(chain_definitions[i].root_link_renamed == ""){
            rbs.targetFrame = chain_definitions[i].root_link;
        }
        else{
            rbs.targetFrame = chain_definitions[i].root_link_renamed;
        }
        rbs.invalidate();
        bt_frames_[i] = rbs;

        //Prepare solver
        pos_solvers_[i] = new KDL::ChainFkSolverPos_recursive(chains_[i]);
    }

    //Create dynamic output per chain
    for(int i=0; i<n_defined_chains_; i++){
        std::string port_name = chain_names_[i];
        LOG_INFO("Adding port %s to component", port_name.c_str());
        RTT::OutputPort<base::samples::RigidBodyState>* output_port =
                new RTT::OutputPort<base::samples::RigidBodyState>(port_name);
        ports()->addPort(port_name, *output_port);
        out_ports_[i] = output_port;
    }

    return true;
}
bool ChainPublisher::startHook()
{
    if (! ChainPublisherBase::startHook())
        return false;
    return true;
}
void ChainPublisher::updateHook()
{
    ChainPublisherBase::updateHook();

    while (_input.read(joint_state_, false) == RTT::NewData){
        int st;
        //Go thourgh chains
        for(int i=0; i<n_defined_chains_; i++){
            KDL::ChainFkSolverPos_recursive* solver = pos_solvers_[i];

            //Extract joint array for chain
            if(!unpack_joints(joint_state_, involved_active_joints_[i], joint_arrays_[i]))
                throw std::runtime_error("Unable to correctly unpack joint values on kinematic chain");

            //Calculate
            st = solver->JntToCart(joint_arrays_[i], kdl_frames_[i]);
            if(st < 0){
                LOG_ERROR_S << "Something went wrong solving forward kineamtics for chain " << chain_names_[i] << std::endl;
            }

            //Convert and write to port
            convert(kdl_frames_[i], bt_frames_[i]);
            bt_frames_[i].time = joint_state_.time;
            out_ports_[i]->write(bt_frames_[i]);
        }
    }
}
void ChainPublisher::errorHook()
{
    ChainPublisherBase::errorHook();
}
void ChainPublisher::stopHook()
{
    ChainPublisherBase::stopHook();
}
void ChainPublisher::cleanupHook()
{
    ChainPublisherBase::cleanupHook();
}
