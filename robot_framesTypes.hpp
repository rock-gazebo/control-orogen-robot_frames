#ifndef robot_frames_TYPES_HPP
#define robot_frames_TYPES_HPP

#include <string>

namespace robot_frames {
    /** Description of a chain to compute the forward kinematics for
     */
    struct Chain
    {
        /** The name of the chain
         *
         * Used only in ChainPublisher and Task to determine the output port
         * name
         */
        std::string name;

        /** The name of the root link in the resolved pose
         *
         * The computed pose is the pose of the tip expressed in the root frame,
         * a.k.a. sourceFrame is tip and targetFrame is root
         */
        std::string root_link;

        /** If non-empty, override the targetFrame of the output pose to this
         * value instead of using root_link
         */
        std::string root_link_renamed;

        /** The name of the tip link in the resolved pose
         *
         * The computed pose is the pose of the tip expressed in the root frame,
         * a.k.a. sourceFrame is tip and targetFrame is root
         */
        std::string tip_link;

        /** If non-empty, override the sourceFrame of the output pose to this
         * value instead of using root_link
         */
        std::string tip_link_renamed;
    };
}

#endif

