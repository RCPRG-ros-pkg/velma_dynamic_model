// Copyright (c) 2020, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <ros/ros.h>
#include "ros/package.h"

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include <dart/utils/PackageResourceRetriever.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <velma_dynamic_model/velma_dynamic_model_base.h>

VelmaDynamicModelBase::VelmaDynamicModelBase(dart::dynamics::SkeletonPtr &sk,
        const std::vector<std::string >& controlled_joints)
: sk_(sk)
, controlled_joints_(controlled_joints)
, world_(new dart::simulation::World())
, time_step_(0.002)
, grav_(0,0,-9.8) {

    for (int jnt_idx = 0; jnt_idx < controlled_joints_.size(); ++jnt_idx) {
        const std::string& moveable_joint_name = controlled_joints_[jnt_idx];
        dart::dynamics::Joint *joint = sk_->getJoint(moveable_joint_name);
        int q_idx = joint->getIndexInSkeleton(0);
        controlled_joints_idx_map_.push_back( q_idx );
        //std::cout << "Joint \"" << moveable_joint_name << "\" has index " << q_idx << std::endl;
    }

    sk_->disableSelfCollisionCheck();
    world_->addSkeleton(sk);
    world_->setGravity(grav_);
    world_->setTimeStep(time_step_);
}

const Eigen::Vector3d& VelmaDynamicModelBase::getGravVector() const {
    return grav_;       
}

bool VelmaDynamicModelBase::getFk(const std::string &link_name, Eigen::Isometry3d &T_B_L) const {
    dart::dynamics::BodyNode *b = sk_->getBodyNode(link_name);
    if (!b) {
        return false;
    }
    T_B_L = b->getTransform();
    return true;
}

std::vector<std::string > VelmaDynamicModelBase::getLinkNames() const {
    std::vector<std::string > result;
    for (int bidx = 0; bidx < sk_->getNumBodyNodes(); bidx++) {
        result.push_back( sk_->getBodyNode(bidx)->getName() );
    }
    return result;
}

dart::dynamics::SkeletonPtr VelmaDynamicModelBase::getSkeleton() {
    return sk_;
}
