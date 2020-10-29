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

#ifndef __VELMA_DYNAMIC_MODEL_BASE_HPP__
#define __VELMA_DYNAMIC_MODEL_BASE_HPP__

#include <ros/ros.h>
#include "ros/package.h"

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

class VelmaDynamicModelBase;
typedef std::shared_ptr<VelmaDynamicModelBase> VelmaDynamicModelBasePtr;

class VelmaDynamicModelBase {
protected:
	std::vector<std::string > controlled_joints_;
	dart::dynamics::SkeletonPtr sk_;
	dart::simulation::WorldPtr world_;
	double time_step_;
	std::vector<int> controlled_joints_idx_map_;
    VelmaDynamicModelBase(dart::dynamics::SkeletonPtr &sk,
    										const std::vector<std::string >& controlled_joints);

public:

	virtual void getGravityForces(Eigen::VectorXd &result) const = 0;
	virtual void getCoriolisForces(Eigen::VectorXd &result) const = 0;
	virtual void getPositions(Eigen::VectorXd &result) const = 0;
	virtual void setPositions(const Eigen::VectorXd &pos) = 0;
	virtual void getVelocities(Eigen::VectorXd &result) const = 0;
	virtual void setVelocities(const Eigen::VectorXd &vel) = 0;
	virtual void setForces(const Eigen::VectorXd &force) = 0;
	virtual void step() = 0;

	bool getFk(const std::string &link_name, Eigen::Isometry3d &T_B_L) const;
	std::vector<std::string > getLinkNames() const;

	dart::dynamics::SkeletonPtr getSkeleton();
};

#endif  // __VELMA_DYNAMIC_MODEL_BASE_HPP__