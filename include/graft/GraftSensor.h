/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#ifndef GRAFT_SENSOR_H_
#define GRAFT_SENSOR_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <hratc2014_framework/GraftState.h>
#include <hratc2014_framework/GraftSensorResidual.h>

#include <nav_msgs/Odometry.h>

using namespace Eigen;

class GraftSensor{
  public:
    virtual ~GraftSensor(){}

    //virtual MatrixXd H(graft::GraftState& state) = 0;

    virtual hratc2014_framework::GraftSensorResidual::Ptr z() = 0;

    virtual hratc2014_framework::GraftSensorResidual::Ptr h(const hratc2014_framework::GraftState& state) = 0;

    virtual void setName(const std::string& name) = 0;

    virtual std::string getName() = 0;

    virtual void clearMessage() = 0;

    //virtual graft::GraftSensorResidual y(graft::GraftState& predicted) = 0;

    //virtual MatrixXd R() = 0;

  private:

    

    
};

#endif