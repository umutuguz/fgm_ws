/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef PEOPLE_TRACKING_FILTER_SYSMODEL_VECTOR_H
#define PEOPLE_TRACKING_FILTER_SYSMODEL_VECTOR_H


#include <people_tracking_filter/gaussian_vector.h>
#include <bfl/model/systemmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>

namespace BFL
{

class SysPdfVector
  : public ConditionalPdf<tf::Vector3, tf::Vector3>
{
public:
  /// Constructor
  explicit SysPdfVector(const tf::Vector3& sigma);

  /// Destructor
  virtual ~SysPdfVector();

  // set time
  void SetDt(double dt)
  {
    dt_ = dt;
  }

  // Redefining pure virtual methods
  virtual bool SampleFrom(BFL::Sample<tf::Vector3>& one_sample, int method, void *args) const;
  virtual tf::Vector3 ExpectedValueGet() const;  // not applicable
  virtual Probability ProbabilityGet(const tf::Vector3& state) const;  // not applicable
  virtual MatrixWrapper::SymmetricMatrix  CovarianceGet() const;  // Not applicable

private:
  GaussianVector noise_;
  double dt_;
};  // class

class SysModelVector
  : public SystemModel<tf::Vector3>
{
public:
  explicit SysModelVector(const tf::Vector3& sigma)
    : SystemModel<tf::Vector3>(new SysPdfVector(sigma))
  {}

  /// destructor
  ~SysModelVector()
  {
    delete SystemPdfGet();
  }

  // set time
  void SetDt(double dt)
  {
    static_cast<SysPdfVector*>(SystemPdfGet())->SetDt(dt);
  }
};  // class
}  // namespace BFL
#endif  // PEOPLE_TRACKING_FILTER_SYSMODEL_VECTOR_H
