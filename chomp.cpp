/* Trials with CHOMP.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
   \file pp2d.cpp

   Interactive trials with CHOMP for point vehicles moving
   holonomously in the plane.  There is a fixed start and goal
   configuration, and you can drag a circular obstacle around to see
   how the CHOMP algorithm reacts to that.  Some of the computations
   involve guesswork, for instance how best to compute velocities, so
   a simple first-order scheme has been used.  This appears to produce
   some unwanted drift of waypoints from the start configuration to
   the end configuration.  Parameters could also be tuned a bit
   better.  Other than that, it works pretty nicely.
*/


#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <stdlib.h>
#include <err.h>
#include "chomp.hpp"

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Isometry3d Transform;

using namespace std;

namespace chomp{
//////////////////////////////////////////////////
// trajectory etc


static size_t const nq (20);	// number of q stacked into xi
static size_t const cdim (2);	// dimension of config space
static size_t const xidim (nq * cdim); // dimension of trajectory, xidim = nq * cdim
static double const dt (1.0);	       // time step
static double const eta (100.0); // >= 1, regularization factor for gradient descent
static double const lambda (1.0); // weight of smoothness objective

//////////////////////////////////////////////////
// gradient descent etc

Matrix AA;			// metric
Vector bb;			// acceleration bias for start and end config
Matrix Ainv;			// inverse of AA


/*
static handle_s rep1 (3.0, 0.0,   2.0, 0.0, 0.0, 1.0, 0.2);
static handle_s rep2 (0.0, 3.0,   2.0, 0.0, 0.5, 1.0, 0.2);

static handle_s * handle[] = { &rep1, &rep2, 0 };
*/
//////////////////////////////////////////////////
// robot (one per waypoint)



static void init_chomp (Vector const &qs,Vector const &qe,Vector  &xi)
{

  xi = Vector::Zero (xidim);
  for (size_t ii (0); ii < nq; ++ii) {
    xi.block (cdim * ii, 0, cdim, 1) = qs;
  }

  AA = Matrix::Zero (xidim, xidim);
  for (size_t ii(0); ii < nq; ++ii) {
    AA.block (cdim * ii, cdim * ii, cdim , cdim) = 2.0 * Matrix::Identity (cdim, cdim);
    if (ii > 0) {
      AA.block (cdim * (ii-1), cdim * ii, cdim , cdim) = -1.0 * Matrix::Identity (cdim, cdim);
      AA.block (cdim * ii, cdim * (ii-1), cdim , cdim) = -1.0 * Matrix::Identity (cdim, cdim);
    }
  }
  AA /= dt * dt * (nq + 1);

  bb = Vector::Zero (xidim);
  bb.block (0,            0, cdim, 1) = qs;
  bb.block (xidim - cdim, 0, cdim, 1) = qe;
  bb /= - dt * dt * (nq + 1);

  // not needed anyhow
  // double cc (double (qs.transpose() * qs) + double (qe.transpose() * qe));
  // cc /= dt * dt * (nq + 1);

  Ainv = AA.inverse();

  // cout << "AA\n" << AA
  //      << "\nAinv\n" << Ainv
  //      << "\nbb\n" << bb << "\n\n";
}


static void chomp_iteration (Vector const &qs,Vector const &qe,Vector  &xi, Matrix const &obs)
{

  //////////////////////////////////////////////////
  // beginning of "the" CHOMP iteration

  // static size_t stepcounter (0);
  // cout << "step " << stepcounter++ << "\n";


  Vector nabla_smooth (AA * xi + bb);
  Vector const & xidd (nabla_smooth); // indeed, it is the same in this formulation...

  Vector nabla_obs (Vector::Zero (xidim));
  for (size_t iq (0); iq < nq; ++iq) {
    Vector const qq (xi.block (iq * cdim, 0, cdim, 1));
    Vector qd;
    if (0 == iq) {
      qd = 0.5 * (xi.block ((iq+1) * cdim, 0, cdim, 1) - qs);
    }
    else if (iq == nq - 1) {
      qd = 0.5 * (qe - xi.block ((iq-1) * cdim, 0, cdim, 1));
    }
    else {
      qd = 0.5 * (xi.block ((iq+1) * cdim, 0, cdim, 1) - xi.block ((iq-1) * cdim, 0, cdim, 1));;
    }

    // In this case, C and W are the same, Jacobian is identity.  We
    // still write more or less the full-fledged CHOMP expressions
    // (but we only use one body point) to make subsequent extension
    // easier.
    //
    Vector const & xx (qq);
    Vector const & xd (qd);
    Matrix const JJ (Matrix::Identity (2, 2)); // a little silly here, as noted above.
    double const vel (xd.norm());
    if (vel < 1.0e-3) {	// avoid div by zero further down
      continue;
    }
    Vector const xdn (xd / vel);
    Vector const xdd (JJ * xidd.block (iq * cdim, 0, cdim , 1));
    Matrix const prj (Matrix::Identity (2, 2) - xdn * xdn.transpose()); // hardcoded planar case
    Vector const kappa (prj * xdd / pow (vel, 2.0));


    for (size_t ii=0;ii<obs.cols();ii++){
      Vector delta (xx - obs.block(0,ii,cdim,1));
      double const dist (delta.norm());
      if ((dist >= obs(2,ii)) || (dist < 1e-9)) {
          continue;
      }
      static double const gain (10.0); // hardcoded param
      double const cost (gain * obs(2,ii) * pow (1.0 - dist / obs(2,ii), 3.0) / 3.0); // hardcoded param
      delta *= - gain * pow (1.0 - dist / obs(2,ii), 2.0) / dist; // hardcoded param
      nabla_obs.block (iq * cdim, 0, cdim, 1) += JJ.transpose() * vel * (prj * delta - cost * kappa);
    }
/*
    for (handle_s ** hh (handle); *hh != 0; ++hh) {
      Vector delta (xx - (*hh)->point_);
      double const dist (delta.norm());
      if ((dist >= (*hh)->radius_) || (dist < 1e-9)) {
	continue;
      }
      static double const gain (10.0); // hardcoded param
      double const cost (gain * (*hh)->radius_ * pow (1.0 - dist / (*hh)->radius_, 3.0) / 3.0); // hardcoded param
      delta *= - gain * pow (1.0 - dist / (*hh)->radius_, 2.0) / dist; // hardcoded param
      nabla_obs.block (iq * cdim, 0, cdim, 1) += JJ.transpose() * vel * (prj * delta - cost * kappa);
    }
    */
  }

  Vector dxi (Ainv * (nabla_obs + lambda * nabla_smooth));
  xi -= dxi / eta;

  // end of "the" CHOMP iteration
  //////////////////////////////////////////////////

  //update_robots ();

}

void run_chomp(Vector const &qs,Vector const &qe, Vector &xi, Matrix const &obs)
{
/*

  chomp_iteration();
  cout<<"Done!"<<'\n';
*/

cout<<"Start Point:"<<'\n';
cout<<qs<<'\n';
cout<<"End Point:"<<'\n';
cout<<qe<<'\n';

cout<<"xi':"<<'\n';
cout<<xi.transpose()<<'\n';
init_chomp(qs,qe,xi);
cout<<"xi':"<<'\n';
cout<<xi.transpose()<<'\n';
for (int i=0;i<500;i++){
chomp_iteration(qs,qe,xi,obs);
}
cout<<"xi':"<<'\n';
cout<<xi.transpose()<<'\n';
}
} //namespace
