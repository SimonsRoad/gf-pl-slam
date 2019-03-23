/*****************************************************************************
**   PL-SLAM: stereo visual SLAM with points and line segment features  	**
******************************************************************************
**																			**
**	Copyright(c) 2017, Ruben Gomez-Ojeda, University of Malaga              **
**	Copyright(c) 2017, MAPIR group, University of Malaga					**
**																			**
**  This program is free software: you can redistribute it and/or modify	**
**  it under the terms of the GNU General Public License (version 3) as		**
**	published by the Free Software Foundation.								**
**																			**
**  This program is distributed in the hope that it will be useful, but		**
**	WITHOUT ANY WARRANTY; without even the implied warranty of				**
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the			**
**  GNU General Public License for more details.							**
**																			**
**  You should have received a copy of the GNU General Public License		**
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.	**
**																			**
*****************************************************************************/

#pragma once

#include <iostream>
#include <vector>
using namespace std;

#include <eigen3/Eigen/Core>
#include <config.h>
using namespace Eigen;

#include <opencv2/highgui/highgui.hpp>

namespace StVO{

class PointFeature
{

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointFeature( const Vector3d & P_, const Vector2d & pl_obs_);
    PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_ );
    PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_, 
        const int & idx_ );
    PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_, 
        const int & idx_, const int & level_ );
    PointFeature( const Vector2d & pl_, const double & disp_, const Vector3d & P_, 
        const Vector2d & pl_obs_ );
    ~PointFeature(){};

    int idx;
    Vector2d pl, pl_obs;
    double   disp;
    Vector3d P;
    bool inlier;

    int level;
    double sigma2 = 1.0;

    bool frame_matched = false;

};

class LineFeature
{

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LineFeature( const Vector3d & sP_, const Vector3d & eP_, const Vector3d & le_obs_);

    LineFeature( const Vector3d & sP_, const Vector3d & eP_, const Vector3d & le_obs_,
                 const Vector2d & spl_obs_, const Vector2d & epl_obs_);

    LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                 const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                 const Vector3d & le_);

    LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                 const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                 const Vector3d & le_, const Vector3d & le_obs_);

    LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                 const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                 const Vector3d & le_,  const int & idx_);

    LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                 const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                 const Vector3d & le_,  const double & angle_, const int & idx_);

    LineFeature( const Vector2d & spl_, const double & sdisp_, const Vector3d & sP_,
                 const Vector2d & epl_, const double & edisp_, const Vector3d & eP_,
                 const Vector3d & le_,  const double & angle_, const int & idx_, const int & level_);

    ~LineFeature(){};

    int idx;
    Vector2d spl, epl, spl_obs, epl_obs;
    double   sdisp, edisp, angle, sdisp_obs, edisp_obs;
    Vector3d sP,eP;
    Vector3d le, le_obs;
    bool inlier;

    int level;
    double sigma2 = 1.0;

    cv::Scalar color;

    bool frame_matched = false;

    // NOTE
    // for uncertainty modeling only
    Matrix3d covSpt_proj2D, covEpt_proj2D;
    Matrix3d covSpt3D, covEpt3D;
    //
    double cutRatio[2];
    vector<double>      volumeArr;
    //
    vector<double>      displPt2D;
    vector<Vector2d>    lPt2D;
    vector<Matrix3d>    covlPt2D;
    vector<double>      varLoss2D;
    Matrix<double, 6, 6> invCovPose;
    int fst_idx, lst_idx;

    // NOTE
    // for error assessment in synthetic data only
    Vector3d sPt_err, ePt_err;
    double sdisp_err, edisp_err;
    bool valid_eval;
};

}
