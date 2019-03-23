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

#include "pinholeStereoCamera.h"

PinholeStereoCamera::PinholeStereoCamera( const int & width_, const int & height_,
                                          const double & fx_, const double & fy_, const double & cx_, const double & cy_,
                                          const double & b_,
                                          const double & d0, const double & d1, const double & d2, const double & d3, const double & d4) :
    width(width_), height(height_), fx(fx_), fy(fy_), cx(cx_), cy(cy_), b(b_), dist( d0 != 0.0 )
{
    d   << d0, d1, d2, d3, d4;
    Kl = ( Mat_<float>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 );
    Dl = ( Mat_<float>(1,5) << d(0), d(1), d(2), d(3), d(4) );
    //    Pl = ( Mat_<float>(3,4) << fx, 0.0, cx, 0.0,   0.0, fx, cy, 0.0,   0.0, 0.0, 1.0, 0.0 );
    // MODIFIED by Yipu;
    // seems like the 2nd row of Pl is wrong ???
    // not matter for now since it's not being called
    Pl = ( Mat_<float>(3,4) << fx, 0.0, cx, 0.0,   0.0, fy, cy, 0.0,   0.0, 0.0, 1.0, 0.0 );

    K    << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    // initialize undistort rectify map OpenCV
    initUndistortRectifyMap( Kl, Dl, cv::Mat_<double>::eye(3,3), Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
    undistmap1r = undistmap1l;
    undistmap2r = undistmap2l;
}

PinholeStereoCamera::PinholeStereoCamera( const int & width_, const int & height_,
                                          const double & fx_, const double & fy_, const double & cx_, const double & cy_,
                                          const double & b_,
                                          const cv::Mat & Rl_, const cv::Mat & Rr_,
                                          const double & d0, const double & d1, const double & d2, const double & d3, const double & d4) :
    width(width_), height(height_), fx(fx_), fy(fy_), cx(cx_), cy(cy_), b(b_), dist( d0 != 0.0 ), Rl(Rl_), Rr(Rr_)
{
    d   << d0, d1, d2, d3, d4;
    Kl = ( Mat_<float>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 );
    Dl = ( Mat_<float>(1,5) << d(0), d(1), d(2), d(3), d(4) );
    K    << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    // initialize undistort rectify map OpenCV
    initUndistortRectifyMap( Kl, Dl, Rl, Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
    initUndistortRectifyMap( Kr, Dr, Rr, Pr, cv::Size(width,height), CV_16SC2, undistmap1r, undistmap2r );
}

PinholeStereoCamera::PinholeStereoCamera( const int & width_, const int & height_,
                                          const double & b_,
                                          const Mat & Kl_, const Mat & Kr_,
                                          const Mat & R_, const Mat & t_,
                                          const Mat & Dl_, const Mat & Dr_,
                                          const bool & equi) :
    width(width_), height(height_), Kl(Kl_), Kr(Kr_), b(b_), R(R_), t(t_), Dl(Dl_), Dr(Dr_)
{

    // initialize undistort rectify map OpenCV
    if(equi)
    {
        stereoRectify( Kl, Dl, Kr, Dr, cv::Size(width,height), R, t, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0 );
        cv::fisheye::initUndistortRectifyMap( Kl, Dl, Rl, Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
        cv::fisheye::initUndistortRectifyMap( Kr, Dr, Rr, Pr, cv::Size(width,height), CV_16SC2, undistmap1r, undistmap2r );
    }
    else
    {
        stereoRectify( Kl, Dl, Kr, Dr, cv::Size(width,height), R, t, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0 );
        initUndistortRectifyMap( Kl, Dl, Rl, Pl, cv::Size(width,height), CV_16SC2, undistmap1l, undistmap2l );
        initUndistortRectifyMap( Kr, Dr, Rr, Pr, cv::Size(width,height), CV_16SC2, undistmap1r, undistmap2r );
    }

    dist = true;

    fx = Pl.at<double>(0,0);
    fy = Pl.at<double>(1,1);
    cx = Pl.at<double>(0,2);
    cy = Pl.at<double>(1,2);

    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

}

PinholeStereoCamera::~PinholeStereoCamera() {};

void PinholeStereoCamera::rectifyImage( const Mat& img_src, Mat& img_rec)
{
    if(dist)
      remap( img_src, img_rec, undistmap1l, undistmap2l, cv::INTER_LINEAR);
    else
      img_rec = img_src.clone();
}

void PinholeStereoCamera::rectifyImagesLR( const Mat& img_src_l, Mat& img_rec_l,
                                           const Mat& img_src_r, Mat& img_rec_r )
{
    if(dist)
    {
        remap( img_src_l, img_rec_l, undistmap1l, undistmap2l, cv::INTER_LINEAR);
        remap( img_src_r, img_rec_r, undistmap1r, undistmap2r, cv::INTER_LINEAR);
    }
    else
    {
        img_rec_l = img_src_l.clone();
        img_rec_r = img_src_r.clone();
    }
}

// Proyection and Back-projection (internally we are supposed to work with rectified images because of the line segments)
Vector3d PinholeStereoCamera::backProjection_unit( const double &u, const double &v, const double &disp,
                                                   double &depth )
{
    Vector3d P_unit;
    P_unit(0) = (u-cx)/fx;
    P_unit(1) = (v-cy)/fy;
    P_unit(2) = 1.0;
    depth     = b*fx/disp;
    return P_unit;
}

Vector3d PinholeStereoCamera::backProjection( const double &u, const double &v, const double &disp )
{
    Vector3d P;
    double bd = b/disp;
    P(0) = bd*(u-cx);
    P(1) = bd*(v-cy);
    P(2) = bd*fx;
    return P;
}

Vector2d PinholeStereoCamera::normalizedCoordinates( const Vector2d & P )
{
    Vector3d uv_unit;
    uv_unit(0) = (P(0)-cx)/fx ;
    uv_unit(1) = (P(1)-cy)/fy ;
    uv_unit(2) = 1.0;
    //double sum = uv_unit(0)*uv_unit(0) + uv_unit(1)*uv_unit(1)* + uv_unit(2)*uv_unit(2);
    double sum = uv_unit.norm();
    uv_unit(0) = uv_unit(0)/sum;
    uv_unit(1) = uv_unit(1)/sum;
    uv_unit(2) = uv_unit(2)/sum;
    Vector2d uv_norm;
    uv_norm<<uv_unit(0)/uv_unit(2),uv_unit(1)/uv_unit(2);
    return uv_norm;
}

double PinholeStereoCamera::getDisparity( const double pZ )
{
    return fx * b / pZ;
}

Vector2d PinholeStereoCamera::projection( const Vector3d & P )
{
    Vector2d uv_unit;
    uv_unit(0) = cx + fx * P(0) / P(2);
    uv_unit(1) = cy + fy * P(1) / P(2);
    return uv_unit;
}

Vector2d PinholeStereoCamera::projectionNormalized( const Vector3d & P )
{
    Vector2d uv_unit;
    uv_unit(0) = P(0) / P(2);
    uv_unit(1) = P(1) / P(2);
    return uv_unit;
}

Vector3d PinholeStereoCamera::projectionNH( const Vector3d & P )
{
    Vector3d uv_proj;
    uv_proj(0) = cx * P(2) + fx * P(0);
    uv_proj(1) = cy * P(2) + fy * P(1);
    uv_proj(2) = P(2);
    return uv_proj;
}

Vector2d PinholeStereoCamera::nonHomogeneous( const Vector3d & x)
{
    Vector2d x_; x_ << x(0) / x(2), x(1) / x(2);
    return x_;
}

Vector2d PinholeStereoCamera::projectionDistorted( const Vector3d & P,
                                                   const double & K1, const double & K2, const double & K3 )
{
    double x_norm = P(0) / P(2), y_norm = P(1) / P(2);
    double r_2 = x_norm * x_norm + y_norm * y_norm;
    //
    double dist_coeff = ( 1 + K1*r_2 + K2*pow(r_2, 2) + K3*pow(r_2, 3) );
    double x_dist = x_norm * dist_coeff,
            y_dist = y_norm * dist_coeff;
    Vector2d uv_unit;
    uv_unit(0) = cx + fx * x_dist;
    uv_unit(1) = cy + fy * y_dist;
    return uv_unit;
}
