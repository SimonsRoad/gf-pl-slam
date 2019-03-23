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

#include <assert.h>
//#define NODEBUG           // only uncomment when running full pipeline test!
#include <fstream>

#include <future>
#include <thread>
#include <time.h>
using namespace std;

#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/viz.hpp>

#include "ORBextractor.h"

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
using namespace cv;
using namespace line_descriptor;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include <config.h>
#include <stereoFeatures.h>
#include <pinholeStereoCamera.h>
#include <auxiliar.h>
#include <linespec.h>

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

#define MAX_VAL 9999999999

//#define DEBUG_PRINT_OUT
#define TIMECOST_VERBOSE


namespace StVO{


// Structure for reporting time cost per module
struct TimeLog {
    double frame_time_stamp;
    //
    double time_track;
    double time_pt_extract;
    double time_ln_detect;
    double time_ln_descri;
    double time_pt_stereo;
    double time_ln_stereo;
    double time_pt_cross;
    double time_ln_cross;
    double time_ln_cut;
    double time_pose_optim;
    //
    double num_pt_detect;
    double num_ln_detect;
    double num_pt_stereo;
    double num_ln_stereo;
    double num_pt_cross;
    double num_ln_cross;
};


class StereoFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    StereoFrame();
    StereoFrame(const Mat & img_l_, const Mat & img_r_ , const int & idx_,
                PinholeStereoCamera *cam_, const double & time_stamp_);
    StereoFrame(const Mat & img_l_, const Mat & img_r_ , const int & idx_,
                PinholeStereoCamera *cam_,
                ORB_SLAM2::ORBextractor * orbExtractor_l_,
                ORB_SLAM2::ORBextractor * orbExtractor_r_,
                const double & time_stamp_);
    ~StereoFrame();

    void extractInitialStereoFeatures( int fast_th = 20 );
    void extractStereoFeatures_Brute( int fast_th = 20 );
    void extractStereoFeatures_ORBSLAM( int fast_th = 20 );
    //
    void subPixelStereoRefine_ORBSLAM(const cv::KeyPoint &kpL,
                                      const cv::KeyPoint &kpR,
                                      float & disparity,
                                      float & bestuR);
    //
    void detectFeatures( double min_line_length, int fast_th = 20, int isRight = 0);
    void detectPointFeatures( const Mat & img, int fast_th = 20, int isRight = 0 );
    void detectLineFeatures( const Mat & img, double min_line_length, int isRight = 0 );
    void matchPointFeatures(BFMatcher* bfm,
                            const Mat & pdesc_1, const Mat & pdesc_2,
                            vector<vector<DMatch>> &pmatches_12);
    void matchLineFeatures(BFMatcher* bfm,
                           const Mat & ldesc_1, const Mat & ldesc_2,
                           vector<vector<DMatch>> &lmatches_12 );
    void pointDescriptorMAD( const vector<vector<DMatch>> matches,
                             double &nn_mad, double &nn12_mad );
    void lineDescriptorMAD( const vector<vector<DMatch>> matches,
                            double &nn_mad, double &nn12_mad );
    //
    void matchPointFeatures_radius(BFMatcher* bfm,
                                   const Mat & pdesc_1, const Mat & pdesc_2,
                                   vector<vector<DMatch>> &pmatches_12);
    void matchLineFeatures_radius(BFMatcher* bfm,
                                  const Mat & ldesc_1, const Mat & ldesc_2,
                                  vector<vector<DMatch>> &lmatches_12 );
    //
    void pointDescriptorBudgetThres( const vector<vector<DMatch>> matches,
                                     double &thres_budget );
    void lineDescriptorBudgetThres( const vector<vector<DMatch>> matches,
                                    double &thres_budget );
    //
    //
    //
    /* line uncertainty modeling */
    void getJacob2D_3D(const double &u, const double &v, const double &d,
                       Matrix3d & jacMat);
    void getJacob3D_2D(const double &px, const double &py, const double &pz,
                       Matrix3d & jacMat);
    void getCovMat2D_3D(const double &u, const double &v, const double &u_std,
                        const double &d, const double &d_std, Matrix3d & covMat);
    void estimateStereoUncertainty();
    //
    /* util functions */
    void getLineCoeff(const Vector2d lsp,
                      const Vector2d lep,
                      Vector3d & coeff);
    bool isOutOfFrame(const int col, const int row);
    double pointToLineDist(const Vector3d l_coe,
                           const Vector2d pt);

    void projectPrev3DPoint(const Matrix4d & prev_Tfw,
                            const PointFeature * point_prev,
                            Vector2d & point_proj_l,
                            Vector2d & point_proj_r) ;
    void projectPrev3DLine(const Matrix4d & prev_Tfw,
                           const LineFeature * line_seg,
                           Vector2d & line_proj_sp,
                           Vector2d & line_proj_ep);
    bool withinLineSegment(const Vector2d lsp,
                           const Vector2d lep,
                           const Vector2d pt,
                           const double rng_included);
    bool withinPointWindow(const Vector2d pt_proj,
                           const double u,
                           const double v,
                           const double rng_included);

    void plotStereoFrameBoth( bool save_screen_shot );
    void plotStereoFrameLeft( bool save_screen_shot );

    double lineSegmentOverlapStereo(double spl_obs, double epl_obs,
                                    double spl_proj, double epl_proj );



    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int descriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }


    // DEBUG ONLY
    TimeLog log_;

    //
    double time_stamp;
    int frame_idx;
    Mat rgbImg_l, rgbImg_r;
    Mat gryImg_l, gryImg_r;
    Matrix4d Tfw;
    Matrix4d DT;

    Matrix6d Tfw_cov;
    Vector6d Tfw_cov_eig;
    double   entropy_first;

    Matrix6d DT_cov;
    Vector6d DT_cov_eig;
    double   err_norm;

    vector<PointFeature*> stereo_pt;
    vector<LineFeature*>  stereo_ls;

    vector<KeyPoint> points_l, points_r;
    vector<KeyLine>  lines_l, lines_r;
    Mat pdesc_l, pdesc_r, ldesc_l, ldesc_r;

    PinholeStereoCamera* cam;

    ORB_SLAM2::ORBextractor *  orbExtractor_l;
    ORB_SLAM2::ORBextractor *  orbExtractor_r;
    Ptr<line_descriptor::LSDDetectorC> lsdExtractor_l, lsdExtractor_r;
    Ptr<BinaryDescriptor>   lbdExtractor_l, lbdExtractor_r;
    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;
    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;
    //
//    int TH_HIGH;
//    int TH_LOW;
//    int HISTO_LENGTH;

    // viz & debug only
    vector<KeyLine>     image_ls_l, image_ls_r;
    vector<KeyPoint>    image_pt_l, image_pt_r;
    // canvas for viz
    cv::Mat stereo_frame_detect;
    cv::Mat stereo_frame_match;
    std::string save_path = "/mnt/DATA/tmp/LineCut/Gazebo/";
};

}
