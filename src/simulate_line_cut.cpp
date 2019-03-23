#include <stereoFrameHandler.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <ctime>

int main() {

    std::ofstream fout_vol_info;
    fout_vol_info.open("/home/yipuzhao/Codes/VSLAM/GF_PL_SLAM/simu/simulate_line_vol.log",
                       ios::out);

    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    K.at<double> (0,0) = 525.0;
    K.at<double> (0,2) = 319.5;
    K.at<double> (1,1) = 525.0;
    K.at<double> (1,2) = 239.5;

    cv::Mat D = cv::Mat::eye(1,5,CV_64F);
    D.at<double> (0,0) = 0.0;
    D.at<double> (0,1) = 0.0;
    D.at<double> (0,2) = 0.0;
    D.at<double> (0,3) = 0.0;
    D.at<double> (0,4) = 0.0;

    //    cv::Matx44f pose_l = cv::Matx44f::eye();
    //    cv::Matx44f pose_r = cv::Matx44f::eye();
    //    pose_r(0, 3) += 1.0;
    //    PinholeStereoCamera* camera =
    //            new PinholeStereoCamera (int(640),int(480),double(0.12),K,K,pose_l,pose_r,D,D);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    t.at<double>(0, 0) = -0.12;
    PinholeStereoCamera* camera =
            new PinholeStereoCamera (int(640),int(480),double(0.12),K,K,R,t,D,D,false);

    StVO::StereoFrameHandler* Stvo =
            new StVO::StereoFrameHandler(camera);
    Stvo->viz_3D_lines = new cv::viz::Viz3d("3D Line Visualization");
    Stvo->viz_3D_lines->setWindowSize(cv::Size(800, 600));

    //    Stvo->setLeftCamExtrinCalib(0.0020257533, -0.0007686956, 0.0010088773,
    //                                0.0544501091, 0.0000497216, 0.0001748168);
    Stvo->setLeftCamExtrinCalib(0, 0, 0, 0, 0, 0);

    std::cout << "left_cam_T_rigframe = " << Stvo->left_cam_T_rigframe << std::endl;

    // set up random number generation
    std::srand(std::time(0)); // use current time as seed for random generator

    // start simulation
    int iter_num = 0;
    while (iter_num < 1000) {

        std::cout << "simulation round " << iter_num
                  << " =====================" << std::endl;

        // create fake prev & curr frame
        cv::Mat img = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
        //        cv::imshow("input img", img);
        //        cv::waitKey(0);

        Stvo->prev_frame = new StVO::StereoFrame( img, img, iter_num, camera, 0 );
        Stvo->prev_frame->Tfw = Matrix4d::Identity();
        Stvo->curr_frame = new StVO::StereoFrame( img, img, iter_num, camera, 1 );
        Stvo->curr_frame->Tfw = Matrix4d::Identity();

        // insert randomized line to prev_frame
        double sX, sY, sD, eX, eY, eD;
        sX = double(std::rand()) / double(RAND_MAX) * 640.0;
        sY = double(std::rand()) / double(RAND_MAX) * 480.0;
        sD = double(std::rand()) / double(RAND_MAX) * 100.0;
        eX = double(std::rand()) / double(RAND_MAX) * 640.0;
        eY = double(std::rand()) / double(RAND_MAX) * 480.0;
        eD = double(std::rand()) / double(RAND_MAX) * 100.0;

        std::cout << "randomized line param: " << std::endl
                  << "<sX, sY, sD> = " << sX << ", " << sY << ", " << sD << "; " << std::endl
                  << "<eX, eY, eD> = " << eX << ", " << eY << ", " << eD << "; " << std::endl;

        Vector3d sp_l;
        sp_l << sX, sY, 1.0;
        Vector3d ep_l;
        ep_l << eX, eY, 1.0;
        Vector3d le_l;
        le_l << sp_l.cross(ep_l);
        le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        // Normalization
        Vector3d le_prev_n;
        le_prev_n << le_l(0) * camera->getFx(),
                le_l(1) * camera->getFy(),
                le_l(2) + le_l(0)*camera->getCx() + le_l(1)*camera->getCy();
        le_prev_n = le_prev_n / sqrt(
                    le_prev_n(0)*le_prev_n(0) +
                    le_prev_n(1)*le_prev_n(1) );
        Vector3d sP_;
        sP_ = camera->backProjection( sp_l(0), sp_l(1), sD);
        Vector3d eP_;
        eP_ = camera->backProjection( ep_l(0), ep_l(1), eD);

        std::cout << "Prev back-projected 3D point-pair: " << sP_ << "; " << eP_ << std::endl;

        Stvo->prev_frame->stereo_ls.push_back( new StVO::LineFeature(
                                                   Vector2d(sp_l(0),sp_l(1)),
                                                   sD,
                                                   sP_,
                                                   Vector2d(ep_l(0),ep_l(1)),
                                                   eD,
                                                   eP_,
                                                   le_prev_n  //  le_l_test
                                                   ) );
        Stvo->prev_frame->estimateStereoUncertainty();

        // randomize pose transform between prev & curr frame
        double tRange = 0.1; // 0.5;
        double qRange = 0.2; // 0.5;
        double tX, tY, tZ, qX, qY, qZ, qW;
        tX = double(std::rand()) / double(RAND_MAX) * 2 * tRange - tRange;
        tY = double(std::rand()) / double(RAND_MAX) * 2 * tRange - tRange;
        tZ = double(std::rand()) / double(RAND_MAX) * 2 * tRange - tRange;
        qX = double(std::rand()) / double(RAND_MAX) * 2 * qRange - qRange;
        qY = double(std::rand()) / double(RAND_MAX) * 2 * qRange - qRange;
        qZ = double(std::rand()) / double(RAND_MAX) * 2 * qRange - qRange;
        qW = sqrt(1 - pow(qX, 2) - pow(qY, 2) - pow(qZ, 2));

        // create pose transform matrix accordingly
        Quaterniond q(qW, qX, qY, qZ);
        Matrix3d rMat = q.normalized().toRotationMatrix();
        //        cv::Matx33d cv::Matx33d;
        //        CopyEigenToMatX(rMat, cvRMat);

        cv::Matx44f rel_Tfw = Matx<float,4,4>::eye();
        for (int ii=0; ii<3; ++ii)
            for (int jj=0; jj<3; ++jj)
                rel_Tfw(ii,jj) = rMat(ii,jj);
        rel_Tfw(0,3) = tX;
        rel_Tfw(1,3) = tY;
        rel_Tfw(2,3) = tZ;
        std::cout << "randomized pose param: " << std::endl
                  << "<tX, tY, tz> = " << tX << ", " << tY << ", " << tZ << "; " << std::endl
                  << "<qW, qX, qY, qZ> = " << qW << ", " << qX << ", "
                  << qY << ", " << qZ << "; " << std::endl
                  << "rel_Tfw = " << rel_Tfw << std::endl;

        Stvo->setFramePose(rel_Tfw);

        Matrix<double, 6, 1> rigframe_T_world;
        rigframe_T_world << qX, qY, qZ, rel_Tfw(0,3), rel_Tfw(1,3), rel_Tfw(2,3);

        // project the 3D random line to curr frame
        Vector4d spt_4d; spt_4d << sP_, 1;
        Vector4d ept_4d; ept_4d << eP_, 1;
        spt_4d = Stvo->curr_frame->Tfw.inverse() * spt_4d;
        ept_4d = Stvo->curr_frame->Tfw.inverse() * ept_4d;
        //
        Vector3d sP_cur(spt_4d(0), spt_4d(1), spt_4d(2));
        Vector3d eP_cur(ept_4d(0), ept_4d(1), ept_4d(2));
        Vector2d spl_curr = camera->projection(sP_cur);
        double sdisp_curr = camera->getDisparity(sP_cur(2));
        Vector2d epl_curr = camera->projection(eP_cur);
        double edisp_curr = camera->getDisparity(eP_cur(2));
        //
        //        sp_l << spl_curr(0), spl_curr(1), 1.0;
        //        ep_l << epl_curr(0), epl_curr(1), 1.0;
        Vector3d sp_l_curr;
        sp_l_curr << spl_curr(0), spl_curr(1), 1.0;
        Vector3d ep_l_curr;
        ep_l_curr << epl_curr(0), epl_curr(1), 1.0;

        Vector3d le_curr;
        le_curr << sp_l_curr.cross(ep_l_curr);
        le_curr = le_curr / sqrt( le_curr(0)*le_curr(0) + le_curr(1)*le_curr(1) );
        Vector3d le_curr_n;
        le_curr_n << le_curr(0) * camera->getFx(),
                le_curr(1) * camera->getFy(),
                le_curr(2) + le_curr(0)*camera->getCx() + le_curr(1)*camera->getCy();
        le_curr_n = le_curr_n / sqrt(
                    le_curr_n(0)*le_curr_n(0) +
                    le_curr_n(1)*le_curr_n(1) );

        // insert projected line to curr frame
        Stvo->curr_frame->stereo_ls.push_back( new StVO::LineFeature(
                                                   Vector2d(sp_l_curr(0),sp_l_curr(1)),
                                                   sdisp_curr,
                                                   sP_cur,
                                                   Vector2d(ep_l_curr(0),ep_l_curr(1)),
                                                   edisp_curr,
                                                   eP_cur,
                                                   le_curr_n
                                                   ) );


        // insert the cross-frame matching
        StVO::LineFeature* line_ = Stvo->prev_frame->stereo_ls[0];
        line_->sdisp_obs = sdisp_curr;
        line_->edisp_obs = edisp_curr;
        line_->spl_obs = spl_curr;
        line_->epl_obs = epl_curr;
        line_->le_obs  = le_curr_n;
        line_->inlier  = true;
        Stvo->matched_ls.push_back( line_ );
        //
        cv::DMatch match;
        match.queryIdx = 0;
        match.trainIdx = 0;
        match.distance = 0;
        Stvo->matches.push_back(match);

        // model the uncertainty of sampled points along the projected line
        Stvo->estimateProjUncertainty();

        // print out
        vector<double> vol_line_pts;
        for( list<StVO::LineFeature*>::iterator it = Stvo->matched_ls.begin();
             it != Stvo->matched_ls.end(); it ++) {
            int fst_idx = 0, lst_idx =(*it)->lPt2D.size() - 1;

            //            while (fst_idx < lst_idx-1) {
            //                vector<double> eigen_;
            //                Matrix<double, 6, 6> cov_pose_inv_;
            //                Stvo->getPoseCovMatrixOnLine(fst_idx, lst_idx, rigframe_T_world, it, cov_pose_inv_);
            //                getEigenValue( cov_pose_inv_, eigen_ );
            //                //
            //                double prod_ = eigen_[0] * eigen_[1];
            //                std::cout << prod_ << " ";
            //                vol_line_pts.push_back(prod_);

            //                fst_idx ++;

            //                Stvo->getPoseCovMatrixOnLine(fst_idx, lst_idx, rigframe_T_world, it, cov_pose_inv_);
            //                getEigenValue( cov_pose_inv_, eigen_ );
            //                //
            //                prod_ = eigen_[0] * eigen_[1];
            //                std::cout << prod_ << " ";
            //                vol_line_pts.push_back(prod_);

            //                lst_idx --;
            //            }
            //            std::cout << std::endl;

            while (fst_idx < lst_idx-1) {
                vector<double> eigen_;
                Matrix<double, 6, 6> cov_pose_inv_;
                Stvo->getPoseCovMatrixOnLine(fst_idx, lst_idx, rigframe_T_world, it, cov_pose_inv_);
                getEigenValue( cov_pose_inv_, eigen_ );
                //
                double prod_ = eigen_[0] * eigen_[1];
                std::cout << prod_ << " ";
                vol_line_pts.push_back(prod_);

                fst_idx ++;
            }
            std::cout << std::endl;
            //
            fst_idx = 0;
            while (fst_idx < lst_idx-1) {
                vector<double> eigen_;
                Matrix<double, 6, 6> cov_pose_inv_;
                Stvo->getPoseCovMatrixOnLine(fst_idx, lst_idx, rigframe_T_world, it, cov_pose_inv_);
                getEigenValue( cov_pose_inv_, eigen_ );
                //
                double prod_ = eigen_[0] * eigen_[1];
                std::cout << prod_ << " ";
                vol_line_pts.push_back(prod_);

                lst_idx --;
            }
            std::cout << std::endl;
        }

        // save 3D line
        fout_vol_info << sP_(0) << " " << sP_(1) << " " << sP_(2) << " ";
        fout_vol_info << eP_(0) << " " << eP_(1) << " " << eP_(2) << " ";
        // save relative pose
        fout_vol_info << tX << " " << tY << " " << tZ << " ";
        fout_vol_info << qW << " " << qX << " " << qY << " " << qZ << " ";
        // save volume along line
        for (int i=0; i<vol_line_pts.size(); ++i) {
            fout_vol_info << vol_line_pts[i] << " ";
        }
        fout_vol_info << std::endl;

        // viz
        //        rel_Tfw = Matx<float,4,4>::eye();
        Stvo->plotMatchedLines(rel_Tfw.inv(), img, true);
        cv::imshow("line uncertainty propagation", Stvo->canvas_match_frames_old);
//        cv::waitKey(0);
        //        Stvo->plotLine3D();
        Stvo->plotLine3DCrossFrame();

        // recycle everything for next round
        Stvo->matched_ls.clear();
        delete Stvo->prev_frame;
        delete Stvo->curr_frame;
        iter_num ++;
    }
}

