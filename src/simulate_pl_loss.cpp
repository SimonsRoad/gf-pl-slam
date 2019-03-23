#include <stereoFrameHandler.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <ctime>

int main() {

    std::ofstream fout_loss_info;
    fout_loss_info.open("/home/yzhao/Eval/simulate_pl_loss_qZ.log",
                        ios::out);

    // scale up the loss terms so as to avoid precision loss
    // remember to devide it when computing gradient of loss
    double LOSS_SCALE_FAC = 10000;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float> (0,0) = 525.0;
    K.at<float> (0,2) = 319.5;
    K.at<float> (1,1) = 525.0;
    K.at<float> (1,2) = 239.5;

    cv::Mat D = cv::Mat::eye(1,5,CV_32F);
    D.at<float> (0,0) = float(0.0);
    D.at<float> (0,1) = float(0.0);
    D.at<float> (0,2) = float(0.0);
    D.at<float> (0,3) = float(0.0);
    D.at<float> (0,4) = float(0.0);

//    cv::Matx44f pose_l = cv::Matx44f::eye();
//    cv::Matx44f pose_r = cv::Matx44f::eye();
//    pose_r(0, 3) += 1.0;
//    PinholeStereoCamera* camera =
//            new PinholeStereoCamera (int(640),int(480),double(0.12),K,K,pose_l,pose_r,D,D);

    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
    t.at<float>(2, 0) = 1.0;
    PinholeStereoCamera* camera =
            new PinholeStereoCamera (int(640),int(480),double(0.12),K,K,R,t,D,D,false);

    StVO::StereoFrameHandler* Stvo =
            new StVO::StereoFrameHandler(camera);
    Stvo->viz_3D_lines = new cv::viz::Viz3d("3D Line Visualization");
    Stvo->viz_3D_lines->setWindowSize(cv::Size(800, 600));

    Stvo->setLeftCamExtrinCalib(0.0020257533, -0.0007686956, 0.0010088773,
                                0.0544501091, 0.0000497216, 0.0001748168);
    //    Stvo->setLeftCamExtrinCalib(0, 0, 0, 0, 0, 0);

    // set up random number generation
    std::srand(std::time(0)); // use current time as seed for random generator

    vector<vector<double>> map_res_p2l;
    vector<vector<double>> map_res_p2p;

    // start simulation
    int iter_num = 0;
    while (iter_num < 10000) {

        std::cout << "simulation round " << iter_num
                  << " =====================" << std::endl;

        vector<double> vec_res_p2l;
        vector<double> vec_res_p2p_s;
        vector<double> vec_res_p2p_e;
        //
        vector<double> vec_cov_p2l;
        vector<double> vec_cov_p2p_s;
        vector<double> vec_cov_p2p_e;
        //
        vector<double> vec_jac_p2l_s;
        vector<double> vec_jac_p2l_e;

        // create fake prev & curr frame
        cv::Mat img = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
        //        cv::imshow("input img", img);
        //        cv::waitKey(0);

        Stvo->prev_frame = new StVO::StereoFrame( img, img, iter_num, camera, 0 );
        Stvo->prev_frame->Tfw = Matrix4d::Identity();

        //        std::cout << "left_cam_T_rigframe = " << Stvo->left_cam_T_rigframe << std::endl;

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

        Vector3d sp_l_prev;
        sp_l_prev << sX, sY, 1.0;
        Vector3d ep_l_prev;
        ep_l_prev << eX, eY, 1.0;
        Vector3d le_l;
        le_l << sp_l_prev.cross(ep_l_prev);
        le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        // Normalization
        Vector3d le_l_prev;
        le_l_prev << le_l(0) * camera->getFx(),
                le_l(1) * camera->getFy(),
                le_l(2) + le_l(0)*camera->getCx() + le_l(1)*camera->getCy();
        le_l_prev = le_l_prev / sqrt(
                    le_l_prev(0)*le_l_prev(0) +
                    le_l_prev(1)*le_l_prev(1) +
                    le_l_prev(2)*le_l_prev(2) );
        //        // Normalized coordinates
        //        Vector2d sp_n;
        //        sp_n << sX,  sY;
        //        sp_n = camera->normalizedCoordinates(sp_n);
        //        Vector2d ep_n;
        //        ep_n << eX,  eY;
        //        ep_n = camera->normalizedCoordinates(ep_n);
        //        Vector3d sp_ln;
        //        sp_ln << sp_n(0), sp_n(1), 1.0;
        //        Vector3d ep_ln;
        //        ep_ln << ep_n(0), ep_n(1), 1.0;
        //        Vector3d le_n;
        //        le_n << sp_ln.cross(ep_ln);
        //        le_n = le_n / sqrt( le_n(0)*le_n(0) + le_n(1)*le_n(1) );

        Vector3d sP_;
        sP_ = camera->backProjection( sp_l_prev(0), sp_l_prev(1), sD);
        Vector3d eP_;
        eP_ = camera->backProjection( ep_l_prev(0), ep_l_prev(1), eD);
        Vector3d sp_cam_prev;
        sp_cam_prev << sP_(0)/sP_(2), sP_(1)/sP_(2), 1.0;
        Vector3d ep_cam_prev;
        ep_cam_prev << eP_(0)/eP_(2), eP_(1)/eP_(2), 1.0;

        Stvo->prev_frame->stereo_ls.push_back( new StVO::LineFeature(
                                                   Vector2d(sp_l_prev(0),sp_l_prev(1)),
                                                   sD,
                                                   sP_,
                                                   Vector2d(ep_l_prev(0),ep_l_prev(1)),
                                                   eD,
                                                   eP_,
                                                   le_l_prev // le_l // le_n //
                                                   ) );
        Stvo->prev_frame->estimateStereoUncertainty();

        // iterative pose error along all 6 dimensions
        double tRange = 0.05; // 0.1; // 0.5;
        double qRange = 0.1; // 0.2; // 0.5;
        double tX = 0, tY = 0, tZ = 0,
                qX = 0, qY = 0, qZ = 0, qW = 0;
        for (int ii=-20; ii<=20; ++ii) {

            Stvo->curr_frame = new StVO::StereoFrame( img, img, iter_num, camera, 1 );
            Stvo->curr_frame->Tfw = Matrix4d::Identity();

            // map the iteration indices into pose error
            tX = double(ii) / double(20) * tRange;
            //            tY = double(ii) / double(20) * tRange;
            //            tZ = double(ii) / double(20) * tRange;
            //            qX = double(ii) / double(20) * qRange;
            //            qY = double(ii) / double(20) * qRange;
            //            qZ = double(ii) / double(20) * qRange;
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
            //            std::cout << "randomized pose param: " << std::endl
            //                      << "<tX, tY, tz> = " << tX << ", " << tY << ", " << tZ << "; " << std::endl
            //                      << "<qW, qX, qY, qZ> = " << qW << ", " << qX << ", "
            //                      << qY << ", " << qZ << "; " << std::endl
            //                      << "rel_Tfw = " << rel_Tfw << std::endl;

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
                        le_curr_n(1)*le_curr_n(1) +
                        le_curr_n(2)*le_curr_n(2) );

            // insert projected line to curr frame
            Stvo->curr_frame->stereo_ls.push_back( new StVO::LineFeature(
                                                       Vector2d(sp_l_curr(0),sp_l_curr(1)),
                                                       sdisp_curr,
                                                       sP_cur,
                                                       Vector2d(ep_l_curr(0),ep_l_curr(1)),
                                                       edisp_curr,
                                                       eP_cur,
                                                       le_curr_n // le_curr //
                                                       ) );

            //            std::cout << "prev: " << sp_l_prev.transpose() << "; "
            //                      << ep_l_prev.transpose() << std::endl;
            //            std::cout << "curr: " << sp_l_curr.transpose() << "; "
            //                      << ep_l_curr.transpose() << std::endl;
            //            std::cout << "stereo" << std::endl;

            // insert the cross-frame matching
            StVO::LineFeature* line_ = Stvo->prev_frame->stereo_ls[0];
            line_->sdisp_obs = sdisp_curr;
            line_->edisp_obs = edisp_curr;
            line_->spl_obs = spl_curr;
            line_->epl_obs = epl_curr;
            line_->le_obs  = le_curr;
            line_->inlier  = true;
            Stvo->matched_ls.push_back( line_ );
            //
            cv::DMatch match;
            match.queryIdx = 0;
            match.trainIdx = 0;
            match.distance = 0;
            Stvo->matches.push_back(match);

            //            std::cout << "match" << std::endl;
            Stvo->estimateProjUncertainty();

            // compute the endpoint to line distance
            Vector3d sp_cam_curr;
            sp_cam_curr << sP_cur(0)/sP_cur(2), sP_cur(1)/sP_cur(2), 1.0;
            Vector3d ep_cam_curr;
            ep_cam_curr << eP_cur(0)/eP_cur(2), eP_cur(1)/eP_cur(2), 1.0;
            //            double res_spt_l = pow(le_l.dot(sp_l_curr / sp_l_curr(2)), 2);
            //            double res_ept_l = pow(le_l.dot(ep_l_curr / ep_l_curr(2)), 2);
            double res_spt_l = le_l_prev.dot(sp_cam_curr);
            double res_ept_l = le_l_prev.dot(ep_cam_curr);
            double res_p2l = (pow(res_spt_l, 2) + pow(res_ept_l, 2)) * LOSS_SCALE_FAC;

            // compute the endpoint to endpoint distance
            double res_spt_p = (sp_cam_curr - sp_cam_prev).squaredNorm() * LOSS_SCALE_FAC;
            double res_ept_p = (ep_cam_curr - ep_cam_prev).squaredNorm() * LOSS_SCALE_FAC;

            vec_res_p2l.push_back(res_p2l);
//            vec_res_p2l_e.push_back(res_ept_l); * LOSS_SCALE_FAC
            //
            vec_res_p2p_s.push_back(res_spt_p);
            vec_res_p2p_e.push_back(res_ept_p);

            // compute the cov of residual
            for( list<StVO::LineFeature*>::iterator it = Stvo->matched_ls.begin();
                 it != Stvo->matched_ls.end(); it ++) {
                //
                int fst_idx = 0, lst_idx = (*it)->lPt2D.size() - 1;
                Vector3d Jpp;
                Jpp << 2*(sp_cam_curr(0)-sp_cam_prev(0)), -2*(sp_cam_curr(1)-sp_cam_prev(1)), 1;
                vec_cov_p2p_s.push_back( (Jpp.transpose() * (*it)->covlPt2D[fst_idx] * Jpp)(0, 0) * LOSS_SCALE_FAC );
                vec_cov_p2p_e.push_back( (Jpp.transpose() * (*it)->covlPt2D[lst_idx] * Jpp)(0, 0) * LOSS_SCALE_FAC );
                //
                Vector3d J_loss;
                J_loss(0) = le_l_prev[0];
                J_loss(1) = le_l_prev[1];
                J_loss(2) = le_l_prev[2];
                Vector3d Jlsp = 2 * (J_loss.transpose() * sp_cam_curr)(0,0) * J_loss;
                Vector3d Jlep = 2 * (J_loss.transpose() * ep_cam_curr)(0,0) * J_loss;
                double var2D_spt = Jlsp.transpose() * (*it)->covlPt2D[fst_idx] * Jlsp;
                double var2D_ept = Jlep.transpose() * (*it)->covlPt2D[lst_idx] * Jlep;
                vec_cov_p2l.push_back( (var2D_spt + var2D_ept) * LOSS_SCALE_FAC );
            }


            Matrix<double, 6, 1> Jac_spt_l;
            Matrix<double, 6, 1> Jac_ept_l;
            pointPair2LineJacobian(rigframe_T_world,
                                   Stvo->left_cam_T_rigframe,
                                   le_l_prev,
                                   sP_,
                                   eP_,
                                   Jac_spt_l,
                                   Jac_ept_l);
            if (le_l_prev.dot(sp_cam_curr) > 0) {
                Jac_spt_l = Jac_spt_l * (-1);
            }
            if (le_l_prev.dot(ep_cam_curr) > 0) {
                Jac_ept_l = Jac_ept_l * (-1);
            }

            double jac_spt_l = Jac_spt_l(3);
            double jac_ept_l = Jac_ept_l(3);

            //            std::cout << Jac_spt_l.transpose() << std::endl;

            vec_jac_p2l_s.push_back(jac_spt_l);
            vec_jac_p2l_e.push_back(jac_ept_l);

            //            std::cout << "residual" << std::endl;

            //            // viz
            //            //        rel_Tfw = Matx<float,4,4>::eye();
            //            Stvo->plotMatchedLines(rel_Tfw.inv(), img, false);
            //            cv::imshow("line uncertainty propagation", Stvo->canvas_match_frames_old);
            //            cv::waitKey(0);
            //        Stvo->plotLine3D();
//            Stvo->plotLine3DCrossFrame();

            // recycle everything for next round
            Stvo->matched_ls.clear();
            delete Stvo->curr_frame;
        }

//        map_res_p2l.push_back(vec_res_p2l);
//        //
//        map_res_p2p.push_back(vec_res_p2p_s);
//        map_res_p2p.push_back(vec_res_p2p_e);

        // print out
//        std::cout << "true_grad_p2l: ";
//        for (int i=0; i<vec_res_p2l_s.size(); ++i) {
//            double h = double(i-20) / double(20) * tRange;
//            std::cout << (vec_res_p2l_s[i] - vec_res_p2l_s[20]) / (h * LOSS_SCALE_FAC) << " ";
//        }
//        std::cout << std::endl;
//        std::cout << "estm_grad_p2l: ";
//        for (int i=0; i<vec_jac_p2l_s.size(); ++i) {
//            std::cout << vec_jac_p2l_s[i] << " ";
//        }
//        std::cout << std::endl;
//        //
//        std::cout << "true_grad_p2l: ";
//        for (int i=0; i<vec_res_p2l_e.size(); ++i) {
//            double h = double(i-20) / double(20) * qRange;
//            std::cout << (vec_res_p2l_e[i] - vec_res_p2l_e[20]) / h << " ";
//        }
//        std::cout << std::endl;
//        std::cout << "estm_grad_p2l: ";
//        for (int i=0; i<vec_jac_p2l_e.size(); ++i) {
//            std::cout << vec_jac_p2l_e[i] << " ";
//        }
//        std::cout << std::endl;

        std::cout << "var of p2p loss: ";
        fout_loss_info << 30 << " " << sP_(0) << " " << sP_(1) << " " << sP_(2) << " ";
        for (int i=0; i<vec_cov_p2p_s.size(); ++i) {
            std::cout << vec_cov_p2p_s[i] << " ";
            fout_loss_info << vec_cov_p2p_s[i] << " ";
        }
        std::cout << std::endl;
        fout_loss_info << std::endl;
        //
        fout_loss_info << 30 << " " << sP_(0) << " " << sP_(1) << " " << sP_(2) << " ";
        for (int i=0; i<vec_cov_p2p_e.size(); ++i) {
            std::cout << vec_cov_p2p_e[i] << " ";
            fout_loss_info << vec_cov_p2p_e[i] << " ";
        }
        std::cout << std::endl;
        fout_loss_info << std::endl;
        //
        std::cout << "var of p2l loss: ";
        fout_loss_info << 31 << " " << sP_(0) << " " << sP_(1) << " " << sP_(2) << " ";
        for (int i=0; i<vec_cov_p2l.size(); ++i) {
            std::cout << vec_cov_p2l[i] << " ";
            fout_loss_info << vec_cov_p2l[i] << " ";
        }
        std::cout << std::endl;
        fout_loss_info << std::endl;

        std::cout << "res_p2l: ";
        fout_loss_info << 1 << " " << sP_(0) << " " << sP_(1) << " " << sP_(2) << " ";
        for (int i=0; i<vec_res_p2l.size(); ++i) {
            std::cout << vec_res_p2l[i] << " ";
            fout_loss_info << vec_res_p2l[i] << " ";
        }
        std::cout << std::endl;
        fout_loss_info << std::endl;
        //
        std::cout << "res_p2p: ";
        fout_loss_info << 0 << " " << sP_(0) << " " << sP_(1) << " " << sP_(2) << " ";
        for (int i=0; i<vec_res_p2p_s.size(); ++i) {
            std::cout << vec_res_p2p_s[i] << " ";
            fout_loss_info << vec_res_p2p_s[i] << " ";
        }
        std::cout << std::endl;
        fout_loss_info << std::endl;
        //
        std::cout << "res_p2p: ";
        fout_loss_info << 0 << " " << eP_(0) << " " << eP_(1) << " " << eP_(2) << " ";
        for (int i=0; i<vec_res_p2p_e.size(); ++i) {
            std::cout << vec_res_p2p_e[i] << " ";
            fout_loss_info << vec_res_p2p_e[i] << " ";
        }
        std::cout << std::endl;
        fout_loss_info << std::endl;

        // recycle everything for next round
        Stvo->matched_ls.clear();
        delete Stvo->prev_frame;
        iter_num ++;
    }
}

