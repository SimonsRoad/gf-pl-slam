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

#include <stereoFrameHandler.h>


namespace StVO{

// comparison, not case sensitive.
bool compare_line_angle (const LineFeature * first, const LineFeature * second) {
    // convert angle from -180 180 to 0 180
    float ang1 = first->angle;
    //    if ( ang1 < 0 )
    //        ang1 += 180.0;
    //
    float ang2 = second->angle;
    //    if ( ang2 < 0 )
    //        ang2 += 180.0;
    //
    return ( ang1 < ang2 );
}

StereoFrameHandler::StereoFrameHandler( PinholeStereoCamera *cam_ ) : cam(cam_) {}

StereoFrameHandler::~StereoFrameHandler(){}

void StereoFrameHandler::initialize(const Mat & img_l_, const Mat & img_r_ ,
                                    const int idx_, double time_stamp_)
{
    prev_frame = new StereoFrame( img_l_, img_r_, idx_, cam, time_stamp_ );
    prev_frame->extractInitialStereoFeatures();
    prev_frame->Tfw = Matrix4d::Identity();
    prev_frame->Tfw_cov = Matrix6d::Identity();
    prev_frame->DT  = Matrix4d::Identity();
    // variables for adaptative FAST
    orb_fast_th = Config::orbFastTh();
    // SLAM variables for KF decision
    T_prevKF         = Matrix4d::Identity();
    cov_prevKF_currF = Matrix6d::Zero();
    prev_f_iskf      = true;

    this->setLeftCamExtrinCalib(0, 0, 0, 0, 0, 0);

    // init orb extractor

    //    orbExtractor = ORB::create( Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
    //                                Config::orbEdgeTh(), 0, Config::orbWtaK(), Config::orbScore(),
    //                                Config::orbPatchSize(), Config::orbFastTh() );

    //    std::cout << "ORB extractor to be init!" << std::endl;

    mpORBextractor_l = new ORB_SLAM2::ORBextractor(Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
                                                   20, 7);
    mpORBextractor_r = new ORB_SLAM2::ORBextractor(Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
                                                   20, 7);

    //    std::cout << "ORB extractor initialzed!" << std::endl;

    //
    // Create a window
    viz_3D_lines = new cv::viz::Viz3d("3D Line Visualization");
    viz_3D_lines->setWindowSize(cv::Size(800, 600));
}

void StereoFrameHandler::insertStereoPair(const Mat & img_l_, const Mat & img_r_,
                                          const int idx_, double time_stamp_) {
    bool print_debug_info = true; // false; //
    clock_t time_st, time_ed;
    //
    time_st = clock();
    curr_frame = new StereoFrame( img_l_, img_r_, idx_, cam, mpORBextractor_l, mpORBextractor_r, time_stamp_ );
//    curr_frame->extractStereoFeatures_Brute( orb_fast_th );
    curr_frame->extractStereoFeatures_ORBSLAM( orb_fast_th );
    // viz
    //    curr_frame->plotStereoFrameBoth( true );
    time_ed = clock();
    if (print_debug_info)
        std::cout << "func insertStereoPair: time cost to extract stereo features = "
                  << float(time_ed - time_st) / CLOCKS_PER_SEC << std::endl;

    // predict Tfw of current frame with constant vel model
    predictFramePose();

    time_st = clock();
    if (Config::useLineConfCut() == true) {
        // assess the uncertainty of stereo from previous frame
        prev_frame->estimateStereoUncertainty();
    }
    //
    // crossFrameMatching_Brute();
    crossFrameMatching_Hybrid();
    //    crossFrameMatching_Proj();
    time_ed = clock();
    if (print_debug_info)
        std::cout << "func insertStereoPair: time cost to match features cross-frame = "
                  << float(time_ed - time_st) / CLOCKS_PER_SEC << std::endl;

            #ifdef DO_VIZ
    plotMatchedPointLine(false);
#endif

    if (Config::useLineConfCut() == true) {
        time_st = clock();
        // propogate the uncertainty to current frame
        //        estimateProjUncertainty_greedy();
        //        if (Config::hasPoints() == false && Config::hasLines() == true)
        //            // for line-only case, it turned out that the unbounded gradient descend
        //            // per line works much better than the theoretical submodular solver
        //            estimateProjUncertainty_descent( 0.05, rngCutRatio );
        //        else
        //            // with additional info from point features, submodular solver clearly
        //            // out-performs other line cutting approach; future work would definitely
        //            // involve the comparison between line-only and point + line
        //            estimateProjUncertainty_submodular( 0.05, rngCutRatio );

        double rngCutRatio[2] = {0, 1.0};
        estimateProjUncertainty_submodular( 0.05, rngCutRatio );
        
//        double rngCutRatio[2] = {-0.5, 1.5};
//        estimateProjUncertainty_submodular( 0.05, rngCutRatio );

        time_ed = clock();
        curr_frame->log_.time_ln_cut = float(time_ed - time_st) / CLOCKS_PER_SEC;
        // if (print_debug_info)
        std::cout << "Time cost to propagate uncertainty along lines = "
                  << curr_frame->log_.time_ln_cut * 1000.0
                  << " ms" << std::endl;
    }
    // for viz & debug
    //    this->plotLine3D( true );

    numFrameSinceKeyframe ++;
}

void StereoFrameHandler::predictFramePose() {
    //
    assert(curr_frame != NULL && prev_frame != NULL);
    // predict the motion of current frame
    curr_frame->Tfw    = prev_frame->Tfw * prev_frame->DT;

    //    std::cout << prev_frame->DT << std::endl;

    //    if (print_frame_info)
    //        std::cout << "func predictFramePose: current frame pose pred. = " << std::endl
    //                  << curr_frame->Tfw << std::endl;
    //
    // NOTE
    // for some reason Tfw has to be print out to be valid; not sure what happend
    //
    //    std::cout << "func predictFramePose: current frame pose pred. = " << std::endl
    //              << curr_frame->Tfw << std::endl;
}

void StereoFrameHandler::setFramePose(const cv::Matx44f Tfw_curr) {
    // predict the motion of current frame
    for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j)
            curr_frame->Tfw(i, j) = double( Tfw_curr(i, j) );
    //    cv::Mat temp = cv::Mat(Tfw_curr);
    //    Matrix4d temp_eig;
    //    temp.convertTo(temp,CV_64F);
    //    cv2eigen(temp,temp_eig);
    //    curr_frame->Tfw = temp_eig;
}

//
// NOTE
// the main body of this function should be transfered to map-to-frame matching easily
// the general input would be: a set of 3D lines (from stereo triangulation or from 3D map), and
// a set of lines being detected and matched from current stereo frame;
// then the objective would be: find the matching between the 2 input sets
//
void StereoFrameHandler::crossFrameMatching_Proj()
{
    // points f2f tracking
    matched_pt.clear();
    if( Config::hasPoints() && !(curr_frame->stereo_pt.size()==0) && !(prev_frame->stereo_pt.size()==0)  )
    {
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );    // cross-check
        Mat pdesc_l1, pdesc_l2;
        vector<vector<DMatch>> pmatches_12; // , pmatches_21;
        // 12 and 21 matches
        pdesc_l1 = prev_frame->pdesc_l;
        pdesc_l2 = curr_frame->pdesc_l;

        bfm->radiusMatch( pdesc_l1, pdesc_l2, pmatches_12, Config::pointMatchRadius() );
        //        bfm->radiusMatch( pdesc_l2, pdesc_l1, pmatches_21, Config::pointMatchRadius() );

        std::multimap<int, std::pair<int, float>> pair_point_match;
        for ( int i = 0; i < pmatches_12.size(); i++ ) {

            if ((pmatches_12[i]).size() <= 0)
                continue ;

            int p12_qdx = pmatches_12[i][0].queryIdx;
            //            if (prev_frame->left_sel[l12_qdx].frame_matched == true) {
            //                continue ;
            //            }

            Vector2d point_proj_l, point_proj_r;
            curr_frame->projectPrev3DPoint(prev_frame->Tfw,
                                           prev_frame->stereo_pt[p12_qdx],
                                           point_proj_l,
                                           point_proj_r);
            // instead of enforcing bidirectional best match between left and right,
            // top-K matches are tested to generate more matches
            double rng_included = 20.0; // 10.0; // 40.0; //
            for (int j=0; j<(pmatches_12[i]).size(); ++j) {
                assert(pmatches_12[i][j].queryIdx == p12_qdx);
                //
                int p12_tdx = pmatches_12[i][j].trainIdx;

                if (curr_frame->withinPointWindow(point_proj_l,
                                                  curr_frame->stereo_pt[p12_tdx]->pl(0),
                                                  curr_frame->stereo_pt[p12_tdx]->pl(1),
                                                  rng_included) == false) {
                    continue ;
                }

                if (curr_frame->withinPointWindow(point_proj_r,
                                                  curr_frame->stereo_pt[p12_tdx]->pl(0) + curr_frame->stereo_pt[p12_tdx]->disp,
                                                  curr_frame->stereo_pt[p12_tdx]->pl(1),
                                                  rng_included) == false) {
                    continue ;
                }

                // add distance check
                if (pmatches_12[i][j].distance > Config::pointMatchRadius() * 0.8)
                    continue ;

                std::pair<int,float> current_match(p12_qdx, pmatches_12[i][j].distance);
                pair_point_match.insert(std::pair<int,pair<int,float>>(p12_tdx, current_match));
                //
                break ;
            }
        }
        //
        auto x = pair_point_match.begin();
        while(pair_point_match.size() > 0 ) {

            std::multimap<int,pair<int,float>>::const_iterator it  = pair_point_match.lower_bound(x->first);
            std::multimap<int,pair<int,float>>::const_iterator it2 = pair_point_match.upper_bound(x->first);
            float min_value = 1e+7;
            int key = x->first;
            int matched_previous_point = 0;

            while (it !=it2 ) {
                std::pair<int, float> value = it->second;
                if(value.second < min_value) {
                    min_value = value.second;
                    matched_previous_point = value.first;
                }
                ++it;
            }
            size_t duplicates = pair_point_match.count(x->first);
            //std::cout<<"the number of duplocates are"<<pair_matching.count(x->first)<<std::endl;
            pair_point_match.erase(x->first);
            advance(x,duplicates);
            int l12_qdx = matched_previous_point;
            int l12_tdx = key;

            PointFeature* point_ = prev_frame->stereo_pt[l12_qdx];
            point_->pl_obs = curr_frame->stereo_pt[l12_tdx]->pl;
            point_->inlier = true;
            matched_pt.push_back( point_ );
            curr_frame->stereo_pt[l12_tdx]->idx = prev_frame->stereo_pt[l12_qdx]->idx; // prev idx

            //            // if the matching is accepted
            //            // kick the matched points out of following stereo match
            //            prev_frame->left_sel[l12_qdx].frame_matched = true;
            //            curr_frame->left_sel[l12_tdx].frame_matched = true;

            // enforce a upper bound on number of features matched across two frames
            if ( matched_pt.size() >= Config::maxPointMatchNum() ) {
                // force exit
                break ;
            }
        }
    }

    // line segments f2f tracking
    matched_ls.clear();
    if( Config::hasLines() && !(curr_frame->stereo_ls.size()==0) && !(prev_frame->stereo_ls.size()==0)  )
    {
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );    // cross-check
        Mat ldesc_l1, ldesc_l2;
        vector<vector<DMatch>> lmatches_12; // , lmatches_21;
        // 12 and 21 matches
        ldesc_l1 = prev_frame->ldesc_l;
        ldesc_l2 = curr_frame->ldesc_l;

        bfm->radiusMatch( ldesc_l1, ldesc_l2, lmatches_12, Config::lineMatchRadius() );
        //        bfm->radiusMatch( ldesc_l2, ldesc_l1, lmatches_21, Config::lineMatchRadius() );

        std::multimap<int, std::pair<int, float>> pair_line_match;
        for ( int i = 0; i < lmatches_12.size(); i++ ) {

            if ((lmatches_12[i]).size() <= 0)
                continue ;

            int l12_qdx = lmatches_12[i][0].queryIdx;
            //            if (prev_frame->stereo_ls[l12_qdx]->frame_matched == true) {
            //                continue ;
            //            }

            Vector2d line_proj_sp, line_proj_ep;
            curr_frame->projectPrev3DLine(prev_frame->Tfw,
                                          prev_frame->stereo_ls[l12_qdx],
                                          line_proj_sp,
                                          line_proj_ep);

            Vector3d line_proj_coef;
            curr_frame->getLineCoeff(line_proj_sp, line_proj_ep, line_proj_coef);

            Vector2d line_proj_dir = line_proj_ep - line_proj_sp;
            line_proj_dir = line_proj_dir / line_proj_dir.norm();

            // instead of enforcing bidirectional best match between left and right,
            // top-K matches are tested to generate more matches
            double rng_included = 30.0; // 10.0; // 60.0; //
            for (int j=0; j<(lmatches_12[i]).size(); ++j) {
                assert(lmatches_12[i][j].queryIdx == l12_qdx);

                // check the distance between projected line and observed line =====
                int l12_tdx = lmatches_12[i][j].trainIdx;
                //
                //if (curr_frame->left_sel[l12_tdx].frame_matched == true) {
                //    continue ;
                //}
                if (curr_frame->withinLineSegment(line_proj_sp, line_proj_ep,
                                                  curr_frame->stereo_ls[l12_tdx]->spl,
                                                  rng_included) == false) {
                    continue ;
                }
                if (curr_frame->withinLineSegment(line_proj_sp, line_proj_ep,
                                                  curr_frame->stereo_ls[l12_tdx]->epl,
                                                  rng_included) == false) {
                    continue ;
                }

                double maxAngleDiff = CV_PI / 6; // CV_PI / 4;

                Vector2d line_meas_dir;
                line_meas_dir(0) = curr_frame->image_ls_l[l12_tdx].endPointX - curr_frame->image_ls_l[l12_tdx].startPointX;
                line_meas_dir(1) = curr_frame->image_ls_l[l12_tdx].endPointY - curr_frame->image_ls_l[l12_tdx].startPointY;
                line_meas_dir /= line_meas_dir.norm();
                if ( acos(line_proj_dir.dot(line_meas_dir)) >= maxAngleDiff) {
                    continue ;
                }
                //
                double proj_ept_dist = ( curr_frame->pointToLineDist(
                                             line_proj_coef,
                                             curr_frame->stereo_ls[l12_tdx]->spl) +
                                         curr_frame->pointToLineDist(
                                             line_proj_coef,
                                             curr_frame->stereo_ls[l12_tdx]->epl)
                                         ) / 2.0;
                //                std::cout << "Average point-to-line distance: " << proj_ept_dist << std::endl;

                if (proj_ept_dist <= 8.0) { //  4.0) { // 12.0) { //
                    //

                    // add distance check
                    if (lmatches_12[i][j].distance > Config::lineMatchRadius() * 0.8)
                        continue ;


                    std::pair<int,float> current_match(l12_qdx,lmatches_12[i][j].distance);
                    pair_line_match.insert(std::pair<int,pair<int,double>>(l12_tdx,current_match));

                    break ;
                }
            }
        }
        //
        auto x = pair_line_match.begin();
        while(pair_line_match.size() > 0 ) {

            std::multimap<int,pair<int,float>>::const_iterator it  = pair_line_match.lower_bound(x->first);
            std::multimap<int,pair<int,float>>::const_iterator it2 = pair_line_match.upper_bound(x->first);
            float min_value = 1e+7;
            int key = x->first;
            int matched_previous_line = 0;

            while (it !=it2 ) {
                std::pair<int, float> value = it->second;
                if(value.second < min_value) {
                    min_value = value.second;
                    matched_previous_line = value.first;
                }
                ++it;
            }
            size_t duplicates = pair_line_match.count(x->first);
            //std::cout<<"the number of duplocates are"<<pair_matching.count(x->first)<<std::endl;
            pair_line_match.erase(x->first);
            advance(x,duplicates);
            int l12_qdx = matched_previous_line;
            int l12_tdx = key;

            LineFeature* line_ = prev_frame->stereo_ls[l12_qdx];
            line_->sdisp_obs = curr_frame->stereo_ls[l12_tdx]->sdisp;
            line_->edisp_obs = curr_frame->stereo_ls[l12_tdx]->edisp;
            line_->spl_obs = curr_frame->stereo_ls[l12_tdx]->spl;
            line_->epl_obs = curr_frame->stereo_ls[l12_tdx]->epl;
            line_->le_obs  = curr_frame->stereo_ls[l12_tdx]->le;
            line_->inlier  = true;
            matched_ls.push_back( line_ );
            curr_frame->stereo_ls[l12_tdx]->idx = prev_frame->stereo_ls[l12_qdx]->idx; // prev idx

            //            // if the matching is accepted
            //            // kick the matched points out of following stereo match
            //            prev_frame->left_sel[l12_qdx].frame_matched = true;
            //            curr_frame->left_sel[l12_tdx].frame_matched = true;

            // enforce a upper bound on number of features matched across two frames
            if ( matched_ls.size() >= Config::maxLineMatchNum() ) {
                // force exit
                break ;
            }
        }
    }

    n_inliers_pt = matched_pt.size();
    n_inliers_ls = matched_ls.size();
    n_inliers    = n_inliers_pt + n_inliers_ls;

    //    if (print_frame_info)
    std::cout << "func crossFrameMatching: line matching num = " << n_inliers_ls
              << "; point matching num = " << n_inliers_pt << std::endl;
}


void StereoFrameHandler::crossFrameMatching_Hybrid() {
    // points f2f tracking
    matched_pt.clear();
    if( Config::hasPoints() && !(curr_frame->stereo_pt.size()==0) && !(prev_frame->stereo_pt.size()==0) ) {
        double startTime = clock();
        // reset the matching flag in stereo frames
        for (int i=0; i<prev_frame->stereo_pt.size(); ++i) {
            prev_frame->stereo_pt[i]->frame_matched = false;
        }
        //
        for (int i=0; i<curr_frame->stereo_pt.size(); ++i) {
            curr_frame->stereo_pt[i]->frame_matched = false;
        }

        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );    // cross-check
        Mat pdesc_l1, pdesc_l2;
        vector<vector<DMatch>> pmatches_12, pmatches_21;
        // 12 and 21 matches
        pdesc_l1 = prev_frame->pdesc_l;
        pdesc_l2 = curr_frame->pdesc_l;
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                // auto match_l = async( launch::async, &StereoFrame::matchPointFeatures, prev_frame, bfm, pdesc_l1, pdesc_l2, ref(pmatches_12) );
                // auto match_r = async( launch::async, &StereoFrame::matchPointFeatures, prev_frame, bfm, pdesc_l2, pdesc_l1, ref(pmatches_21) );
                auto match_l = async( launch::async, &StereoFrame::matchPointFeatures_radius,
                                      prev_frame, bfm, pdesc_l1, pdesc_l2, ref(pmatches_12) );
                auto match_r = async( launch::async, &StereoFrame::matchPointFeatures_radius,
                                      prev_frame, bfm, pdesc_l2, pdesc_l1, ref(pmatches_21) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( pdesc_l1, pdesc_l2, pmatches_12, 2);
                bfm->knnMatch( pdesc_l2, pdesc_l1, pmatches_21, 2);
            }
            //            bfm->radiusMatch( pdesc_l1, pdesc_l2, pmatches_12, Config::pointMatchRadius() );
            //            bfm->radiusMatch( pdesc_l2, pdesc_l1, pmatches_21, Config::pointMatchRadius() );
        }
        else
            bfm->knnMatch( pdesc_l1, pdesc_l2, pmatches_12, 2);

        std::multimap<int, std::pair<int,float>> pair_frame;
        for ( int i = 0; i < pmatches_12.size(); i++ ) {

            if ((pmatches_12[i]).size() <= 0)
            {
                continue ;
            }

            int p12_qdx = pmatches_12[i][0].queryIdx;
            if (prev_frame->stereo_pt[p12_qdx]->frame_matched == true) {
                continue ;
            }
            //            std::cout << "start matching point candidates for prev line " << p12_qdx
            //                      << " ---------------------------" << std::endl;
            //
            // [1] 3D-to-2D projection  ============================================
            Vector2d point_proj_l, point_proj_r;
            curr_frame->projectPrev3DPoint(prev_frame->Tfw,
                                           prev_frame->stereo_pt[p12_qdx],
                                           point_proj_l,
                                           point_proj_r);

            //            std::cout << "Projection of 3D point: " << pt_proj_tmp(0)
            //                      << "; " << pt_proj_tmp(1) << std::endl;

            //                double  proj_ept_dist_min = 9999999;
            //            int     p12_tdx_min = -1;
            // instead of enforcing bidirectional best match between left and right,
            // top-K matches are tested to generate more matches
            for (int j=0; j<(pmatches_12[i]).size(); ++j) {
                assert(pmatches_12[i][j].queryIdx == p12_qdx);
                // [2] check the distance between projected line and observed line =====
                int p12_tdx = pmatches_12[i][j].trainIdx;
                //
                //if (curr_frame->left_sel[l12_tdx].frame_matched == true) {
                //    continue ;
                //}
                // line_proj_tmp
                // curr_frame->stereo_ls[l12_tdx]
                double rng_included = 10.0; // 20.0; //
                // std::cout << pmatches_12[i][j].distance << std::endl;
                if ( (point_proj_l - curr_frame->stereo_pt[p12_tdx]->pl).norm() > rng_included ) {
                    continue ;
                }
                //                std::cout << (pt_proj_tmp - curr_frame->stereo_pt[p12_tdx]->pl).norm() << " ";

                //
                std::pair<int,float> current_match(p12_qdx, pmatches_12[i][j].distance);
                pair_frame.insert(std::pair<int,pair<int,double>>(p12_tdx, current_match));

                //                break ;
            }

            //            std::cout << std::endl;
        }

        auto x = pair_frame.begin();
        while(pair_frame.size() > 0 ) {

            std::multimap<int,pair<int,float>>::const_iterator it  = pair_frame.lower_bound(x->first);
            std::multimap<int,pair<int,float>>::const_iterator it2 = pair_frame.upper_bound(x->first);
            float min_value = 1e+7;
            int key = x->first;
            int matched_previous_pt = 0;

            while (it !=it2 ) {
                std::pair<int, float> value = it->second;
                if(value.second < min_value) {
                    min_value = value.second;
                    matched_previous_pt = value.first;
                }
                ++it;
            }

            size_t duplicates = pair_frame.count(x->first);
            //std::cout<<"the number of duplocates are"<<pair_matching.count(x->first)<<std::endl;
            pair_frame.erase(x->first);
            advance(x, duplicates);
            int p12_qdx = matched_previous_pt;
            int p12_tdx = key;

            PointFeature* point_ = prev_frame->stereo_pt[p12_qdx];
            point_->pl_obs = curr_frame->stereo_pt[p12_tdx]->pl;
            point_->inlier = true;
            matched_pt.push_back( point_ );
            curr_frame->stereo_pt[p12_tdx]->idx = prev_frame->stereo_pt[p12_qdx]->idx; // prev idx


            // if the matching is accepted
            // kick the matched lines out of following stereo match
            prev_frame->stereo_pt[p12_qdx]->frame_matched = true;
            curr_frame->stereo_pt[p12_tdx]->frame_matched = true;

            //            cv::DMatch match;
            //            match.queryIdx = p12_qdx;
            //            match.trainIdx = p12_tdx;
            //            match.distance = min_value;
            //            matches.push_back(match);

            // enforce a upper bound on number of features matched across two frames
            if ( matched_pt.size() >= Config::maxPointMatchNum() ) {
                // force exit
                break ;
            }
        }

        curr_frame->log_.num_pt_cross = matched_pt.size();
        curr_frame->log_.time_pt_cross = double(clock() - startTime) / double(CLOCKS_PER_SEC);
    }

    // line segments f2f tracking
    matched_ls.clear();
    if( Config::hasLines() && !(curr_frame->stereo_ls.size()==0) && !(prev_frame->stereo_ls.size()==0) ) {
        double startTime = clock();
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );    // cross-check
        Mat ldesc_l1, ldesc_l2;
        vector<vector<DMatch>> lmatches_12, lmatches_21;
        // 12 and 21 matches
        ldesc_l1 = prev_frame->ldesc_l;
        ldesc_l2 = curr_frame->ldesc_l;
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = async( launch::async, &StereoFrame::matchLineFeatures,
                                      prev_frame, bfm, ldesc_l1, ldesc_l2, ref(lmatches_12) );
                auto match_r = async( launch::async, &StereoFrame::matchLineFeatures,
                                      prev_frame, bfm, ldesc_l2, ldesc_l1, ref(lmatches_21) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( ldesc_l1,ldesc_l2, lmatches_12, 2);
                bfm->knnMatch( ldesc_l2,ldesc_l1, lmatches_21, 2);
            }
        }
        else
            bfm->knnMatch( ldesc_l1,ldesc_l2, lmatches_12, 2);
        // sort matches by the distance between the best and second best matches
        double nn_dist_th, nn12_dist_th, budget_dist_th;
        curr_frame->lineDescriptorMAD(lmatches_12,nn_dist_th, nn12_dist_th);
        nn12_dist_th  = nn12_dist_th * Config::descThL();
        curr_frame->lineDescriptorBudgetThres(lmatches_12, budget_dist_th);
        //        std::cout << "budget thres = " << budget_dist_th << std::endl;
        // resort according to the queryIdx
        sort( lmatches_12.begin(), lmatches_12.end(), sort_descriptor_by_queryIdx() );
        if( Config::bestLRMatches() )
            sort( lmatches_21.begin(), lmatches_21.end(), sort_descriptor_by_queryIdx() );

        // bucle around pmatches
        for( int i = 0; i < lmatches_12.size(); i++ )
        {
            // check if they are mutual best matches
            int lr_qdx = lmatches_12[i][0].queryIdx;
            int lr_tdx = lmatches_12[i][0].trainIdx;
            int rl_tdx;
            if( Config::bestLRMatches() )
                rl_tdx = lmatches_21[lr_tdx][0].trainIdx;
            else
                rl_tdx = lr_qdx;

            // enforce a upper bound on descriptor distance, so that only the top-N matchings are considered
            //            std::cout << "budget thres = " << budget_dist_th << "; current dist = " << lmatches_12[i][0].distance << std::endl;
            if ( lmatches_12[i][0].distance > 1.2 * budget_dist_th )
                continue ;

            // check if they are mutual best matches and the minimum distance
            double dist_12 = lmatches_12[i][1].distance - lmatches_12[i][0].distance;
            if( lr_qdx == rl_tdx  && dist_12 > nn12_dist_th )
            {
                LineFeature* line_ = prev_frame->stereo_ls[lr_qdx];
                line_->sdisp_obs = curr_frame->stereo_ls[lr_tdx]->sdisp;
                line_->edisp_obs = curr_frame->stereo_ls[lr_tdx]->edisp;
                line_->spl_obs = curr_frame->stereo_ls[lr_tdx]->spl;
                line_->epl_obs = curr_frame->stereo_ls[lr_tdx]->epl;
                line_->le_obs  = curr_frame->stereo_ls[lr_tdx]->le;
                line_->inlier  = true;
                matched_ls.push_back( line_ );
                curr_frame->stereo_ls[lr_tdx]->idx = prev_frame->stereo_ls[lr_qdx]->idx; // prev idx
            }

            // enforce a upper bound on number of features matched across two frames
            if ( matched_ls.size() >= Config::maxLineMatchNum() ) {
                // force exit
                break ;
            }
        }

        curr_frame->log_.num_ln_cross = matched_ls.size();
        curr_frame->log_.time_ln_cross = double(clock() - startTime) / double(CLOCKS_PER_SEC);
    }

    n_inliers_pt = matched_pt.size();
    n_inliers_ls = matched_ls.size();
    n_inliers    = n_inliers_pt + n_inliers_ls;

    //    if (print_frame_info)
    std::cout << "func crossFrameMatching: line matching num = " << n_inliers_ls
              << "; point matching num = " << n_inliers_pt << std::endl;
}


void StereoFrameHandler::crossFrameMatching_Brute() {
    // points f2f tracking
    matched_pt.clear();
    if( Config::hasPoints() && !(curr_frame->stereo_pt.size()==0) && !(prev_frame->stereo_pt.size()==0)  )
    {
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );    // cross-check
        Mat pdesc_l1, pdesc_l2;
        vector<vector<DMatch>> pmatches_12, pmatches_21;
        // 12 and 21 matches
        pdesc_l1 = prev_frame->pdesc_l;
        pdesc_l2 = curr_frame->pdesc_l;
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = async( launch::async, &StereoFrame::matchPointFeatures,
                                      prev_frame, bfm, pdesc_l1, pdesc_l2, ref(pmatches_12) );
                auto match_r = async( launch::async, &StereoFrame::matchPointFeatures,
                                      prev_frame, bfm, pdesc_l2, pdesc_l1, ref(pmatches_21) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( pdesc_l1, pdesc_l2, pmatches_12, 2);
                bfm->knnMatch( pdesc_l2, pdesc_l1, pmatches_21, 2);
            }
        }
        else
            bfm->knnMatch( pdesc_l1, pdesc_l2, pmatches_12, 2);
        // sort matches by the distance between the best and second best matches
        double nn12_dist_th = Config::maxRatio12P(), budget_dist_th;
        curr_frame->pointDescriptorBudgetThres(pmatches_12, budget_dist_th);
        // resort according to the queryIdx
        sort( pmatches_12.begin(), pmatches_12.end(), sort_descriptor_by_queryIdx() );
        if( Config::bestLRMatches() )
            sort( pmatches_21.begin(), pmatches_21.end(), sort_descriptor_by_queryIdx() );
        // bucle around pmatches
        for( int i = 0; i < pmatches_12.size(); i++ )
        {
            // check if they are mutual best matches
            int lr_qdx = pmatches_12[i][0].queryIdx;
            int lr_tdx = pmatches_12[i][0].trainIdx;
            int rl_tdx;
            if( Config::bestLRMatches() )
                rl_tdx = pmatches_21[lr_tdx][0].trainIdx;
            else
                rl_tdx = lr_qdx;

            // enforce a upper bound on descriptor distance, so that only the top-N matchings are considered
            if ( pmatches_12[i][0].distance > 1.2 * budget_dist_th )
                continue ;

            // check if they are mutual best matches and the minimum distance
            //            double dist_nn = pmatches_12[i][0].distance;
            double dist_12 = pmatches_12[i][0].distance / pmatches_12[i][1].distance;
            //            std::cout << nn12_dist_th << std::endl;
            //            if( lr_qdx == rl_tdx  && dist_12 > nn12_dist_th )
            if( lr_qdx == rl_tdx && dist_12 <= nn12_dist_th )
            {
                PointFeature* point_ = prev_frame->stereo_pt[lr_qdx];
                point_->pl_obs = curr_frame->stereo_pt[lr_tdx]->pl;
                point_->inlier = true;
                matched_pt.push_back( point_ );
                curr_frame->stereo_pt[lr_tdx]->idx = prev_frame->stereo_pt[lr_qdx]->idx; // prev idx
            }

            // enforce a upper bound on number of features matched across two frames
            if ( matched_pt.size() >= Config::maxPointMatchNum() ) {
                // force exit
                break ;
            }
        }
    }

    // line segments f2f tracking
    matched_ls.clear();
    if( Config::hasLines() && !(curr_frame->stereo_ls.size()==0) && !(prev_frame->stereo_ls.size()==0)  )
    {
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );    // cross-check
        Mat ldesc_l1, ldesc_l2;
        vector<vector<DMatch>> lmatches_12, lmatches_21;
        // 12 and 21 matches
        ldesc_l1 = prev_frame->ldesc_l;
        ldesc_l2 = curr_frame->ldesc_l;
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = async( launch::async, &StereoFrame::matchLineFeatures,
                                      prev_frame, bfm, ldesc_l1, ldesc_l2, ref(lmatches_12) );
                auto match_r = async( launch::async, &StereoFrame::matchLineFeatures,
                                      prev_frame, bfm, ldesc_l2, ldesc_l1, ref(lmatches_21) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( ldesc_l1,ldesc_l2, lmatches_12, 2);
                bfm->knnMatch( ldesc_l2,ldesc_l1, lmatches_21, 2);
            }
        }
        else
            bfm->knnMatch( ldesc_l1,ldesc_l2, lmatches_12, 2);
        // sort matches by the distance between the best and second best matches
        double nn_dist_th, nn12_dist_th, budget_dist_th;
        curr_frame->lineDescriptorMAD(lmatches_12,nn_dist_th, nn12_dist_th);
        nn12_dist_th  = nn12_dist_th * Config::descThL();
        curr_frame->lineDescriptorBudgetThres(lmatches_12, budget_dist_th);
        //        std::cout << "budget thres = " << budget_dist_th << std::endl;
        // resort according to the queryIdx
        sort( lmatches_12.begin(), lmatches_12.end(), sort_descriptor_by_queryIdx() );
        if( Config::bestLRMatches() )
            sort( lmatches_21.begin(), lmatches_21.end(), sort_descriptor_by_queryIdx() );

        // bucle around pmatches
        for( int i = 0; i < lmatches_12.size(); i++ )
        {
            // check if they are mutual best matches
            int lr_qdx = lmatches_12[i][0].queryIdx;
            int lr_tdx = lmatches_12[i][0].trainIdx;
            int rl_tdx;
            if( Config::bestLRMatches() )
                rl_tdx = lmatches_21[lr_tdx][0].trainIdx;
            else
                rl_tdx = lr_qdx;

            // enforce a upper bound on descriptor distance, so that only the top-N matchings are considered
            //            std::cout << "budget thres = " << budget_dist_th << "; current dist = " << lmatches_12[i][0].distance << std::endl;
            if ( lmatches_12[i][0].distance > 1.2 * budget_dist_th )
                continue ;

            // check if they are mutual best matches and the minimum distance
            double dist_12 = lmatches_12[i][1].distance - lmatches_12[i][0].distance;
            if( lr_qdx == rl_tdx  && dist_12 > nn12_dist_th )
            {
                LineFeature* line_ = prev_frame->stereo_ls[lr_qdx];
                line_->sdisp_obs = curr_frame->stereo_ls[lr_tdx]->sdisp;
                line_->edisp_obs = curr_frame->stereo_ls[lr_tdx]->edisp;
                line_->spl_obs = curr_frame->stereo_ls[lr_tdx]->spl;
                line_->epl_obs = curr_frame->stereo_ls[lr_tdx]->epl;
                line_->le_obs  = curr_frame->stereo_ls[lr_tdx]->le;
                line_->inlier  = true;
                matched_ls.push_back( line_ );
                curr_frame->stereo_ls[lr_tdx]->idx = prev_frame->stereo_ls[lr_qdx]->idx; // prev idx
            }

            // enforce a upper bound on number of features matched across two frames
            if ( matched_ls.size() >= Config::maxLineMatchNum() ) {
                // force exit
                break ;
            }
        }
    }

    n_inliers_pt = matched_pt.size();
    n_inliers_ls = matched_ls.size();
    n_inliers    = n_inliers_pt + n_inliers_ls;

    //    if (print_frame_info)
    std::cout << "func crossFrameMatching: line matching num = " << n_inliers_ls
              << "; point matching num = " << n_inliers_pt << std::endl;
}



void StereoFrameHandler::updateFrame_ECCV18( const Matrix4d T_base )
{

    // update FAST threshold for the keypoint detection
    if( Config::adaptativeFAST() )
    {
        int min_fast  = Config::fastMinTh();
        int max_fast  = Config::fastMaxTh();
        int fast_inc  = Config::fastIncTh();
        int feat_th   = Config::fastFeatTh();
        float err_th  = Config::fastErrTh();

        // if bad optimization, -= 2*fast_inc
        if( curr_frame->DT == Matrix4d::Identity() || curr_frame->err_norm > err_th )
            orb_fast_th = std::max( min_fast, orb_fast_th - 2*fast_inc );
        // elif number of features ...
        else if( n_inliers_pt < feat_th )
            orb_fast_th = std::max( min_fast, orb_fast_th - 2*fast_inc );
        else if( n_inliers < feat_th * 2 )
            orb_fast_th = std::max( min_fast, orb_fast_th - fast_inc );
        else if( n_inliers > feat_th * 3 )
            orb_fast_th = std::min( max_fast, orb_fast_th + fast_inc );
        else if( n_inliers > feat_th * 4 )
            orb_fast_th = std::min( max_fast, orb_fast_th + 2*fast_inc );
    }

    // clean and update variables
    matched_pt.clear();
    matched_ls.clear();

    line_projection_end.clear();
    line_projection_start.clear();

    // visualize the pose optimization results as well as the line matches
    //    if (curr_frame->frame_idx % 20 == 0)
    //    plotLine3DCrossFrame();
    //    plotLine3D();

    if (print_frame_info)
        std::cout << "Before update, num of stereo lines at previous frame = "
                  << prev_frame->stereo_ls.size() << std::endl;

    // predict the rel motion of current frame with constant velocity model
    //    curr_frame->DT = prev_frame->Tfw.inverse() * curr_frame->Tfw;
    Matrix4d T_curr_w = T_base * prev_frame->Tfw;

    delete prev_frame;
    prev_frame = curr_frame;
    curr_frame = NULL;

    if (print_frame_info)
        std::cout << "After update, num of stereo lines at previous frame = "
                  << prev_frame->stereo_ls.size() << std::endl;

    // save the current pose estimation
//    T_allF.push_back(std::make_pair(prev_frame->time_stamp, T_curr_w));
    vec_all_frame_pose.push_back(T_curr_w);

}



void StereoFrameHandler::updateFrame( )
{

    // update FAST threshold for the keypoint detection
    if( Config::adaptativeFAST() )
    {
        int min_fast  = Config::fastMinTh();
        int max_fast  = Config::fastMaxTh();
        int fast_inc  = Config::fastIncTh();
        int feat_th   = Config::fastFeatTh();
        float err_th  = Config::fastErrTh();

        // if bad optimization, -= 2*fast_inc
        if( curr_frame->DT == Matrix4d::Identity() || curr_frame->err_norm > err_th )
            orb_fast_th = std::max( min_fast, orb_fast_th - 2*fast_inc );
        // elif number of features ...
        else if( n_inliers_pt < feat_th )
            orb_fast_th = std::max( min_fast, orb_fast_th - 2*fast_inc );
        else if( n_inliers < feat_th * 2 )
            orb_fast_th = std::max( min_fast, orb_fast_th - fast_inc );
        else if( n_inliers > feat_th * 3 )
            orb_fast_th = std::min( max_fast, orb_fast_th + fast_inc );
        else if( n_inliers > feat_th * 4 )
            orb_fast_th = std::min( max_fast, orb_fast_th + 2*fast_inc );
    }

    // clean and update variables
    matched_pt.clear();
    matched_ls.clear();

    line_projection_end.clear();
    line_projection_start.clear();

    // visualize the pose optimization results as well as the line matches
    //    if (curr_frame->frame_idx % 20 == 0)
    //    plotLine3DCrossFrame();
    //    plotLine3D();

    if (print_frame_info)
        std::cout << "Before update, num of stereo lines at previous frame = "
                  << prev_frame->stereo_ls.size() << std::endl;

    // predict the rel motion of current frame with constant velocity model
    //    curr_frame->DT = prev_frame->Tfw.inverse() * curr_frame->Tfw;

    delete prev_frame;
    prev_frame = curr_frame;
    curr_frame = NULL;

    if (print_frame_info)
        std::cout << "After update, num of stereo lines at previous frame = "
                  << prev_frame->stereo_ls.size() << std::endl;

    // save the current pose estimation
//    T_allF.push_back(std::make_pair(prev_frame->time_stamp, prev_frame->Tfw));

}


void StereoFrameHandler::setLeftCamExtrinCalib(const double cam_rx,
                                               const double cam_ry,
                                               const double cam_rz,
                                               const double cam_tx,
                                               const double cam_ty,
                                               const double cam_tz) {
    this->left_cam_T_rigframe << cam_rx, cam_ry, cam_rz,
            cam_tx, cam_ty, cam_tz;
}

void StereoFrameHandler::getPoseCovMatrixOnLine(const int fst_idx,
                                                const int lst_idx,
                                                const Matrix<double, 6, 1> rigframe_T_world,
                                                const list<LineFeature*>::iterator it,
                                                Matrix<double, 6, 6> & cov_pose_inv) {

    Vector2d spl_tmp = (*it)->lPt2D[fst_idx];
    Vector2d epl_tmp = (*it)->lPt2D[lst_idx];
    double sdisp_tmp = (*it)->displPt2D[fst_idx];
    double edisp_tmp = (*it)->displPt2D[lst_idx];
    Vector3d sP_tmp = this->cam->backProjection(spl_tmp[0], spl_tmp[1], sdisp_tmp);
    Vector3d eP_tmp = this->cam->backProjection(epl_tmp[0], epl_tmp[1], edisp_tmp);

    Matrix<double, 6, 1> Jac_sr2x;
    Matrix<double, 6, 1> Jac_er2x;
    pointPair2LineJacobian(rigframe_T_world ,
                           this->left_cam_T_rigframe,
                           (*it)->le_obs,
                           sP_tmp, eP_tmp,
                           Jac_sr2x, Jac_er2x);

    //    // sanity check
    //    Matrix<double, 6, 1> Jac_sr2x_vld;
    //    Matrix<double, 6, 1> Jac_er2x_vld;
    //    point2LineJacobian(rigframe_T_world ,
    //                       this->left_cam_T_rigframe,
    //                       (*it)->le_obs,
    //                       sP_tmp,
    //                       Jac_sr2x_vld);
    //    point2LineJacobian(rigframe_T_world ,
    //                       this->left_cam_T_rigframe,
    //                       (*it)->le_obs,
    //                       eP_tmp,
    //                       Jac_er2x_vld);
    //    std::cout << "new spt Jac = " << Jac_sr2x << std::endl;
    //    std::cout << "old spt Jac = " << Jac_sr2x_vld << std::endl;
    //    std::cout << "new ept Jac = " << Jac_er2x << std::endl;
    //    std::cout << "old ept Jac = " << Jac_er2x_vld << std::endl;

    // NOTE
    // instead of taking the original line loss Jacobian, the normalize form is used here;
    // only in this way the line cut can be encouraged, but it might not be theoretically stand;
    // to further explore line cut, we might either:
    // 1) look into the possibility of line expanding
    // 2) emphize the impact of difference in eigen vector over the magnitude of eigen value
    //    cov_pose_inv =
    //            (Jac_sr2x * Jac_sr2x.transpose()) / ( pow(Jac_sr2x.norm(), 2) * (*it)->varLoss2D[fst_idx] ) +
    //            (Jac_er2x * Jac_er2x.transpose()) / ( pow(Jac_er2x.norm(), 2) * (*it)->varLoss2D[lst_idx] );
    cov_pose_inv =
            (Jac_sr2x * Jac_sr2x.transpose()) / ( (*it)->varLoss2D[fst_idx] ) +
            (Jac_er2x * Jac_er2x.transpose()) / ( (*it)->varLoss2D[lst_idx] );

}

// OBSELETE
// assess the uncertainty of each line match wrt stereo uncertainty model
// to begin with, we steal the exact same code from pl-svo to demonstrate the concept of uncertainty
void StereoFrameHandler::estimateProjUncertainty_greedy() {
    //
    assert(matched_ls.size() > 0);
    //
    //    Matrix4d rel_Tfw = prev_frame->Tfw.inverse() * curr_frame->Tfw;
    Matrix4d rel_Tfw = curr_frame->Tfw.inverse() * prev_frame->Tfw;
    Matrix3d J_dt = rel_Tfw.block(0,0,3,3);

    Quaterniond q_rel( J_dt );
    Matrix<double, 6, 1> rigframe_T_world;
    rigframe_T_world << q_rel.coeffs()(0,0), q_rel.coeffs()(1,0), q_rel.coeffs()(2,0),
            rel_Tfw(0,3), rel_Tfw(1,3), rel_Tfw(2,3);
    //    std::cout << "rigframe_T_world = " << rigframe_T_world << std::endl;
    //
    //    Matrix<double, 6, 1> camera_T_rigframe;
    //    std::cout << "left_cam_T_rigframe = " << this->left_cam_T_rigframe << std::endl;

    Matrix3d J_proj;
    Vector3d sP_cur, eP_cur;
    Vector2d spl_cur, epl_cur;
    Vector3d J_loss;
    //
    for( list<StVO::LineFeature *>::iterator it = matched_ls.begin();
         it != matched_ls.end(); it++) {

        // [1] === project 3D uncertainty from prev frame to curr frame ===
        sP_cur = rel_Tfw.block(0,0,3,3) * (*it)->sP + rel_Tfw.col(3).head(3);
        spl_cur = this->cam->projection(sP_cur);
        curr_frame->getJacob3D_2D(sP_cur[0], sP_cur[1], sP_cur[2], J_proj);
        (*it)->covSpt_proj2D = J_proj * J_dt * (*it)->covSpt3D * J_dt.transpose() * J_proj.transpose();
        //
        eP_cur = rel_Tfw.block(0,0,3,3) * (*it)->eP + rel_Tfw.col(3).head(3);
        epl_cur = this->cam->projection(eP_cur);
        curr_frame->getJacob3D_2D(eP_cur[0], eP_cur[1], eP_cur[2], J_proj);
        (*it)->covEpt_proj2D = J_proj * J_dt * (*it)->covEpt3D * J_dt.transpose() * J_proj.transpose();

        // [2] === define the perpendicular direction to minimize uncertainty ===
        //
        // NOTE
        // due to the fact that the line coe has been normalized from image frame
        // to camera frame, we need to devide the focu-length from J_loss accordingly
        // however the resulting volume seems to be too high; therefore here we only
        // multiply the ratio rather than the actually focu-length
        //
        J_loss(0) = (*it)->le_obs[0];
        J_loss(1) = (*it)->le_obs[1];
        J_loss(2) = (*it)->le_obs[2];

        // [3] === propogate along the 3D/2D line to get the confident portion ===
        (*it)->displPt2D.clear();
        (*it)->lPt2D.clear();
        (*it)->covlPt2D.clear();
        (*it)->varLoss2D.clear();
        //
        // NOTE
        // Would it make more sense to sample along 3D line?
        // Otherwise we may run into "out-of-frame" issues
        // May revisit the math later to confirm the durability of this update
        //
        // Also, never put double/float as index in for loop!
        for (int np = 0; np <= NUM_PTS_SAMPLE; ++ np) {
            // sample along the projected line
            double alpha = (double)np / NUM_PTS_SAMPLE;
            // NOTE
            // the line above is wrong! we are supposed to sample along the line projected
            // on current frame; while spl & epl are at previous frame!
            Vector2d pt2D_cur = (1.0 - alpha) * spl_cur + alpha * epl_cur;
            Matrix3d cov2D = (1-alpha) * (1-alpha) * ((*it)->covSpt_proj2D) +
                    alpha * alpha * ((*it)->covEpt_proj2D);
            double var2D = J_loss.transpose() * cov2D * J_loss;
            //
            // handle out of bound variation cases (horizontal line mostly)
            if (var2D > MAX_VAL || var2D < 0)
                var2D = MAX_VAL;
            //
            // NOTE
            // the propogation is performed on current frame; meanwhile the variable to be used in
            // optimization lies in previous frame; a conversion is called in the following for that
            // purpose
            //
            Vector3d ptRay = this->cam->backProjection(pt2D_cur(0), pt2D_cur(1), 1.0);
            Vector3d pt3D_curr, pt3D_1, pt3D_prev;
            //            std::cout << "ptRay = " << ptRay << std::endl;
            //            std::cout << "sP_cur = " << sP_cur << std::endl;
            //            std::cout << "eP_cur = " << eP_cur << std::endl;
            double dist_tmp = get3DLineIntersection(Vector3d(0.0, 0.0, 0.0), ptRay,
                                                    sP_cur, eP_cur,
                                                    pt3D_1, pt3D_curr);
            assert(dist_tmp < 0.1);
            //            std::cout << "dist = " << dist_tmp << "; "
            //                      << "pt3D = " << pt3D << "; "
            //                      << "pt3D_1 = " << pt3D_1 << "; "
            //                      << std::endl;
            //            std::cout << "original 2D lpt = " << pt2D << "; ";

            // remember everything lives at current frame in the code above
            // now we need to transfrom back to previous frame
            pt3D_prev = rel_Tfw.block(0,0,3,3).transpose() * (pt3D_curr - rel_Tfw.col(3).head(3));
            // and solve the disparity as well as pixel localtion
            Vector2d pt2D_prev = this->cam->projection(pt3D_prev);
            double disp2D_prev = this->cam->getDisparity(pt3D_prev(2));
            //            std::cout << "transformed 3D lpt = " << pt3D
            //                      << "; " << " pZ = " << pt3D(2) << std::endl;
            //
            //            std::cout << "projected new lpt = " << pt2D << std::endl;
            //            std::cout << "original disparity = "
            //                      << (1-alpha) * (*it)->sdisp + alpha * (*it)->edisp
            //                      << "; new disparity = " << disp2D << std::endl;
            //
            (*it)->displPt2D.push_back(disp2D_prev);
            (*it)->lPt2D.push_back(pt2D_prev);
            (*it)->covlPt2D.push_back(cov2D);
            // given the var of loss term, we can later map back to cov of pose and
            // assess the contribution of each sampled point
            (*it)->varLoss2D.push_back(var2D);
            //            (*it)->cvLoss2D.push_back(sqrt(var2D) / loss2D);
        }

        // [4] === cut the line to confident portion only ===
        int fst_idx = 0, lst_idx = (*it)->lPt2D.size() - 1;
        double len_line_orig = ((*it)->sP - (*it)->eP).norm();

        if (Config::useLineConfCut()) {
            //            if (len_line_orig > 1.0) {
            while (fst_idx < lst_idx - 1) {
                double len_line = ((*it)->epl - (*it)->spl).norm();
                // ratio of line length serve as the bottom line of line cut
                // it generally forbits cutting too much of a line
                //                if (len_line < 0.3 * len_line_orig)
                //                if (len_line < Config::ratioLineConfCut() * len_line_orig)
                //                    break ;
                //
                // EXPERIMENTAL
                // try the max eigen value of cov_x
                //
                Matrix<double, 6, 6> cov_pose_inv_0, cov_pose_inv_1, cov_pose_inv_2;
                getPoseCovMatrixOnLine(fst_idx+1, lst_idx, rigframe_T_world, it, cov_pose_inv_0);
                getPoseCovMatrixOnLine(fst_idx, lst_idx, rigframe_T_world, it, cov_pose_inv_1);
                getPoseCovMatrixOnLine(fst_idx, lst_idx-1, rigframe_T_world, it, cov_pose_inv_2);

                vector<double> eigen_0, eigen_1, eigen_2;
                //
                getEigenValue( cov_pose_inv_0, eigen_0 );
                getEigenValue( cov_pose_inv_1, eigen_1 );
                getEigenValue( cov_pose_inv_2, eigen_2 );

                double prod_0 = eigen_0[0] * eigen_0[1],
                        prod_1 = eigen_1[0] * eigen_1[1],
                        prod_2 = eigen_2[0] * eigen_2[1];
                //                    double prod_0 = eigen_0[0],
                //                            prod_1 = eigen_1[0],
                //                            prod_2 = eigen_2[0];

                //                std::cout << eigen_0[0] << ", " << eigen_0[1] << ": " << prod_0 << std::endl;
                //                std::cout << eigen_1[0] << ", " << eigen_1[1] << ": " << prod_1 << std::endl;
                //                std::cout << eigen_2[0] << ", " << eigen_2[1] << ": " << prod_2 << std::endl;

                if ( prod_0 > prod_1 && prod_0 > prod_2 ) {
                    fst_idx ++;
                    //                        std::cout << "line cutting trigger!" << std::endl;
                }
                else if ( prod_2 > prod_0 && prod_2 > prod_1 ) {
                    lst_idx --;
                    //                        std::cout << "line cutting trigger!" << std::endl;
                }
                else {
                    break ;
                }

                //
                (*it)->spl = (*it)->lPt2D[fst_idx];
                (*it)->epl = (*it)->lPt2D[lst_idx];
                (*it)->sdisp = (*it)->displPt2D[fst_idx];
                (*it)->edisp = (*it)->displPt2D[lst_idx];
                (*it)->sP = this->cam->backProjection((*it)->spl[0], (*it)->spl[1], (*it)->sdisp);
                (*it)->eP = this->cam->backProjection((*it)->epl[0], (*it)->epl[1], (*it)->edisp);
                //
                //                (*it)->varSptLoss = (*it)->varLoss2D[fst_idx];
                //                (*it)->varEptLoss = (*it)->varLoss2D[lst_idx];
                //                (*it)->cvSptLoss = (*it)->cvLoss2D[fst_idx];
                //                (*it)->cvEptLoss = (*it)->cvLoss2D[lst_idx];
            }
        }
        //
        (*it)->fst_idx = fst_idx;
        (*it)->lst_idx = lst_idx;

        // debug
        if (print_frame_info) {
            std::cout << "ept uncertainty in loss term: "
                      << (*it)->varLoss2D[(*it)->fst_idx] << "; "
                      << (*it)->varLoss2D[(*it)->lst_idx] << std::endl;
        }
    }
}


void StereoFrameHandler::getPoseCovOnLine(const Matrix4d DT_inv,
                                          const Vector2d J_loss,
                                          const StVO::LineFeature * line,
                                          const double cutRatio[2], Matrix<double, 6, 6> & cov_pose) {
    //
    Vector3d sP_tmp = (1 - cutRatio[0]) * line->sP + cutRatio[0] * line->eP;
    Vector3d eP_tmp = (1 - cutRatio[1]) * line->eP + cutRatio[1] * line->sP;
    Matrix3d cov_sP_tmp = pow(1 - cutRatio[0], 2) * line->covSpt3D +
            pow(cutRatio[0], 2) * line->covEpt3D;
    Matrix3d cov_eP_tmp = pow(1 - cutRatio[1], 2) * line->covEpt3D +
            pow(cutRatio[1], 2) * line->covSpt3D;
    Matrix3d J_dt = DT_inv.block(0,0,3,3);

    // get the cov matrix of residual
    Matrix3d J_proj;
    Vector3d cur_sP_tmp = J_dt * sP_tmp + DT_inv.col(3).head(3);
    curr_frame->getJacob3D_2D(cur_sP_tmp[0], cur_sP_tmp[1], cur_sP_tmp[2], J_proj);
    double cov_sR_tmp = J_loss.transpose() *
            ( J_proj.block(0,0,2,3) * J_dt * cov_sP_tmp
              * J_dt.transpose() * J_proj.block(0,0,2,3).transpose() )
            * J_loss;
    //
    Vector3d cur_eP_tmp = J_dt * eP_tmp + DT_inv.col(3).head(3);
    curr_frame->getJacob3D_2D(cur_eP_tmp[0], cur_eP_tmp[1], cur_eP_tmp[2], J_proj);
    double cov_eR_tmp = J_loss.transpose() *
            ( J_proj.block(0,0,2,3) * J_dt * cov_eP_tmp
              * J_dt.transpose() * J_proj.block(0,0,2,3).transpose() )
            * J_loss;
    //
    Matrix<double, 2, 2> cov_r;
    cov_r << cov_sR_tmp, 0, 0, cov_eR_tmp;

    // get the Jacobian wrt residual
    Matrix<double, 6, 1> Jac_sr2x;
    double gx   = cur_sP_tmp(0);
    double gy   = cur_sP_tmp(1);
    double gz   = cur_sP_tmp(2);
    double gz2  = gz*gz;
    double fgz2 = this->cam->getFx() / std::max(Config::homogTh(), gz2);
    double lx   = J_loss(0);
    double ly   = J_loss(1);
    Jac_sr2x << + fgz2 * lx * gz,
            + fgz2 * ly * gz,
            - fgz2 * ( gx*lx + gy*ly ),
            - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
            + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
            + fgz2 * ( gx*gz*ly - gy*gz*lx );
    //
    Matrix<double, 6, 1> Jac_er2x;
    gx   = cur_eP_tmp(0);
    gy   = cur_eP_tmp(1);
    gz   = cur_eP_tmp(2);
    gz2  = gz*gz;
    fgz2 = this->cam->getFx() / std::max(Config::homogTh(),gz2);
    Jac_er2x << + fgz2 * lx * gz,
            + fgz2 * ly * gz,
            - fgz2 * ( gx*lx + gy*ly ),
            - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
            + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
            + fgz2 * ( gx*gz*ly - gy*gz*lx );

    //
    //    Quaterniond q_rel( J_dt );
    //    Matrix<double, 6, 1> rigframe_T_world;
    //    rigframe_T_world << q_rel.coeffs()(0,0), q_rel.coeffs()(1,0), q_rel.coeffs()(2,0),
    //            DT_inv(0,3), DT_inv(1,3), DT_inv(2,3);

    //    Matrix<double, 6, 1> Jac_sr2x;
    //    Matrix<double, 6, 1> Jac_er2x;
    //    pointPair2LineJacobian(rigframe_T_world ,
    //                           this->left_cam_T_rigframe,
    //                           line->le_obs,
    //                           sP_tmp, eP_tmp,
    //                           Jac_sr2x, Jac_er2x);

    //    std::cout << "Jac - original: " << Jac_sr2x_ori << std::endl;
    //    std::cout << "Jac - modified: " << Jac_sr2x << std::endl;
    //    std::cout << "Jac - original: " << Jac_er2x_ori << std::endl;
    //    std::cout << "Jac - modified: " << Jac_er2x << std::endl;

    // Put together two jacobians
    Matrix<double, 6, 2> Jac_r2x(6, 2);
    Jac_r2x << Jac_sr2x, Jac_er2x;
    Matrix<double, 2, 6> Jac_r2x_inv = (Jac_r2x.transpose()*Jac_r2x).inverse() * Jac_r2x.transpose();

    // Obtain the pose covariance (to be verified)
    cov_pose = Jac_r2x_inv.transpose() * cov_r * Jac_r2x_inv;
    //    cov_pose = Jac_r2x_inv.transpose() * Jac_r2x_inv;

}



void StereoFrameHandler::getPoseInfoOnLine(const Matrix4d DT_inv,
                                           const Vector2d J_loss,
                                           const StVO::LineFeature * line,
                                           const double cutRatio[2], Matrix<double, 6, 6> & info_pose) {
    //
    Vector3d sP_tmp = (1 - cutRatio[0]) * line->sP + cutRatio[0] * line->eP;
    Vector3d eP_tmp = (1 - cutRatio[1]) * line->eP + cutRatio[1] * line->sP;
    Matrix3d cov_sP_tmp = pow(1 - cutRatio[0], 2) * line->covSpt3D +
            pow(cutRatio[0], 2) * line->covEpt3D;
    Matrix3d cov_eP_tmp = pow(1 - cutRatio[1], 2) * line->covEpt3D +
            pow(cutRatio[1], 2) * line->covSpt3D;
    Matrix3d J_dt = DT_inv.block(0,0,3,3);

    // get the cov matrix of residual
    Matrix3d J_proj;
    Vector3d cur_sP_tmp = J_dt * sP_tmp + DT_inv.col(3).head(3);
    curr_frame->getJacob3D_2D(cur_sP_tmp[0], cur_sP_tmp[1], cur_sP_tmp[2], J_proj);
    double cov_sR_tmp = J_loss.transpose() *
            ( J_proj.block(0,0,2,3) * J_dt * cov_sP_tmp
              * J_dt.transpose() * J_proj.block(0,0,2,3).transpose() )
            * J_loss;
    //
    Vector3d cur_eP_tmp = J_dt * eP_tmp + DT_inv.col(3).head(3);
    curr_frame->getJacob3D_2D(cur_eP_tmp[0], cur_eP_tmp[1], cur_eP_tmp[2], J_proj);
    double cov_eR_tmp = J_loss.transpose() *
            ( J_proj.block(0,0,2,3) * J_dt * cov_eP_tmp
              * J_dt.transpose() * J_proj.block(0,0,2,3).transpose() )
            * J_loss;
    //
    Matrix<double, 2, 2> cov_r;
    cov_r << cov_sR_tmp, 0, 0, cov_eR_tmp;

    // get the Jacobian wrt residual
    Matrix<double, 6, 1> Jac_sr2x;
    double gx   = cur_sP_tmp(0);
    double gy   = cur_sP_tmp(1);
    double gz   = cur_sP_tmp(2);
    double gz2  = gz*gz;
    double fgz2 = this->cam->getFx() / std::max(Config::homogTh(), gz2);
    double lx   = J_loss(0);
    double ly   = J_loss(1);
    Jac_sr2x << + fgz2 * lx * gz,
            + fgz2 * ly * gz,
            - fgz2 * ( gx*lx + gy*ly ),
            - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
            + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
            + fgz2 * ( gx*gz*ly - gy*gz*lx );
    //
    Matrix<double, 6, 1> Jac_er2x;
    gx   = cur_eP_tmp(0);
    gy   = cur_eP_tmp(1);
    gz   = cur_eP_tmp(2);
    gz2  = gz*gz;
    fgz2 = this->cam->getFx() / std::max(Config::homogTh(),gz2);
    Jac_er2x << + fgz2 * lx * gz,
            + fgz2 * ly * gz,
            - fgz2 * ( gx*lx + gy*ly ),
            - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
            + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
            + fgz2 * ( gx*gz*ly - gy*gz*lx );

    // Put together two jacobians
    Matrix<double, 6, 2> Jac_r2x(6, 2);
    Jac_r2x << Jac_sr2x, Jac_er2x;

    info_pose = Jac_r2x * cov_r.inverse() * Jac_r2x.transpose();
    //    Matrix<double, 2, 6> Jac_r2x_inv = (Jac_r2x.transpose()*Jac_r2x).inverse() * Jac_r2x.transpose();
    //    cov_pose = Jac_r2x_inv.transpose() * cov_r * Jac_r2x_inv;

}


void StereoFrameHandler::getPoseInfoPoint(const Matrix4d DT_inv,
                                          const StVO::PointFeature * point,
                                          Matrix<double, 6, 6> & info_pose) {

    Matrix3d J_dt = DT_inv.block(0,0,3,3);

    // get the cov matrix of residual
    Vector3d cur_P_tmp = J_dt * point->P + DT_inv.col(3).head(3);
    Vector2d pl_proj = cam->projection( cur_P_tmp );

    // projection error
    Vector2d err_i    = pl_proj - point->pl_obs;

    // estimate variables for J, H, and g
    double gx   = cur_P_tmp(0);
    double gy   = cur_P_tmp(1);
    double gz   = cur_P_tmp(2);
    double gz2  = gz*gz;
    double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
    double dx   = err_i(0);
    double dy   = err_i(1);

    // jacobian
    Vector6d J_aux;
    J_aux << + fgz2 * dx * gz,
            + fgz2 * dy * gz,
            - fgz2 * ( gx*dx + gy*dy ),
            - fgz2 * ( gx*gy*dx + gy*gy*dy + gz*gz*dy ),
            + fgz2 * ( gx*gx*dx + gz*gz*dx + gx*gy*dy ),
            + fgz2 * ( gx*gz*dy - gy*gz*dx );

    // assume norm distribution for point residual
    info_pose = J_aux * J_aux.transpose();
}



void StereoFrameHandler::updateEndPointByRatio(StVO::LineFeature * line) {
    //
    if ( fabs(line->cutRatio[0]) < 0.0001 && fabs(line->cutRatio[1]) < 0.0001 )
        return ;
    //
    if ( fabs(line->cutRatio[0]) > 0.0001 ) {
        // update starting point
        Vector3d sP_cut = (1 - line->cutRatio[0]) * line->sP + line->cutRatio[0] * line->eP;
        line->sP = sP_cut;
        line->spl = this->cam->projection(sP_cut);
        line->sdisp = this->cam->getDisparity(sP_cut(2));
    }
    if ( fabs(line->cutRatio[1]) > 0.0001 ) {
        // update ending point
        Vector3d eP_cut = (1 - line->cutRatio[1]) * line->eP + line->cutRatio[1] * line->sP;
        line->eP = eP_cut;
        line->epl = this->cam->projection(eP_cut);
        line->edisp = this->cam->getDisparity(eP_cut(2));
    }
}


// CURRENT IMPL
// as demonstrated via simulation, the pose covariance matrix being convex wrt the line cutting ratio
// therefore a simple gradient descend approach suffix for searching optimal line cut ratio
void StereoFrameHandler::estimateProjUncertainty_descent(const double stepCutRatio,
                                                         const double rngCutRatio[2]) {
    //
    if (matched_ls.size() == 0)
        return ;
    //
    //    Matrix4d rel_Tfw = prev_frame->Tfw.inverse() * curr_frame->Tfw;
    Matrix4d DT_inv = curr_frame->Tfw.inverse() * prev_frame->Tfw;

    double neighborStep[8][2] = {
        { stepCutRatio, 0 },
        { -stepCutRatio, 0 },
        { 0, stepCutRatio },
        { 0, -stepCutRatio },
        { stepCutRatio, stepCutRatio },
        { stepCutRatio, -stepCutRatio },
        { -stepCutRatio, stepCutRatio },
        { -stepCutRatio, -stepCutRatio }
    };

    Vector2d J_loss;
    //
    for( list<StVO::LineFeature *>::iterator it = matched_ls.begin();
         it != matched_ls.end(); it++) {

        // [1] === define the perpendicular direction to minimize uncertainty ===
        //
        // NOTE
        // due to the fact that the line coe has been normalized from image frame
        // to camera frame, we need to devide the focu-length from J_loss accordingly
        // however the resulting volume seems to be too high; therefore here we only
        // multiply the ratio rather than the actually focu-length
        //
        J_loss(0) = (*it)->le_obs[0];
        J_loss(1) = (*it)->le_obs[1];
        //        J_loss(2) = (*it)->le_obs[2];

        (*it)->cutRatio[0] = 0;
        (*it)->cutRatio[1] = 0;
        Matrix<double, 6, 6> cov_x;
        getPoseCovOnLine( DT_inv, J_loss, *it, (*it)->cutRatio, cov_x );
        vector<double> ev;
        getEigenValue( cov_x, ev );
        //
        (*it)->volumeArr.clear();
        (*it)->volumeArr.push_back(ev[0] * ev[1]);

        while ((*it)->cutRatio[0] + (*it)->cutRatio[1] < 1.0) {

            bool hitLower = false;
            // search for 8 neighbor of <rt_1, rt_2>
            for (int j=0; j<8; ++j) {
                double cutRatio_tmp[2] = { (*it)->cutRatio[0] + neighborStep[j][0],
                                           (*it)->cutRatio[1] + neighborStep[j][1] };
                // apply the searching range on neighborhood
                if ( cutRatio_tmp[0] + cutRatio_tmp[1] >= 1.0 )
                    continue ;
                //
                if ( cutRatio_tmp[0] < rngCutRatio[0] || cutRatio_tmp[0] > rngCutRatio[1] )
                    continue ;
                //
                if ( cutRatio_tmp[1] < rngCutRatio[0] || cutRatio_tmp[1] > rngCutRatio[1] )
                    continue ;

                // check the volume
                getPoseCovOnLine( DT_inv, J_loss, *it, cutRatio_tmp, cov_x );
                getEigenValue( cov_x, ev );

                if ( ev[0] * ev[1] < (*it)->volumeArr.back() ) {
                    (*it)->volumeArr.push_back( ev[0] * ev[1] );
                    (*it)->cutRatio[0] = cutRatio_tmp[0];
                    (*it)->cutRatio[1] = cutRatio_tmp[1];
                    hitLower = true;
                }
            }

            // early termination
            if (hitLower == false)
                break ;
        }

        if ((*it)->cutRatio[0] > 0 || (*it)->cutRatio[1] > 0) {

            std::cout << "cut ratio = " << (*it)->cutRatio[0]
                      << "; " << (*it)->cutRatio[1] << std::endl;

            //            std::cout << "volume track: ";
            //            for (int kk=0; kk<(*it)->volumeArr.size(); ++kk)
            //                std::cout << (*it)->volumeArr[kk] << " ";
            //            std::cout << std::endl;

        }

        //
        Vector3d sP_cut = (1 - (*it)->cutRatio[0]) * (*it)->sP + (*it)->cutRatio[0] * (*it)->eP;
        Vector3d eP_cut = (1 - (*it)->cutRatio[1]) * (*it)->eP + (*it)->cutRatio[1] * (*it)->sP;
        Matrix3d cov_sP_cut = pow(1 - (*it)->cutRatio[0], 2) * (*it)->covSpt3D +
                pow((*it)->cutRatio[0], 2) * (*it)->covEpt3D;
        Matrix3d cov_eP_cut = pow(1 - (*it)->cutRatio[1], 2) * (*it)->covEpt3D +
                pow((*it)->cutRatio[1], 2) * (*it)->covSpt3D;
        Matrix3d J_dt = DT_inv.block(0,0,3,3);

        // get the cov matrix of residual
        (*it)->covlPt2D.clear();
        (*it)->covlPt2D.push_back( MatrixXd::Identity(3, 3) );
        //
        Matrix3d J_proj;
        Vector3d cur_sP_cut = J_dt * sP_cut + DT_inv.col(3).head(3);
        curr_frame->getJacob3D_2D(cur_sP_cut[0], cur_sP_cut[1], cur_sP_cut[2], J_proj);
        (*it)->covlPt2D.push_back( J_proj * J_dt * cov_sP_cut * J_dt.transpose() * J_proj.transpose() );
        //
        Vector3d cur_eP_cut = J_dt * eP_cut + DT_inv.col(3).head(3);
        curr_frame->getJacob3D_2D(cur_eP_cut[0], cur_eP_cut[1], cur_eP_cut[2], J_proj);
        (*it)->covlPt2D.push_back( J_proj * J_dt * cov_eP_cut * J_dt.transpose() * J_proj.transpose() );
        //
        (*it)->covlPt2D.push_back( MatrixXd::Identity(3, 3) );

        //
        (*it)->lPt2D.clear();
        (*it)->lPt2D.push_back( this->cam->projection(J_dt * (*it)->sP + DT_inv.col(3).head(3)) );
        (*it)->lPt2D.push_back( this->cam->projection(cur_sP_cut) );
        (*it)->lPt2D.push_back( this->cam->projection(cur_eP_cut) );
        (*it)->lPt2D.push_back( this->cam->projection(J_dt * (*it)->eP + DT_inv.col(3).head(3)) );

        //
        (*it)->fst_idx = 0; // 1;
        (*it)->lst_idx = 3; // 2;
        (*it)->sP = sP_cut;
        (*it)->eP = eP_cut;
        (*it)->spl = this->cam->projection(sP_cut);
        (*it)->epl = this->cam->projection(eP_cut);
        (*it)->sdisp = this->cam->getDisparity(sP_cut(2));
        (*it)->edisp = this->cam->getDisparity(eP_cut(2));
    }
}


// EXPERIMENTAL
// as proved in CVPR submission, the minimal problem to be solved is P3L; meanwhile the min-eigen
// (maybe max-Vol as well) should be solved per P3L.  By exploiting the sub-modular property of
// min-eigen and max-vol, we approach the sub-optimal solution towards line cutting ratio with
// greedy approach
void StereoFrameHandler::estimateProjUncertainty_submodular(const double stepCutRatio,
                                                            const double rngCutRatio[2]) {
    //
    if (matched_ls.size() == 0)
        return ;
    //
    double neighborStep[8][2] = {
        { stepCutRatio, 0 },
        { -stepCutRatio, 0 },
        { 0, stepCutRatio },
        { 0, -stepCutRatio },
        { stepCutRatio, stepCutRatio },
        { stepCutRatio, -stepCutRatio },
        { -stepCutRatio, stepCutRatio },
        { -stepCutRatio, -stepCutRatio }
    };
    //    Matrix4d rel_Tfw = prev_frame->Tfw.inverse() * curr_frame->Tfw;
    Matrix4d DT_inv = curr_frame->Tfw.inverse() * prev_frame->Tfw;
    Vector2d J_loss;
    Matrix<double, 6, 6> invCov_sum = Matrix6d::Zero();
    for( list<StVO::LineFeature *>::iterator it = matched_ls.begin();
         it != matched_ls.end(); it++) {
        // compute the initial info matrix per line
        J_loss(0) = (*it)->le_obs[0];
        J_loss(1) = (*it)->le_obs[1];
        //
        (*it)->cutRatio[0] = 0;
        (*it)->cutRatio[1] = 0;
        getPoseInfoOnLine( DT_inv, J_loss, *it, (*it)->cutRatio, (*it)->invCovPose );
        //
        invCov_sum += (*it)->invCovPose;
    }

    // add the point term to the base info mat
    for( list<StVO::PointFeature *>::iterator it = matched_pt.begin();
         it != matched_pt.end(); it++) {
        // compute the initial info matrix per line
        Matrix<double, 6, 6> invCovTmp;
        getPoseInfoPoint( DT_inv, *it, invCovTmp );
        //
        invCov_sum += invCovTmp;
    }

    // iterativly searching for optimal cut ratio per line
    for( list<StVO::LineFeature *>::iterator it = matched_ls.begin();
         it != matched_ls.end(); it++) {
        //
        // prepare the param for cutting current line
        J_loss(0) = (*it)->le_obs[0];
        J_loss(1) = (*it)->le_obs[1];
        //
        vector<double> metricArr;
        double metric_tmp;
        if (Config::cutWithMaxVol()) {
            //            metric_tmp = getLogVolume( invCov_sum );
            metric_tmp = logdet(invCov_sum);
            //            if (fabs(metric_tmp - getLogVolume( invCov_sum )) > 0.0001)
            //                std::cout << "ERROR 1!" << std::endl;
        }
        else
            metric_tmp = getMinEigenValue( invCov_sum );
        metricArr.push_back( metric_tmp );

        // TODO
        // add two more init points to complete the greedy single-line cut
        // add condition check on whether line cutting is legit or not
        //
        // deduct current info matrix from the sum matrix
        invCov_sum -= (*it)->invCovPose;
        while ((*it)->cutRatio[0] + (*it)->cutRatio[1] <= 1.0) {

            bool hitLower = false;
            double cutRatio_cand[2];
            Matrix<double, 6, 6> invCovPose_cand;

            double metric_init = metricArr.back();

            // search for 8 neighbor of <rt_1, rt_2>
            for (int j=0; j<8; ++j) {
                double cutRatio_tmp[2] = { (*it)->cutRatio[0] + neighborStep[j][0],
                                           (*it)->cutRatio[1] + neighborStep[j][1] };
                // apply the searching range on neighborhood
                if ( cutRatio_tmp[0] + cutRatio_tmp[1] > 1.0 )
                    continue ;
                //
                if ( cutRatio_tmp[0] < rngCutRatio[0] || cutRatio_tmp[0] > rngCutRatio[1] )
                    continue ;
                //
                if ( cutRatio_tmp[1] < rngCutRatio[0] || cutRatio_tmp[1] > rngCutRatio[1] )
                    continue ;

                // check the volume
                Matrix<double, 6, 6> invCov_tmp;
                getPoseInfoOnLine( DT_inv, J_loss, (*it), cutRatio_tmp, invCov_tmp );
                if (Config::cutWithMaxVol()) {
                    //                    metric_tmp = getLogVolume( invCov_tmp + invCov_sum );
                    metric_tmp = logdet(invCov_tmp + invCov_sum);
                    //                    if (fabs(metric_tmp - getLogVolume( invCov_tmp + invCov_sum )) > 0.0001)
                    //                        std::cout << "ERROR 2!" << std::endl;
                }
                else
                    metric_tmp = getMinEigenValue( invCov_tmp + invCov_sum );

                // search for the max-vol solution
                if ( metric_tmp > metric_init ) {
                    metric_init = metric_tmp;
                    // metricArr.push_back( metric_tmp );
                    // accept the solution as candidate
                    cutRatio_cand[0]    = cutRatio_tmp[0];
                    cutRatio_cand[1]    = cutRatio_tmp[1];
                    invCovPose_cand     = invCov_tmp;
                    hitLower = true;
                }
            }

            if (hitLower == true) {
                // take the best move
                // update the cut ratio for current line
                (*it)->cutRatio[0]  = cutRatio_cand[0];
                (*it)->cutRatio[1]  = cutRatio_cand[1];
                (*it)->invCovPose   = invCovPose_cand;
                metricArr.push_back(metric_init);
            }
            else
                // early termination
                break ;
        }

        // //
        // // print out info after solving the P3L line cutting problem
        // if ((*it)->cutRatio[0] > 0 || (*it)->cutRatio[1] > 0) {

        //     std::cout << "cut ratio = " << (*it)->cutRatio[0]
        //               << "; " << (*it)->cutRatio[1] << std::endl;

        //     //            std::cout << "volume track: ";
        //     //            for (int kk=0; kk<volumeArr.size(); ++kk)
        //     //                std::cout << volumeArr[kk] << " ";
        //     //            std::cout << std::endl;
        // }

        // update the info in LineFeature object accordingly
        updateEndPointByRatio( *it );
        // add new current info matrix back to sum matrix
        invCov_sum += (*it)->invCovPose;
    }
}


void StereoFrameHandler::greedySolveLineCut_P3L(const Matrix4d DT_inv,
                                                const double stepCutRatio,
                                                const double rngCutRatio[2], vector<StVO::LineFeature *> line_P3L) {

    //
    assert(line_P3L.size() == 3);

    double neighborStep[8][2] = {
        { stepCutRatio, 0 },
        { -stepCutRatio, 0 },
        { 0, stepCutRatio },
        { 0, -stepCutRatio },
        { stepCutRatio, stepCutRatio },
        { stepCutRatio, -stepCutRatio },
        { -stepCutRatio, stepCutRatio },
        { -stepCutRatio, -stepCutRatio }
    };

    for (int i=0; i<line_P3L.size(); ++i) {
        // solving the line cut ratio for line i while keeping the other 2 fixed
        Matrix<double, 6, 6> invCov_tmp, invCov_compl;
        if (i == 0)
            invCov_compl = line_P3L[1]->invCovPose + line_P3L[2]->invCovPose;
        else if (i == 1)
            invCov_compl = line_P3L[0]->invCovPose + line_P3L[2]->invCovPose;
        else
            invCov_compl = line_P3L[0]->invCovPose + line_P3L[1]->invCovPose;

        Vector2d J_loss;
        J_loss(0) = line_P3L[i]->le_obs[0];
        J_loss(1) = line_P3L[i]->le_obs[1];
        //
        vector<double> volumeArr;
        double vol_tmp;
        if (Config::cutWithMaxVol())
            vol_tmp = getLogVolume( line_P3L[i]->invCovPose + invCov_compl );
        else
            vol_tmp = getMinEigenValue( line_P3L[i]->invCovPose + invCov_compl );
        volumeArr.push_back( vol_tmp );

        while (line_P3L[i]->cutRatio[0] + line_P3L[i]->cutRatio[1] < 1.0) {

            bool hitLower = false;
            // search for 8 neighbor of <rt_1, rt_2>
            for (int j=0; j<8; ++j) {
                double cutRatio_tmp[2] = { line_P3L[i]->cutRatio[0] + neighborStep[j][0],
                                           line_P3L[i]->cutRatio[1] + neighborStep[j][1] };
                // apply the searching range on neighborhood
                if ( cutRatio_tmp[0] + cutRatio_tmp[1] >= 1.0 )
                    continue ;
                //
                if ( cutRatio_tmp[0] < rngCutRatio[0] || cutRatio_tmp[0] > rngCutRatio[1] )
                    continue ;
                //
                if ( cutRatio_tmp[1] < rngCutRatio[0] || cutRatio_tmp[1] > rngCutRatio[1] )
                    continue ;

                // check the volume
                getPoseInfoOnLine( DT_inv, J_loss, line_P3L[i], cutRatio_tmp, invCov_tmp );
                if (Config::cutWithMaxVol())
                    vol_tmp = getLogVolume( invCov_tmp + invCov_compl );
                else
                    vol_tmp = getMinEigenValue( invCov_tmp + invCov_compl );

                // search for the max-vol solution
                if ( vol_tmp > volumeArr.back() ) {
                    volumeArr.push_back( vol_tmp );
                    // update the cut ratio for current line
                    line_P3L[i]->cutRatio[0] = cutRatio_tmp[0];
                    line_P3L[i]->cutRatio[1] = cutRatio_tmp[1];
                    line_P3L[i]->invCovPose = invCov_tmp;
                    hitLower = true;
                }
            }

            // early termination
            if (hitLower == false)
                break ;
        }

        //
        // print out info after solving the P3L line cutting problem
        if (line_P3L[i]->cutRatio[0] > 0 || line_P3L[i]->cutRatio[1] > 0) {

            std::cout << "cut ratio = " << line_P3L[i]->cutRatio[0]
                      << "; " << line_P3L[i]->cutRatio[1] << std::endl;

            //            std::cout << "volume track: ";
            //            for (int kk=0; kk<volumeArr.size(); ++kk)
            //                std::cout << volumeArr[kk] << " ";
            //            std::cout << std::endl;
        }
    }

}


void StereoFrameHandler::optimizePose()
{

    // definitions
    Matrix6d DT_cov;
    Matrix4d DT, DT_;
    //    Vector6d DT_cov_eig;
    double   err;

    // set init pose
    DT     = prev_frame->DT;
    DT_cov = prev_frame->DT_cov;

    // solver
    if( n_inliers > Config::minFeatures() )
    {
        // optimize
        DT_ = DT;
        gaussNewtonOptimization(DT_,DT_cov,err,Config::maxIters());
        // remove outliers (implement some logic based on the covariance's eigenvalues and optim error)
        if( is_finite(DT_) )
        {
            removeOutliers(DT_);
            // refine without outliers
            if( n_inliers > Config::minFeatures() )
                gaussNewtonOptimization(DT,DT_cov,err,Config::maxItersRef());
            else
            {
                DT     = Matrix4d::Identity();
                DT_cov = Matrix6d::Zero();
            }
        }
        else
        {
            DT     = Matrix4d::Identity();
            DT_cov = Matrix6d::Zero();
        }
    }
    else
    {
        DT     = Matrix4d::Identity();
        DT_cov = Matrix6d::Zero();
    }

    // set estimated pose
    if( is_finite(DT) )
    {
        curr_frame->DT     = inverse_se3( DT );
        //        Vector3d tran_tmp = curr_frame->DT.col(3).head(3);
        //        //
        //        if ( tran_tmp.norm() > 10.0 ) {
        //            // reject pose estimation
        //            curr_frame->DT = Matrix4d::Identity();;
        //        }

        curr_frame->Tfw    = prev_frame->Tfw * curr_frame->DT;
        curr_frame->DT_cov = DT_cov;
        SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
        curr_frame->DT_cov_eig = eigensolver.eigenvalues();
        curr_frame->Tfw_cov = unccomp_se3( prev_frame->Tfw, prev_frame->Tfw_cov, DT_cov );
        curr_frame->err_norm   = err;
    }
    else
    {
        curr_frame->DT     = Matrix4d::Identity();
        curr_frame->Tfw    = prev_frame->Tfw;
        curr_frame->Tfw_cov= prev_frame->Tfw_cov;
        curr_frame->DT_cov = DT_cov;
        SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
        curr_frame->DT_cov_eig = eigensolver.eigenvalues();
        curr_frame->err_norm   = -1.0;
    }

}

void StereoFrameHandler::optimizePose(Matrix4d DT_ini)
{

    // definitions
    Matrix6d DT_cov;
    Matrix4d DT, DT_;
    //    Vector6d DT_cov_eig;
    double   err;

    // set init pose
    DT     = DT_ini;
    DT_cov = prev_frame->DT_cov;

    // solver
    if( n_inliers > Config::minFeatures() )
    {
        // optimize
        DT_ = DT;
        gaussNewtonOptimization(DT_,DT_cov,err,Config::maxIters());
        // remove outliers (implement some logic based on the covariance's eigenvalues and optim error)
        if( is_finite(DT_) )
        {
            removeOutliers(DT_);
            // refine without outliers
            if( n_inliers > Config::minFeatures() )
                gaussNewtonOptimization(DT,DT_cov,err,Config::maxItersRef());
            else
            {
                DT     = Matrix4d::Identity();
                DT_cov = Matrix6d::Zero();
            }
        }
        else
        {
            DT     = Matrix4d::Identity();
            DT_cov = Matrix6d::Zero();
        }
    }
    else
    {
        DT     = Matrix4d::Identity();
        DT_cov = Matrix6d::Zero();
    }

    // set estimated pose
    if( is_finite(DT) )
    {
        curr_frame->DT     = inverse_se3( DT );
        //        std::cout << "DT = " << DT << std::endl;
        //        std::cout << "inverse_se3(DT) = " << curr_frame->DT << std::endl;
        //        std::cout << "curr_frame->Tfw = " << prev_frame->Tfw * curr_frame->DT << std::endl;

        // verify the magnitude of "step"
        if ( curr_frame->DT.col(3).head(3).norm() <
             Config::motionStepThres() * (curr_frame->time_stamp - prev_frame->time_stamp) ) {
            // accept the pose optimization; update the pose of current frame accordingly
            curr_frame->Tfw    = prev_frame->Tfw * curr_frame->DT;
            curr_frame->DT_cov = DT_cov;
            SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
            curr_frame->DT_cov_eig = eigensolver.eigenvalues();
            curr_frame->Tfw_cov = unccomp_se3( prev_frame->Tfw, prev_frame->Tfw_cov, DT_cov );
            curr_frame->err_norm   = err;
        }
        else
        {
            // roll back to identical matrix
            curr_frame->DT     = Matrix4d::Identity();
            curr_frame->Tfw    = prev_frame->Tfw;
            curr_frame->Tfw_cov= prev_frame->Tfw_cov;
            curr_frame->DT_cov = DT_cov;
            SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
            curr_frame->DT_cov_eig = eigensolver.eigenvalues();
            curr_frame->err_norm   = -1.0;
        }
        //
        numFrameLoss = 0;
    }
    else
    {
        // simply setting static motion
        curr_frame->DT     = Matrix4d::Identity();
        curr_frame->Tfw    = prev_frame->Tfw;
        curr_frame->Tfw_cov= prev_frame->Tfw_cov;
        curr_frame->DT_cov = DT_cov;
        SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
        curr_frame->DT_cov_eig = eigensolver.eigenvalues();
        curr_frame->err_norm   = -1.0;
        //
        numFrameLoss ++;
    }

}

void StereoFrameHandler::gaussNewtonOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters)
{
    Matrix6d H;
    Vector6d g, DT_inc;
    double err, err_prev = 999999999.9;
    for( int iters = 0; iters < max_iters; iters++)
    {
        // estimate hessian and gradient (select)
        optimizeFunctions( DT, H, g, err );
        // if the difference is very small stop
        if( ( abs(err-err_prev) < Config::minErrorChange() ) || ( err < Config::minError()) )
            break;
        // update step
        LDLT<Matrix6d> solver(H);
        DT_inc = solver.solve(g);
        DT  << DT * inverse_se3( expmap_se3(DT_inc) );
        // if the parameter change is small stop (TODO: change with two parameters, one for R and another one for t)
        if( DT_inc.norm() < numeric_limits<double>::epsilon() )
            break;
        // update previous values
        err_prev = err;
    }
    DT_cov = H.inverse();
    err_   = err;
}

void StereoFrameHandler::removeOutliers(Matrix4d DT)
{

    vector<double> res_p, res_l, ove_l;

    // point features
    int iter = 0;
    for( list<PointFeature*>::iterator it = matched_pt.begin(); it!=matched_pt.end(); it++, iter++)
    {
        // projection error
        Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);
        Vector2d pl_proj = cam->projection( P_ );
        res_p.push_back( ( pl_proj - (*it)->pl_obs ).norm() * sqrt((*it)->sigma2) );
    }

    // line segment features
    for( list<LineFeature*>::iterator it = matched_ls.begin(); it!=matched_ls.end(); it++, iter++)
    {
        // projection error
        Vector3d sP_ = DT.block(0,0,3,3) * (*it)->sP + DT.col(3).head(3);
        Vector3d eP_ = DT.block(0,0,3,3) * (*it)->eP + DT.col(3).head(3);
        Vector2d spl_proj = cam->projection( sP_ );
        Vector2d epl_proj = cam->projection( eP_ );
        Vector3d l_obs    = (*it)->le_obs;
        Vector2d err_li;
        err_li(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
        err_li(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
        res_l.push_back( err_li.norm() * sqrt((*it)->sigma2) );
    }

    // estimate mad standard deviation
    double inlier_th_p =  Config::inlierK() * vector_stdv_mad( res_p );
    double inlier_th_l =  Config::inlierK() * vector_stdv_mad( res_l );
    //inlier_th_p = sqrt(7.815);
    //inlier_th_l = sqrt(7.815);

    // filter outliers
    iter = 0;
    for( list<PointFeature*>::iterator it = matched_pt.begin(); it!=matched_pt.end(); it++, iter++)
    {
        if( res_p[iter] > inlier_th_p )
        {
            (*it)->inlier = false;
            n_inliers--;
            n_inliers_pt--;
        }
    }
    iter = 0;
    for( list<LineFeature*>::iterator it = matched_ls.begin(); it!=matched_ls.end(); it++, iter++)
    {
        if( res_l[iter] > inlier_th_l )
        {
            (*it)->inlier = false;
            n_inliers--;
            n_inliers_ls--;
        }
    }

}

void StereoFrameHandler::optimizeFunctions(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e )
{

    // define hessians, gradients, and residuals
    Matrix6d H_l, H_p;
    Vector6d g_l, g_p;
    double   e_l = 0.0, e_p = 0.0, S_l, S_p;
    H   = Matrix6d::Zero(); H_l = H; H_p = H;
    g   = Vector6d::Zero(); g_l = g; g_p = g;
    e   = 0.0;

    // point features
    int N_p = 0;
    for( list<PointFeature*>::iterator it = matched_pt.begin(); it!=matched_pt.end(); it++)
    {
        if( (*it)->inlier )
        {
            Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);
            Vector2d pl_proj = cam->projection( P_ );
            // projection error
            Vector2d err_i    = pl_proj - (*it)->pl_obs;
            double err_i_norm = err_i.norm();
            // estimate variables for J, H, and g
            double gx   = P_(0);
            double gy   = P_(1);
            double gz   = P_(2);
            double gz2  = gz*gz;
            double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            double dx   = err_i(0);
            double dy   = err_i(1);
            // jacobian
            Vector6d J_aux;
            J_aux << + fgz2 * dx * gz,
                    + fgz2 * dy * gz,
                    - fgz2 * ( gx*dx + gy*dy ),
                    - fgz2 * ( gx*gy*dx + gy*gy*dy + gz*gz*dy ),
                    + fgz2 * ( gx*gx*dx + gz*gz*dx + gx*gy*dy ),
                    + fgz2 * ( gx*gz*dy - gy*gz*dx );
            J_aux = J_aux / std::max(Config::homogTh(), err_i_norm);
            // if employing robust cost function
            double w  = 1.0;
            double s2 = (*it)->sigma2;
            w = 1.0 / ( 1.0 + err_i_norm * err_i_norm * s2 );
            // update hessian, gradient, and error
            H_p += J_aux * J_aux.transpose() * w;
            g_p += J_aux * err_i_norm * w;
            e_p += err_i_norm * err_i_norm * w;
            N_p++;
        }
    }

    // line segment features
    int N_l = 0;
    for( list<LineFeature*>::iterator it = matched_ls.begin(); it!=matched_ls.end(); it++)
    {
        if( (*it)->inlier )
        {
            Vector3d sP_ = DT.block(0,0,3,3) * (*it)->sP + DT.col(3).head(3);
            Vector2d spl_proj = cam->projection( sP_ );
            Vector3d eP_ = DT.block(0,0,3,3) * (*it)->eP + DT.col(3).head(3);
            Vector2d epl_proj = cam->projection( eP_ );
            Vector3d l_obs = (*it)->le_obs;
            // projection error
            Vector2d err_i;
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            double err_i_norm = err_i.norm();
            // estimate variables for J, H, and g
            // -- start point
            double gx   = sP_(0);
            double gy   = sP_(1);
            double gz   = sP_(2);
            double gz2  = gz*gz;
            double fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            double ds   = err_i(0);
            double de   = err_i(1);
            double lx   = l_obs(0);
            double ly   = l_obs(1);
            Vector6d Js_aux;
            Js_aux << + fgz2 * lx * gz,
                    + fgz2 * ly * gz,
                    - fgz2 * ( gx*lx + gy*ly ),
                    - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                    + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                    + fgz2 * ( gx*gz*ly - gy*gz*lx );
            // -- end point
            gx   = eP_(0);
            gy   = eP_(1);
            gz   = eP_(2);
            gz2  = gz*gz;
            fgz2 = cam->getFx() / std::max(Config::homogTh(),gz2);
            Vector6d Je_aux, J_aux;
            Je_aux << + fgz2 * lx * gz,
                    + fgz2 * ly * gz,
                    - fgz2 * ( gx*lx + gy*ly ),
                    - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                    + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                    + fgz2 * ( gx*gz*ly - gy*gz*lx );
            // jacobian
            J_aux = ( Js_aux * ds + Je_aux * de ) / std::max(Config::homogTh(),err_i_norm);
            // if employing robust cost function
            double w  = 1.0;
            double s2 = (*it)->sigma2;
            w = 1.0 / ( 1.0 + err_i_norm * err_i_norm * s2 );
            // estimating overlap between line segments
            /*bool has_overlap = false;
            double overlap = 1.0;
            if( has_overlap )
                overlap = lineSegmentOverlap( (*it)->spl, (*it)->epl, spl_proj, epl_proj );
            w *= overlap;*/
            // update hessian, gradient, and error
            H_l += J_aux * J_aux.transpose() * w;
            g_l += J_aux * err_i_norm * w;
            e_l += err_i_norm * err_i_norm * w;
            N_l++;
        }

    }

    // sum H, g and err from both points and lines
    H = H_p + H_l;
    g = g_p + g_l;
    e = e_p + e_l;

    // normalize error
    e /= (N_l+N_p);

}

double StereoFrameHandler::lineSegmentOverlap( Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj  )
{

    Vector2d l = spl_obs - epl_obs;
    double lxx  = l(0)*l(0);
    double lyy  = l(1)*l(1);
    double lxy  = l(0)*l(1);
    double lxy2 = 1.f/(lxx+lyy);

    Matrix2d u;
    u << lxx, lxy, lxy, lyy;
    u = u * lxy2;
    Vector2d v;
    Matrix2d vm;
    vm << lxx, -lxy, -lxy, lyy;
    v = lxy2 * vm * spl_obs;

    Vector2d sp;
    sp << u * spl_proj + v;
    Vector2d ep;
    ep << u * epl_proj + v;

    double lnorm  = 1.f / l.norm();
    double seno   = -l(0)*lnorm;
    double coseno = -l(1)*lnorm;

    Matrix2d rot; rot << coseno, -seno, seno, coseno;
    Vector2d sl, el;

    sl     << rot * spl_obs;
    el     << rot * epl_obs;
    sp     << rot * sp;
    ep     << rot * ep;

    double sln    = min(sl(1), el(1));
    double eln    = max(sl(1), el(1));
    double spn    = min(sp(1), ep(1));
    double epn    = max(sp(1), ep(1));

    double length = eln-spn;

    double overlap;
    if ( (epn < sln) || (spn > eln) )
        overlap = 0.f;
    else{
        if ( (epn>eln) && (spn<sln) )
            overlap = eln-sln;
        else
            overlap = min(eln,epn) - max(sln,spn);
    }

    if(length>0.01f)
        overlap = overlap / length;
    else
        overlap = 0.f;

    return overlap;

}

/*  slam functions  */

bool StereoFrameHandler::needNewKF()
{

    // if the previous KF was a KF, update the entropy_first_prevKF value
    if( prev_f_iskf )
    {
        entropy_first_prevKF = 3.0*(1.0+log(2.0*acos(-1))) + 0.5*log( curr_frame->DT_cov.determinant() );
        prev_f_iskf = false;
    }

    // check geometric distances from previous KF
    //    Matrix4d DT = inverse_se3( curr_frame->Tfw ) * T_prevKF;
    //    Vector6d dX = logmap_se3( DT );
    //    double t = dX.head(3).norm();
    //    double r = dX.tail(3).norm() * 180.f / CV_PI;

    // check cumulated covariance from previous KF
    Matrix6d adjTprevkf = adjoint_se3( T_prevKF );
    Matrix6d covDTinv   = uncTinv_se3( curr_frame->DT, curr_frame->DT_cov );
    cov_prevKF_currF += adjTprevkf * covDTinv * adjTprevkf.transpose();
    double entropy_curr  = 3.0*(1.0+log(2.0*acos(-1))) + 0.5*log( cov_prevKF_currF.determinant() );
    double entropy_ratio = entropy_curr / entropy_first_prevKF;

    // decide if a new KF is needed
    if ( numFrameSinceKeyframe > Config::maxKFNumFrames() ||
         entropy_ratio < Config::minEntropyRatio() ||
         std::isnan(entropy_ratio) ||
         std::isinf(entropy_ratio) ||
         ( curr_frame->DT_cov == Matrix6d::Zero() && curr_frame->DT == Matrix4d::Identity() ) )
    {
        cout << endl << "Entropy ratio: " << entropy_ratio   << endl;
        return true;
    }
    else
    {
        cout << endl << "Entropy ratio: " << entropy_ratio   << endl;
        cout << endl << "No new KF needed" << endl << endl;
        return false;
    }

}

void StereoFrameHandler::currFrameIsKF()
{
    numFrameSinceKeyframe = 0;

    // restart point indices
    int idx_pt = 0;
    for( vector<PointFeature*>::iterator it = curr_frame->stereo_pt.begin(); it != curr_frame->stereo_pt.end(); it++)
    {
        (*it)->idx = idx_pt;
        idx_pt++;
    }

    // restart line indices
    int idx_ls = 0;
    for( vector<LineFeature*>::iterator it = curr_frame->stereo_ls.begin(); it != curr_frame->stereo_ls.end(); it++)
    {
        (*it)->idx = idx_ls;
        idx_ls++;
    }

    // update KF
    curr_frame->Tfw     = Matrix4d::Identity();
    curr_frame->Tfw_cov = Matrix6d::Identity();

    // update SLAM variables for KF decision
    T_prevKF = curr_frame->Tfw;
    cov_prevKF_currF = Matrix6d::Zero();
    prev_f_iskf = true;

}


//
void StereoFrameHandler::plotLine3D( bool save_screen_shot ) {
    //
    assert(this->viz_3D_lines != NULL);
    //
    if (this->viz_3D_lines == NULL) {
        // re-init the window when being closed
        viz_3D_lines = new cv::viz::Viz3d("3D Line Visualization");
    }
    //    this->viz_3D_lines->removeAllWidgets();

    // add line widget for each stereo line in curr_frame
    for (int i=0; i<curr_frame->stereo_ls.size(); ++i) {
        //
        Vector4d spt_4d; spt_4d << curr_frame->stereo_ls[i]->sP, 1;
        Vector4d ept_4d; ept_4d << curr_frame->stereo_ls[i]->eP, 1;
        // apply the pose transform on top of the cam frame
        spt_4d = curr_frame->Tfw * spt_4d;
        ept_4d = curr_frame->Tfw * ept_4d;
        // assemble the endpoint in world frame into line object
        viz::WLine line_3d(Point3f(spt_4d(0), spt_4d(1), spt_4d(2)),
                           Point3f(ept_4d(0), ept_4d(1), ept_4d(2)));
        line_3d.setRenderingProperty(viz::LINE_WIDTH, 1.0);
        //
        const Scalar color_rnd = curr_frame->stereo_ls[i]->color;
        line_3d.setColor(color_rnd);
        std::string line_wid = std::to_string(curr_frame->frame_idx);
        line_wid += "-";
        line_wid += std::to_string(i);
        this->viz_3D_lines->showWidget(line_wid, line_3d);
    }

    // Add coordinate axes
    Matrix3d Kl_ori = this->cam->getK();
    cv::Matx33d Kl;
    CopyEigenToMatX(Kl_ori, Kl);
    //    for (int i=0; i<3; ++i) {
    //        for (int j=0; j<3; ++j) {
    //            Kl(i,j) = Kl_ori.at<double>(i,j);
    //        }
    //    }
    //    std::cout << Kl << std::endl;
    std::string frame_wid = "frame";
    frame_wid += std::to_string(curr_frame->frame_idx);
    this->viz_3D_lines->showWidget(frame_wid, viz::WCameraPosition(
                                       Kl, this->curr_frame->rgbImg_l, 0.3));
    //
    cv::Affine3f pAffine_curr;
    CopyEigenToAffine(curr_frame->Tfw, pAffine_curr);
    this->viz_3D_lines->setWidgetPose(frame_wid, pAffine_curr);

    cv::Mat Rvec(3, 1, CV_32F);
    Rvec.at<float>(0) = -0.526415;
    Rvec.at<float>(1) = -0.589073; // ;
    Rvec.at<float>(2) = -0.304148; // CV_PI / 4;
    cv::Affine3f viewPose = cv::Affine3f(
                Rvec,
                cv::Vec3f(3.9439, -4.32466, -3.38602)
                );
    this->viz_3D_lines->setViewerPose(pAffine_curr * viewPose);

    //    cv::Mat Rvec(3, 1, CV_64F);
    //    Rvec.at<double>(0) = -0.142623;
    //    Rvec.at<double>(1) = -2.18779; // ;
    //    Rvec.at<double>(2) = -0.283118; // CV_PI / 4;
    //    cv::Affine3d viewPose = cv::Affine3d(
    //                Rvec,
    //                cv::Vec3d(10.8226, -3.84858, 10.7423)
    //                );
    //    this->viz_3D_lines->setViewerPose(viewPose);

    //    std::cout << "start to plot 3D line!" << std::endl;
    this->viz_3D_lines->spinOnce(1, true);
    //    this->viz_3D_lines->spin();

    //    // for acquiring proper view port in debugger only!
    //    while(!this->viz_3D_lines->wasStopped()) {
    //        cv::Affine3d viewPose = this->viz_3D_lines->getViewerPose();
    //        this->viz_3D_lines->spinOnce(1, true);
    //        //
    //        std::cout << "current viewer pose: " << std::endl;
    //        //        for (int i=0; i<4; ++i) {
    //        //            for (int j=0; j<4; ++j)
    //        //                std::cout << viewPose.at<double>(i,j) << " ";
    //        //            std::cout << std::endl;
    //        //        }
    //        cv::Vec<double, 3> tvec = viewPose.translation();
    //        cv::Vec<double, 3> rvec = viewPose.rvec();
    //        std::cout << tvec[0] << " " << tvec[1] << " " << tvec[2] << "; ";
    //        std::cout << rvec[0] << " " << rvec[1] << " " << rvec[2] << std::endl;

    //    }

    if (save_screen_shot) {
        //
        std::string frame_wid = this->curr_frame->save_path + "/3DV/";
        frame_wid += std::to_string(curr_frame->frame_idx);
        frame_wid += ".png";
        this->viz_3D_lines->saveScreenshot(frame_wid);
    }

    this->viz_3D_lines->removeAllWidgets();
}

void StereoFrameHandler::plotMatchedLines(bool vizUncertainty,
                                          bool save_screen_shot) {
    // create new image to modify it
    cv::Mat img_l_aux;
    this->curr_frame->rgbImg_l.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);

    // Variables
    cv::Point       p,q,p1,q1,p2,q2;
    double          thick = 1.0;
    float sum=0;

    // plot line segment features
    int i =0;
n_inliers_ls = 0;

    for( list<LineFeature*>::iterator ls_it = matched_ls.begin();
         ls_it != matched_ls.end(); ls_it++)
    {
        if( (*ls_it)->inlier )
        {
            n_inliers_ls ++;

            Vector4d pt_world;
            Vector3d sP_;
            pt_world << (*ls_it)->sP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            sP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);
            //
            Vector3d eP_;
            pt_world << (*ls_it)->eP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            eP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);

            Vector2d spl_proj2 = cam->projectionNormalized( sP_ );
            Vector2d epl_proj2 = cam->projectionNormalized( eP_ );
            Vector3d l_obs    = (*ls_it)->le_obs;
            Vector2f err_li;
            err_li(0) = float(l_obs(0)) * float(spl_proj2(0)) + float(l_obs(1) * spl_proj2(1)) + float(l_obs(2));
            err_li(1) = float(l_obs(0) * epl_proj2(0) + l_obs(1) * epl_proj2(1) + l_obs(2));
            sum += err_li.squaredNorm();

            // draw optimized line
            Vector2d spl_proj1 = cam->projection( sP_ );
            Vector2d epl_proj1 = cam->projection( eP_ );
            p = cv::Point( int(spl_proj1(0)), int(spl_proj1(1)) );
            q = cv::Point( int(epl_proj1(0)), int(epl_proj1(1)) );
            line( img_l_aux, p, q, Scalar(0,0,255), thick);

            // draw init line
            //            p1 = cv::Point( int((line_projection_start[i])(0)) ,
            //                            int((line_projection_start[i])(1)) );
            //            q1 = cv::Point( int((line_projection_end[i])(0)) ,
            //                            int((line_projection_end[i])(1)) );
            p1 = cv::Point( int( (*ls_it)->lPt2D[0][0] ) ,
                    int( (*ls_it)->lPt2D[0][1] ) );
            q1 = cv::Point( int( (*ls_it)->lPt2D[(*ls_it)->lPt2D.size() - 1][0] ) ,
                    int( (*ls_it)->lPt2D[(*ls_it)->lPt2D.size() - 1][1] ) );
            //p1 = cv::Point( int((*ls_it)->spl_obs(0)) , int((*ls_it)->spl_obs(1)) );
            //q1 = cv::Point( int((*ls_it)->epl_obs(0)) , int((*ls_it)->epl_obs(1)) );
            line( img_l_aux, p1, q1, Scalar(255,0,0), thick);

            if (vizUncertainty) {
                for (int j=(*ls_it)->fst_idx; j<=(*ls_it)->lst_idx; ++j) {
                    //
                    //                    std::cout << "lPt2D = " << (*ls_it)->lPt2D[j][0] << ", " << (*ls_it)->lPt2D[j][1] << std::endl;
                    //                    std::cout << "cov_lpt = " << (*ls_it)->covlPt2D[j] << std::endl;

                    cv::Point lpt( (*ls_it)->lPt2D[j][0], (*ls_it)->lPt2D[j][1] );
                    // draw uncertainty level of init line
                    cv::Mat cov_lpt;
                    cv::eigen2cv( (*ls_it)->covlPt2D[j], cov_lpt );
                    //                    for (int ii=0; ii<2; ++ii)
                    //                        for (int jj=0; jj<2; ++jj)
                    //                            cov_lpt.at<double>(ii, jj) = double( (*ls_it)->covlPt2D[j](ii, jj) );

                    //
                    // NOTE
                    // the variable chi2 only affects the viz;
                    // for better viz a large chi2 is set here
                    //
                    double chi2 = 5.991; // 2.4477; //
                    cv::RotatedRect ellip_lpt = getErrorEllipse(chi2, lpt,
                                                                cov_lpt(cv::Range(0,2),
                                                                        cv::Range(0,2)));
                    //
                    bool skipEllp = false;
                    Point2f bpt_tmp[4];
                    ellip_lpt.points(bpt_tmp);
                    for (int ii=0; ii<4; ++ii) {
                        if (curr_frame->isOutOfFrame(bpt_tmp[ii].x, bpt_tmp[ii].y)) {
                            //                            std::cout << "ellipse out of frame: " << bpt_tmp[ii].x
                            //                                      << ", " << bpt_tmp[ii].y << std::endl;
                            skipEllp = true;
                            break ;
                        }
                    }
                    //
                    if (!skipEllp)
                        cv::ellipse(img_l_aux, ellip_lpt, cv::Scalar::all(255), 1);
                }
            }

            // draw detected line
            p2 = cv::Point( int((*ls_it)->spl_obs(0)) , int((*ls_it)->spl_obs(1)) );
            q2 = cv::Point( int((*ls_it)->epl_obs(0)) , int((*ls_it)->epl_obs(1)) );
            line( img_l_aux, p2, q2, Scalar(0,255,0), thick);
            i ++;
        }
    }
    //
    cv::putText(img_l_aux,
                std::to_string(i),
                cv::Point(25, 25), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                2.0, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2.0); // Thickness

    canvas_match_frames = img_l_aux;

    if (save_screen_shot) {
        //
        std::string frame_wid = this->curr_frame->save_path; // + "/optimize/";
        frame_wid += std::to_string(curr_frame->frame_idx);
        frame_wid += ".png";
        imwrite( frame_wid, this->canvas_match_frames );
    }
}

void StereoFrameHandler::plotMatchedPointLine(bool save_screen_shot) {
    // create new image to modify it
    cv::Mat img_l_aux;
    this->curr_frame->rgbImg_l.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);

    // Variables
    cv::Point       p,q,p1,q1,p2,q2;
    double          thick = 2.0; // 1.0;
    float sum = 0;

    n_inliers_ls = 0;
    n_inliers_pt = 0;

    // plot line segment features
    for( list<LineFeature*>::iterator ls_it = matched_ls.begin();
         ls_it != matched_ls.end(); ls_it++)
    {
        if( (*ls_it)->inlier )
        {
            // // draw init line
            // //            p1 = cv::Point( int((line_projection_start[i])(0)) ,
            // //                            int((line_projection_start[i])(1)) );
            // //            q1 = cv::Point( int((line_projection_end[i])(0)) ,
            // //                            int((line_projection_end[i])(1)) );
            // p1 = cv::Point( int( (*ls_it)->lPt2D[0][0] ) ,
            //         int( (*ls_it)->lPt2D[0][1] ) );
            // q1 = cv::Point( int( (*ls_it)->lPt2D[(*ls_it)->lPt2D.size() - 1][0] ) ,
            //         int( (*ls_it)->lPt2D[(*ls_it)->lPt2D.size() - 1][1] ) );
            // //p1 = cv::Point( int((*ls_it)->spl_obs(0)) , int((*ls_it)->spl_obs(1)) );
            // //q1 = cv::Point( int((*ls_it)->epl_obs(0)) , int((*ls_it)->epl_obs(1)) );
            // line( img_l_aux, p1, q1, Scalar(255,0,0), thick);

            // // draw projected line
            // p2 = cv::Point( int((*ls_it)->spl_obs(0)) , int((*ls_it)->spl_obs(1)) );
            // q2 = cv::Point( int((*ls_it)->epl_obs(0)) , int((*ls_it)->epl_obs(1)) );
            // line( img_l_aux, p2, q2, Scalar(0,255,0), thick);

            // draw optimized line
            Vector4d pt_world;
            Vector3d sP_;
            pt_world << (*ls_it)->sP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            sP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);
            //
            Vector3d eP_;
            pt_world << (*ls_it)->eP, 1;

            // cout << "pt bef = " << pt_world << endl;

            pt_world = this->curr_frame->DT.inverse() * pt_world;

            // cout << "pt aft = " <<  pt_world << endl;

            eP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);

            // draw optimized line
            Vector2d spl_proj1 = cam->projection( sP_ );
            Vector2d epl_proj1 = cam->projection( eP_ );
            p = cv::Point( int(spl_proj1(0)), int(spl_proj1(1)) );
            q = cv::Point( int(epl_proj1(0)), int(epl_proj1(1)) );

            // cout << p << endl;
            // cout << q << endl;

            if (p.x < 0 || p.x > cam->getWidth() || 
                p.y < 0 || p.y > cam->getHeight() || 
                q.x < 0 || q.x > cam->getWidth() || 
                q.y < 0 || q.y > cam->getHeight())
                continue ;

            n_inliers_ls ++;
            // line( img_l_aux, p, q, Scalar(0,0,255), thick);
            line( img_l_aux, p, q, Scalar(0,255,0), thick);
        }
    }
    //

    for( list<PointFeature*>::iterator pt_it = matched_pt.begin();
         pt_it != matched_pt.end(); pt_it++)
    {
        if( (*pt_it)->inlier )
        {
            // // draw detected point
            // p = cv::Point( int((*pt_it)->pl_obs(0)), int((*pt_it)->pl_obs(1)) );
            // circle( img_l_aux, p, 3.0, Scalar(0,255,0), thick);

            // draw projected point
            p = cv::Point( int((*pt_it)->pl(0)), int((*pt_it)->pl(1)) );

            if (p.x < 0 || p.x > cam->getWidth() || 
                p.y < 0 || p.y > cam->getHeight())
                continue ;

            n_inliers_pt ++;

            // circle( img_l_aux, p, 3.0, Scalar(0,0,255), thick);
            circle( img_l_aux, p, 3.0, Scalar(0,255,0), thick);
        }
    }


    canvas_match_frames = img_l_aux;

    if (save_screen_shot) {
        //
        //        std::string frame_wid = this->curr_frame->save_path + "/optimize/";
        std::string frame_wid = this->curr_frame->save_path;
        //        frame_wid += std::to_string(curr_frame->frame_idx);
        long time_stamp = curr_frame->time_stamp * pow(10, 9);
        frame_wid += std::to_string(time_stamp);

        frame_wid += ".png";
        imwrite( frame_wid, this->canvas_match_frames );
    }
}



void StereoFrameHandler::appendPlotCutLine(bool save_screen_shot) {

    // Variables
    cv::Point       p,q;
    double          thick = 2.0; // 1.0;

    // plot line segment features
    for( list<LineFeature*>::iterator ls_it = matched_ls.begin();
         ls_it != matched_ls.end(); ls_it++)
    {
        if( (*ls_it)->inlier )
        {
            // draw optimized line
            Vector4d pt_world;
            Vector3d sP_;
            pt_world << (*ls_it)->sP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            sP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);
            //
            Vector3d eP_;
            pt_world << (*ls_it)->eP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            eP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);

            // draw optimized line
            Vector2d spl_proj1 = cam->projection( sP_ );
            Vector2d epl_proj1 = cam->projection( eP_ );
            p = cv::Point( int(spl_proj1(0)), int(spl_proj1(1)) );
            q = cv::Point( int(epl_proj1(0)), int(epl_proj1(1)) );
            // line( img_l_aux, p, q, Scalar(0,0,255), thick);
            line( canvas_match_frames, p, q, Scalar(0,0,255), thick);
        }
    }
    //

        cv::putText(canvas_match_frames,
                "Points Matched: " + std::to_string(n_inliers_pt) + 
                "; Lines Matched: " + std::to_string(n_inliers_ls),
                cv::Point(10, 470), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                1.5, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2.0); // Thickness

    if (save_screen_shot && n_inliers_ls > 0) {
        //
        //        std::string frame_wid = this->curr_frame->save_path + "/optimize/";
        std::string frame_wid = this->curr_frame->save_path;
        //        frame_wid += std::to_string(curr_frame->frame_idx);
        long time_stamp = curr_frame->time_stamp * pow(10, 9);
        frame_wid += std::to_string(time_stamp);

        frame_wid += ".png";
        imwrite( frame_wid, this->canvas_match_frames );
    }
}




void StereoFrameHandler::appendPlotOptLine(bool save_screen_shot) {

    // Variables
    cv::Point       p,q;
    double          thick = 2.0; // 1.0;

    // plot line segment features
    for( list<LineFeature*>::iterator ls_it = matched_ls.begin();
         ls_it != matched_ls.end(); ls_it++)
    {
        if( (*ls_it)->inlier )
        {
            // draw optimized line
            Vector4d pt_world;
            Vector3d sP_;
            pt_world << (*ls_it)->sP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            sP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);
            //
            Vector3d eP_;
            pt_world << (*ls_it)->eP, 1;
            pt_world = this->curr_frame->DT.inverse() * pt_world;
            eP_ << (pt_world)(0),
                    (pt_world)(1),
                    (pt_world)(2);

            // draw optimized line
            Vector2d spl_proj1 = cam->projection( sP_ );
            Vector2d epl_proj1 = cam->projection( eP_ );
            p = cv::Point( int(spl_proj1(0)), int(spl_proj1(1)) );
            q = cv::Point( int(epl_proj1(0)), int(epl_proj1(1)) );
            // line( img_l_aux, p, q, Scalar(0,0,255), thick);
            line( canvas_match_frames, p, q, Scalar(255,0,0), thick);
        }
    }
    //

    if (save_screen_shot && n_inliers_ls > 0) {
        //
        //        std::string frame_wid = this->curr_frame->save_path + "/optimize/";
        std::string frame_wid = this->curr_frame->save_path;
        //        frame_wid += std::to_string(curr_frame->frame_idx);
        long time_stamp = curr_frame->time_stamp * pow(10, 9);
        frame_wid += std::to_string(time_stamp);

        frame_wid += ".png";
        imwrite( frame_wid, this->canvas_match_frames );
    }
}


}
