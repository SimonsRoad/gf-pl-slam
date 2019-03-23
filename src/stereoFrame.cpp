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

#include <stereoFrame.h>

namespace StVO{

StereoFrame::StereoFrame(){
    // create detectors
    lsdExtractor_l = line_descriptor::LSDDetectorC::createLSDDetectorC();
    lsdExtractor_r = line_descriptor::LSDDetectorC::createLSDDetectorC();
    lbdExtractor_l = BinaryDescriptor::createBinaryDescriptor();
    lbdExtractor_r = BinaryDescriptor::createBinaryDescriptor();
    //
    orbExtractor_l = new ORB_SLAM2::ORBextractor(Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
                                                 20, 7);
    orbExtractor_r = new ORB_SLAM2::ORBextractor(Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
                                                 20, 7);
    // Scale Level Info
    mnScaleLevels = orbExtractor_l->GetLevels();
    mfScaleFactor = orbExtractor_l->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = orbExtractor_l->GetScaleFactors();
    mvInvScaleFactors = orbExtractor_l->GetInverseScaleFactors();
    mvLevelSigma2 = orbExtractor_l->GetScaleSigmaSquares();
    mvInvLevelSigma2 = orbExtractor_l->GetInverseScaleSigmaSquares();
    //
    //    TH_HIGH = 100;
    //    TH_LOW = 50;
    //    HISTO_LENGTH = 30;
}

StereoFrame::StereoFrame(const Mat & img_l_, const Mat & img_r_ , const int & idx_,
                         PinholeStereoCamera *cam_, const double & time_stamp_) :
    rgbImg_l(img_l_), rgbImg_r(img_r_), frame_idx(idx_), cam(cam_), time_stamp(time_stamp_) {

    //    std::cout << "construct stereo frame w\\o init orb extractor!" << std::endl;

    if( img_l_.channels() == 3 )
        cvtColor(rgbImg_l, gryImg_l, CV_BGR2GRAY);
    else
        gryImg_l = rgbImg_l;
    if( img_r_.channels() == 3 )
        cvtColor(rgbImg_r, gryImg_r, CV_BGR2GRAY);
    else
        gryImg_r = rgbImg_r;

    // create detectors
    lsdExtractor_l = line_descriptor::LSDDetectorC::createLSDDetectorC();
    lsdExtractor_r = line_descriptor::LSDDetectorC::createLSDDetectorC();
    lbdExtractor_l = BinaryDescriptor::createBinaryDescriptor();
    lbdExtractor_r = BinaryDescriptor::createBinaryDescriptor();
    //
    orbExtractor_l = new ORB_SLAM2::ORBextractor(Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
                                                 20, 7);
    orbExtractor_r = new ORB_SLAM2::ORBextractor(Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
                                                 20, 7);
    //    int fast_th_ = Config::orbFastTh();
    //    if( fast_th != 0 )
    //        fast_th_ = fast_th;
    //    orbExtractor = ORB::create( Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
    //                                Config::orbEdgeTh(), 0, Config::orbWtaK(), Config::orbScore(),
    //                                Config::orbPatchSize(), fast_th_ );
    // Scale Level Info
    mnScaleLevels = orbExtractor_l->GetLevels();
    mfScaleFactor = orbExtractor_l->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = orbExtractor_l->GetScaleFactors();
    mvInvScaleFactors = orbExtractor_l->GetInverseScaleFactors();
    mvLevelSigma2 = orbExtractor_l->GetScaleSigmaSquares();
    mvInvLevelSigma2 = orbExtractor_l->GetInverseScaleSigmaSquares();
    //
    //    TH_HIGH = 100;
    //    TH_LOW = 50;
    //    HISTO_LENGTH = 30;


    //    std::cout << "finish stereo frame w\\o init orb extractor!" << std::endl;
}


StereoFrame::StereoFrame(const Mat & img_l_, const Mat & img_r_ , const int & idx_,
                         PinholeStereoCamera *cam_,
                         ORB_SLAM2::ORBextractor * orbExtractor_l_,
                         ORB_SLAM2::ORBextractor * orbExtractor_r_,
                         const double & time_stamp_) :
    rgbImg_l(img_l_), rgbImg_r(img_r_), frame_idx(idx_), cam(cam_), time_stamp(time_stamp_) {

    //    std::cout << "construct stereo frame w\\ init orb extractor!" << std::endl;

    if( img_l_.channels() == 3 )
        cvtColor(rgbImg_l, gryImg_l, CV_BGR2GRAY);
    else
        gryImg_l = rgbImg_l;
    if( img_r_.channels() == 3 )
        cvtColor(rgbImg_r, gryImg_r, CV_BGR2GRAY);
    else
        gryImg_r = rgbImg_r;

    // create detectors
    lsdExtractor_l = line_descriptor::LSDDetectorC::createLSDDetectorC();
    lsdExtractor_r = line_descriptor::LSDDetectorC::createLSDDetectorC();
    lbdExtractor_l = BinaryDescriptor::createBinaryDescriptor();
    lbdExtractor_r = BinaryDescriptor::createBinaryDescriptor();
    //
    orbExtractor_l = orbExtractor_l_;
    orbExtractor_r = orbExtractor_r_;
    // Scale Level Info
    mnScaleLevels = orbExtractor_l->GetLevels();
    mfScaleFactor = orbExtractor_l->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = orbExtractor_l->GetScaleFactors();
    mvInvScaleFactors = orbExtractor_l->GetInverseScaleFactors();
    mvLevelSigma2 = orbExtractor_l->GetScaleSigmaSquares();
    mvInvLevelSigma2 = orbExtractor_l->GetInverseScaleSigmaSquares();
    //
    //    TH_HIGH = 100;
    //    TH_LOW = 50;
    //    HISTO_LENGTH = 30;


    //    std::cout << "finish stereo frame w\\ init orb extractor!" << std::endl;
}




StereoFrame::~StereoFrame(){}

void StereoFrame::extractInitialStereoFeatures( const int fast_th )
{
    // Feature detection and description
    double min_line_length_th = Config::minLineLength() * std::min( cam->getWidth(), cam->getHeight() );
    if( Config::lrInParallel() )
    {
        //        std::thread threadLeft(&StereoFrame::detectFeatures, this,
        //                               min_line_length_th, fast_th, 0);
        //        std::thread threadRight(&StereoFrame::detectFeatures, this,
        //                                min_line_length_th, fast_th, 1);
        //        threadLeft.join();
        //        threadRight.join();
        auto detect_l = std::async(std::launch::async, &StereoFrame::detectFeatures, this,
                                   min_line_length_th, fast_th, 0 );
        auto detect_r = std::async(std::launch::async, &StereoFrame::detectFeatures, this,
                                   min_line_length_th, fast_th, 1 );
        detect_l.wait();
        detect_r.wait();
    }
    else
    {
        detectFeatures(min_line_length_th,fast_th,0);
        detectFeatures(min_line_length_th,fast_th,1);
    }

    // Points stereo matching
    if( Config::hasPoints() && !(points_l.size()==0) && !(points_r.size()==0) )
    {
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );
        vector<vector<DMatch>> pmatches_lr, pmatches_rl;
        Mat pdesc_l_;
        stereo_pt.clear();
        // LR and RL matches
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = std::async(std::launch::async, &StereoFrame::matchPointFeatures, this, bfm, pdesc_l, pdesc_r, ref(pmatches_lr) );
                auto match_r = std::async(std::launch::async, &StereoFrame::matchPointFeatures, this, bfm, pdesc_r, pdesc_l, ref(pmatches_rl) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( pdesc_l, pdesc_r, pmatches_lr, 2);
                bfm->knnMatch( pdesc_r, pdesc_l, pmatches_rl, 2);
            }
        }
        else
            bfm->knnMatch( pdesc_l, pdesc_r, pmatches_lr, 2);
        // sort matches by the distance between the best and second best matches
        double nn12_dist_th  = Config::maxRatio12P();
        // resort according to the queryIdx
        sort( pmatches_lr.begin(), pmatches_lr.end(), sort_descriptor_by_queryIdx() );
        if(Config::bestLRMatches())
            sort( pmatches_rl.begin(), pmatches_rl.end(), sort_descriptor_by_queryIdx() );
        // bucle around pmatches
        int pt_idx = 0;
        for( int i = 0; i < pmatches_lr.size(); i++ )
        {
            int lr_qdx, lr_tdx, rl_tdx;
            lr_qdx = pmatches_lr[i][0].queryIdx;
            lr_tdx = pmatches_lr[i][0].trainIdx;
            if( Config::bestLRMatches() )
            {
                // check if they are mutual best matches
                rl_tdx = pmatches_rl[lr_tdx][0].trainIdx;
            }
            else
                rl_tdx = lr_qdx;
            // check if they are mutual best matches and the minimum distance
            double dist_12 = pmatches_lr[i][0].distance / pmatches_lr[i][1].distance;
            //             if( lr_qdx == rl_tdx  && dist_12 > nn12_dist_th )
            if( lr_qdx == rl_tdx  && dist_12 <= nn12_dist_th )
            {
                // check stereo epipolar constraint
                if( fabsf( points_l[lr_qdx].pt.y-points_r[lr_tdx].pt.y) <= Config::maxDistEpip() )
                {
                    // check minimal disparity
                    double disp_ = points_l[lr_qdx].pt.x - points_r[lr_tdx].pt.x;
                    if( disp_ >= Config::minDisp() ){
                        pdesc_l_.push_back( pdesc_l.row(lr_qdx) );
                        //                        PointFeature* point_;
                        Vector2d pl_; pl_ << points_l[lr_qdx].pt.x, points_l[lr_qdx].pt.y;
                        Vector3d P_;  P_ = cam->backProjection( pl_(0), pl_(1), disp_);
                        stereo_pt.push_back( new PointFeature(pl_,disp_,P_,pt_idx,points_l[lr_qdx].octave) );
                        //
                        points_r[lr_tdx].class_id = pt_idx;
                        image_pt_r.push_back(points_r[lr_tdx]);
                        points_l[lr_qdx].class_id = pt_idx;
                        image_pt_l.push_back(points_l[lr_qdx]);

                        pt_idx++;
                    }
                }
            }
        }
        pdesc_l_.copyTo(pdesc_l);
    }

    // Line segments stereo matching
    if( Config::hasLines() && !lines_l.empty() && !lines_r.empty() )
    {
        stereo_ls.clear();
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );
        vector<vector<DMatch>> lmatches_lr, lmatches_rl;
        Mat ldesc_l_;
        // LR and RL matches
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = std::async(std::launch::async, &StereoFrame::matchLineFeatures, this, bfm, ldesc_l, ldesc_r, ref(lmatches_lr) );
                auto match_r = std::async(std::launch::async, &StereoFrame::matchLineFeatures, this, bfm, ldesc_r, ldesc_l, ref(lmatches_rl) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( ldesc_l,ldesc_r, lmatches_lr, 2);
                bfm->knnMatch( ldesc_r,ldesc_l, lmatches_rl, 2);
            }
        }
        else
            bfm->knnMatch( ldesc_l,ldesc_r, lmatches_lr, 2);
        // sort matches by the distance between the best and second best matches
        double nn_dist_th, nn12_dist_th;
        lineDescriptorMAD(lmatches_lr,nn_dist_th, nn12_dist_th);
        nn12_dist_th  = nn12_dist_th * Config::descThL();
        // bucle around pmatches
        sort( lmatches_lr.begin(), lmatches_lr.end(), sort_descriptor_by_queryIdx() );
        if( Config::bestLRMatches() )
            sort( lmatches_rl.begin(), lmatches_rl.end(), sort_descriptor_by_queryIdx() );
        int n_matches;
        if( Config::bestLRMatches() )
            n_matches = min(lmatches_lr.size(),lmatches_rl.size());
        else
            n_matches = lmatches_lr.size();
        int ls_idx = 0;
        for( int i = 0; i < n_matches; i++ )
        {
            // check if they are mutual best matches ( if bestLRMatches() )
            int lr_qdx = lmatches_lr[i][0].queryIdx;
            int lr_tdx = lmatches_lr[i][0].trainIdx;
            int rl_tdx;
            if( Config::bestLRMatches() )
                rl_tdx = lmatches_rl[lr_tdx][0].trainIdx;
            else
                rl_tdx = lr_qdx;
            // check if they are mutual best matches and the minimum distance
            double dist_12 = lmatches_lr[i][1].distance - lmatches_lr[i][0].distance;
            double length  = lines_r[lr_tdx].lineLength;
            if( lr_qdx == rl_tdx && dist_12 > nn12_dist_th )
            {
                // estimate the disparity of the endpoints
                Vector3d sp_l; sp_l << lines_l[lr_qdx].startPointX, lines_l[lr_qdx].startPointY, 1.0;
                Vector3d ep_l; ep_l << lines_l[lr_qdx].endPointX,   lines_l[lr_qdx].endPointY,   1.0;
                Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
                Vector3d sp_r; sp_r << lines_r[lr_tdx].startPointX, lines_r[lr_tdx].startPointY, 1.0;
                Vector3d ep_r; ep_r << lines_r[lr_tdx].endPointX,   lines_r[lr_tdx].endPointY,   1.0;
                Vector3d le_r; le_r << sp_r.cross(ep_r);
                double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );
                sp_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0) , lines_l[lr_qdx].startPointY ,  1.0;
                ep_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0) , lines_l[lr_qdx].endPointY ,    1.0;
                double disp_s = lines_l[lr_qdx].startPointX - sp_r(0);
                double disp_e = lines_l[lr_qdx].endPointX   - ep_r(0);
                // check minimal disparity
                if( disp_s >= Config::minDisp() && disp_e >= Config::minDisp()
                        && fabsf(le_r(0)) > Config::lineHorizTh()
                        && overlap > Config::stereoOverlapTh() )
                {
                    ldesc_l_.push_back( ldesc_l.row(lr_qdx) );
                    Vector3d sP_; sP_ = cam->backProjection( sp_l(0), sp_l(1), disp_s);
                    Vector3d eP_; eP_ = cam->backProjection( ep_l(0), ep_l(1), disp_e);
                    double angle_l = lines_l[lr_qdx].angle;
                    stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0),sp_l(1)),disp_s,sP_,Vector2d(ep_l(0),ep_l(1)),
                                                         disp_e,eP_,le_l,angle_l,ls_idx, lines_l[lr_qdx].octave) );
                    //
                    image_ls_r.push_back(lines_r[lr_tdx]);
                    image_ls_l.push_back(lines_l[lr_qdx]);
                    //
                    ls_idx++;
                }
            }
        }
        ldesc_l_.copyTo(ldesc_l);
    }

}



void StereoFrame::subPixelStereoRefine_ORBSLAM(const cv::KeyPoint &kpL,
                                               const cv::KeyPoint &kpR,
                                               float & disparity,
                                               float & bestuR) {

    disparity = -1;
    bestuR = kpR.pt.x;

    // coordinates in image pyramid at keypoint scale
    const float uR0 = kpR.pt.x;
    const float scaleFactor = mvInvScaleFactors[kpL.octave];
    const float scaleduL = round(kpL.pt.x*scaleFactor);
    const float scaledvL = round(kpL.pt.y*scaleFactor);
    const float scaleduR0 = round(uR0*scaleFactor);

    // sliding window search
    const int w = 5;
    cv::Mat IL = orbExtractor_r->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
    IL.convertTo(IL,CV_32F);
    IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

    int bestDist = INT_MAX;
    int bestincR = 0;
    const int L = 5;
    vector<float> vDists;
    vDists.resize(2*L+1);

    const float iniu = scaleduR0+L-w;
    const float endu = scaleduR0+L+w+1;
    if(iniu<0 || endu >= orbExtractor_r->mvImagePyramid[kpL.octave].cols)
        return ;

    for(int incR=-L; incR<=+L; incR++)
    {
        cv::Mat IR = orbExtractor_r->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
        IR.convertTo(IR,CV_32F);
        IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

        float dist = cv::norm(IL,IR,cv::NORM_L1);
        if(dist<bestDist)
        {
            bestDist =  dist;
            bestincR = incR;
        }

        vDists[L+incR] = dist;
    }

    if(bestincR==-L || bestincR==L)
        return ;

    // Sub-pixel match (Parabola fitting)
    const float dist1 = vDists[L+bestincR-1];
    const float dist2 = vDists[L+bestincR];
    const float dist3 = vDists[L+bestincR+1];

    const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

    if(deltaR<-1 || deltaR>1)
        return ;

    // Re-scaled coordinate
    bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
    disparity = (kpL.pt.x-bestuR);
}

//
// Update the stereo matching code from brute-force stereo to epipolar guided local search
// TODO
// TODO
// TODO
void StereoFrame::extractStereoFeatures_ORBSLAM( const int fast_th ) {

//    clock_t time_st;
//    //
//    time_st = clock();
    
    // Feature detection and description
    double min_line_length_th = Config::minLineLength() * std::min( cam->getWidth(), cam->getHeight() ) ;
    if( Config::lrInParallel() )
    {
//        std::thread thread_1(&StereoFrame::detectLineFeatures,  this,
//                             this->rgbImg_l, min_line_length_th, 0);
//        std::thread thread_2(&StereoFrame::detectPointFeatures, this,
//                             this->gryImg_l, fast_th, 0);
//        std::thread thread_3(&StereoFrame::detectLineFeatures,  this,
//                             this->rgbImg_r, min_line_length_th, 1);
//        std::thread thread_4(&StereoFrame::detectPointFeatures, this,
//                             this->gryImg_r, fast_th, 1);
//        thread_1.join();
//        thread_2.join();
//        thread_3.join();
//        thread_4.join();
        std::thread threadLeft(&StereoFrame::detectFeatures, this,
                               min_line_length_th, fast_th, 0);
        std::thread threadRight(&StereoFrame::detectFeatures, this,
                                min_line_length_th, fast_th, 1);
        threadLeft.join();
        threadRight.join();
        //        auto detect_l = std::async(std::launch::async, &StereoFrame::detectFeatures, this,
        //                              min_line_length_th, fast_th, 0 );
        //        auto detect_r = std::async(std::launch::async, &StereoFrame::detectFeatures, this,
        //                              min_line_length_th, fast_th, 1 );
        //        detect_l.wait();
        //        detect_r.wait();
    }
    else
    {
        detectFeatures(min_line_length_th,fast_th,0);
        detectFeatures(min_line_length_th,fast_th,1);
    }
//    std::cout << "time cost of feature extraction = " << double(clock() - time_st) / double(CLOCKS_PER_SEC) << std::endl;

    if( Config::hasPoints() && !(points_l.size()==0) && !(points_r.size()==0) ) {

        double startTime = clock();
        const int thOrbDist = 80; // (TH_HIGH+TH_LOW)/2;
        const int nRows = orbExtractor_l->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

        for(int i=0; i<nRows; i++)
            vRowIndices[i].reserve(200);

        const int N = points_l.size(), Nr = points_r.size();

        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

#ifdef DEBUG_PRINT_OUT
        std::cout << "keypoints from left cam " << N << std::endl;
        std::cout << "keypoints from right cam " << Nr << std::endl;
#endif

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = points_r[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f*mvScaleFactors[points_r[iR].octave];
            const int maxr = ceil(kpY+r);
            const int minr = floor(kpY-r);

            for(int yi=minr;yi<=maxr;yi++)
                vRowIndices[yi].push_back(iR);
        }


        // Set limits for search
        //    const float minZ = cam->getB();
        const float minD = 0;
        const float maxD = cam->getFx();
        const float mbf = cam->getFx() * cam->getB();

#ifdef DEBUG_PRINT_OUT
        std::cout << "foculength = " << cam->getFx() << "; baseline = " << cam->getB() << std::endl;
#endif

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        for(int iL=0; iL<N; iL++)
        {
            const cv::KeyPoint &kpL = points_l[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

#ifdef DEBUG_PRINT_OUT
            std::cout << "levelL = " << levelL << "; vL = " << vL << "; uL = " << uL << std::endl;
#endif

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if(vCandidates.empty())
                continue;

            const float minU = uL-maxD;
            const float maxU = uL-minD;

            if(maxU<0)
                continue;

            int bestDist = 100;
            size_t bestIdxR = 0;

            const cv::Mat &dL = pdesc_l.row(iL);

#ifdef DEBUG_PRINT_OUT
            std::cout << "vCandidates.size() = " << vCandidates.size() << std::endl;
#endif

            // Compare descriptor to right keypoints
            for(size_t iC=0; iC<vCandidates.size(); iC++)
            {
                //                std::cout << "iC = " << iC << std::endl;

                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = points_r[iR];

                if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                    continue;

                const float &uR = kpR.pt.x;

                if(uR>=minU && uR<=maxU)
                {
                    const cv::Mat &dR = pdesc_r.row(iR);
                    const int dist = descriptorDistance(dL,dR);

                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

#ifdef DEBUG_PRINT_OUT
            std::cout << "bestDist = " << bestDist << "; thOrbDist = "  << thOrbDist << std::endl;
#endif

            // Subpixel match by correlation
            if(bestDist<thOrbDist)
            {
                const cv::KeyPoint &kpR = points_r[bestIdxR];
                float disparity, bestuR;
                subPixelStereoRefine_ORBSLAM(kpL, kpR, disparity, bestuR);

                if(disparity>=minD && disparity<maxD)
                {
                    if(disparity<=0)
                    {
                        disparity=0.01;
                        bestuR = uL-0.01;
                    }
                    mvDepth[iL] = mbf/disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int,int>(bestDist,iL));
                }
            }

        }

        sort(vDistIdx.begin(),vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = 1.5f*1.4f*median;

        for(int i=vDistIdx.size()-1;i>=0;i--)
        {
            if(vDistIdx[i].first<thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second]=-1;
                mvDepth[vDistIdx[i].second]=-1;
            }
        }

        int pt_idx = 0;
        Mat pdesc_l_;
        stereo_pt.clear();
        for(int i=0; i<vDistIdx.size();i++) {
            if(vDistIdx[i].first >= thDist)
                break;
            //
            int iL = vDistIdx[i].second;
            Vector2d pl_; pl_ << points_l[iL].pt.x, points_l[iL].pt.y;

            float disparity = mbf/mvDepth[iL];
            if(disparity < 0)
                continue ;
            Vector3d P_;  P_ = cam->backProjection( pl_(0), pl_(1), disparity);
            pdesc_l_.push_back( pdesc_l.row(iL) );
            stereo_pt.push_back( new PointFeature(pl_, disparity,
                                                  P_, pt_idx, points_l[iL].octave) );
            ////
            //points_r[lr_tdx].class_id = pt_idx;
            //image_pt_r.push_back(points_r[lr_tdx]);
            //points_l[lr_qdx].class_id = pt_idx;
            //image_pt_l.push_back(points_l[lr_qdx]);

            pt_idx ++;
        }

        pdesc_l_.copyTo(pdesc_l);

        log_.num_pt_stereo = stereo_pt.size();
        log_.time_pt_stereo = double(clock() - startTime) / double(CLOCKS_PER_SEC);
    }

    // Line segments stereo matching
    if( Config::hasLines() && !lines_l.empty() && !lines_r.empty() ) {
        double startTime = clock();
        stereo_ls.clear();
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING2, false );
        vector<vector<DMatch>> lmatches_lr, lmatches_rl;
        Mat ldesc_l_;
        // LR and RL matches
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = std::async(std::launch::async, &StereoFrame::matchLineFeatures, this, bfm, ldesc_l, ldesc_r, ref(lmatches_lr) );
                auto match_r = std::async(std::launch::async, &StereoFrame::matchLineFeatures, this, bfm, ldesc_r, ldesc_l, ref(lmatches_rl) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( ldesc_l,ldesc_r, lmatches_lr, 2);
                bfm->knnMatch( ldesc_r,ldesc_l, lmatches_rl, 2);
            }
        }
        else
            bfm->knnMatch( ldesc_l,ldesc_r, lmatches_lr, 2);
        // sort matches by the distance between the best and second best matches
        double nn_dist_th, nn12_dist_th;
        lineDescriptorMAD(lmatches_lr,nn_dist_th, nn12_dist_th);
        nn12_dist_th  = nn12_dist_th * Config::descThL();
        // bucle around pmatches
        sort( lmatches_lr.begin(), lmatches_lr.end(), sort_descriptor_by_queryIdx() );
        if( Config::bestLRMatches() )
            sort( lmatches_rl.begin(), lmatches_rl.end(), sort_descriptor_by_queryIdx() );
        int n_matches;
        if( Config::bestLRMatches() )
            n_matches = min(lmatches_lr.size(),lmatches_rl.size());
        else
            n_matches = lmatches_lr.size();
        for( int i = 0; i < n_matches; i++ )
        {
            // check if they are mutual best matches ( if bestLRMatches() )
            int lr_qdx = lmatches_lr[i][0].queryIdx;
            int lr_tdx = lmatches_lr[i][0].trainIdx;
            int rl_tdx;
            if( Config::bestLRMatches() )
                rl_tdx = lmatches_rl[lr_tdx][0].trainIdx;
            else
                rl_tdx = lr_qdx;
            // check if they are mutual best matches and the minimum distance
            double dist_12 = lmatches_lr[i][1].distance - lmatches_lr[i][0].distance;
            double length  = lines_r[lr_tdx].lineLength;
            if( lr_qdx == rl_tdx && dist_12 > nn12_dist_th )
            {
                // estimate the disparity of the endpoints
                Vector3d sp_l; sp_l << lines_l[lr_qdx].startPointX, lines_l[lr_qdx].startPointY, 1.0;
                Vector3d ep_l; ep_l << lines_l[lr_qdx].endPointX,   lines_l[lr_qdx].endPointY,   1.0;
                Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
                Vector3d sp_r; sp_r << lines_r[lr_tdx].startPointX, lines_r[lr_tdx].startPointY, 1.0;
                Vector3d ep_r; ep_r << lines_r[lr_tdx].endPointX,   lines_r[lr_tdx].endPointY,   1.0;
                Vector3d le_r; le_r << sp_r.cross(ep_r);
                double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );
                sp_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0) , lines_l[lr_qdx].startPointY ,  1.0;
                ep_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0) , lines_l[lr_qdx].endPointY ,    1.0;
                double disp_s = lines_l[lr_qdx].startPointX - sp_r(0);
                double disp_e = lines_l[lr_qdx].endPointX   - ep_r(0);
                // check minimal disparity
                if( disp_s >= Config::minDisp() && disp_e >= Config::minDisp()
                        // && fabsf(le_r(0)) > Config::lineHorizTh()
                        && fabsf(le_l(0)) > Config::lineHorizTh()
                        && overlap > Config::stereoOverlapTh() )
                {
                    Vector3d sP_; sP_ = cam->backProjection( sp_l(0), sp_l(1), disp_s);
                    Vector3d eP_; eP_ = cam->backProjection( ep_l(0), ep_l(1), disp_e);
                    double angle_l = lines_l[lr_qdx].angle;
                    //----------------- DEBUG: 24/05/2016 ----------------------
                    // estimate the uncertainty of the endpoints
                    double cx = cam->getCx();
                    double cy = cam->getCy();
                    double f  = cam->getFx();
                    // - start point
                    double px_hat = sp_l(0) - cx;
                    double py_hat = sp_l(1) - cy;
                    double disp   = disp_s;
                    double disp2  = disp * disp;
                    Matrix3d covS_an;
                    covS_an(0,0) = disp2+2.f*px_hat*px_hat;
                    covS_an(0,1) = 2.f*px_hat*py_hat;
                    covS_an(0,2) = 2.f*f*px_hat;
                    covS_an(1,1) = disp2+2.f*py_hat*py_hat;
                    covS_an(1,2) = 2.f*f*py_hat;
                    covS_an(2,2) = 2.f*f*f;
                    covS_an(1,0) = covS_an(0,1);
                    covS_an(2,0) = covS_an(0,2);
                    covS_an(2,1) = covS_an(1,2);
                    covS_an << covS_an * cam->getB() * cam->getB() / (disp2*disp2);
                    // - end point
                    px_hat = ep_l(0) - cam->getCx();
                    py_hat = ep_l(1) - cam->getCy();
                    disp   = disp_e;
                    disp2  = disp * disp;
                    Matrix3d covE_an;
                    covE_an(0,0) = disp2+2.f*px_hat*px_hat;
                    covE_an(0,1) = 2.f*px_hat*py_hat;
                    covE_an(0,2) = 2.f*f*px_hat;
                    covE_an(1,1) = disp2+2.f*py_hat*py_hat;
                    covE_an(1,2) = 2.f*f*py_hat;
                    covE_an(2,2) = 2.f*f*f;
                    covE_an(1,0) = covE_an(0,1);
                    covE_an(2,0) = covE_an(0,2);
                    covE_an(2,1) = covE_an(1,2);
                    covE_an << covE_an * cam->getB() * cam->getB() / (disp2*disp2);
                    // - estimate eigenvalues
                    Vector3d S_eigen, E_eigen;
                    SelfAdjointEigenSolver<Matrix3d> eigensolver_s(covS_an);
                    S_eigen = eigensolver_s.eigenvalues();
                    SelfAdjointEigenSolver<Matrix3d> eigensolver_e(covE_an);
                    E_eigen = eigensolver_e.eigenvalues();
                    double max_eig = max( S_eigen(2),E_eigen(2) );
                    // - dbg plot
                    if(max_eig < Config::lineCovTh())
                    {
                        stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0),sp_l(1)),disp_s,sP_,
                                                             Vector2d(ep_l(0),ep_l(1)),disp_e,eP_,
                                                             le_l,angle_l,-1,lines_l[lr_qdx].octave) );
                        image_ls_r.push_back(lines_r[lr_tdx]);
                        image_ls_l.push_back(lines_l[lr_qdx]);
                        ldesc_l_.push_back( ldesc_l.row(lr_qdx) );
                    }
                    //----------------------------------------------------------
                }
            }
        }
        ldesc_l_.copyTo(ldesc_l);
        log_.num_ln_stereo = stereo_ls.size();
        log_.time_ln_stereo = double(clock() - startTime) / double(CLOCKS_PER_SEC);
    }
}

void StereoFrame::extractStereoFeatures_Brute( const int fast_th )
{
    // Feature detection and description
    double min_line_length_th = Config::minLineLength() * std::min( cam->getWidth(), cam->getHeight() ) ;
    if( Config::lrInParallel() )
    {
        std::thread threadLeft(&StereoFrame::detectFeatures, this,
                               min_line_length_th, fast_th, 0);
        std::thread threadRight(&StereoFrame::detectFeatures, this,
                                min_line_length_th, fast_th, 1);
        threadLeft.join();
        threadRight.join();
        //        auto detect_l = std::async(std::launch::async, &StereoFrame::detectFeatures,  this,
        //                              min_line_length_th, fast_th, 0);
        //        auto detect_r = std::async(std::launch::async, &StereoFrame::detectFeatures, this,
        //                              min_line_length_th, fast_th, 1);
        //        detect_l.wait();
        //        detect_r.wait();
    }
    else
    {
        detectFeatures(min_line_length_th,fast_th,0);
        detectFeatures(min_line_length_th,fast_th,1);
    }

    // Points stereo matching
    if( Config::hasPoints() && !(points_l.size()==0) && !(points_r.size()==0) )
    {
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );
        vector<vector<DMatch>> pmatches_lr, pmatches_rl;
        Mat pdesc_l_;
        stereo_pt.clear();
        // LR and RL matches
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = std::async(std::launch::async, &StereoFrame::matchPointFeatures, this, bfm, pdesc_l, pdesc_r, ref(pmatches_lr) );
                auto match_r = std::async(std::launch::async, &StereoFrame::matchPointFeatures, this, bfm, pdesc_r, pdesc_l, ref(pmatches_rl) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( pdesc_l, pdesc_r, pmatches_lr, 2);
                bfm->knnMatch( pdesc_r, pdesc_l, pmatches_rl, 2);
            }
        }
        else
            bfm->knnMatch( pdesc_l, pdesc_r, pmatches_lr, 2);
        // sort matches by the distance between the best and second best matches
        double nn12_dist_th  = Config::maxRatio12P();
        // resort according to the queryIdx
        sort( pmatches_lr.begin(), pmatches_lr.end(), sort_descriptor_by_queryIdx() );
        if(Config::bestLRMatches())
            sort( pmatches_rl.begin(), pmatches_rl.end(), sort_descriptor_by_queryIdx() );

        // bucle around pmatches
        int pt_idx = 0;
        for( int i = 0; i < pmatches_lr.size(); i++ )
        {
            int lr_qdx, lr_tdx, rl_tdx;
            lr_qdx = pmatches_lr[i][0].queryIdx;
            lr_tdx = pmatches_lr[i][0].trainIdx;
            if( Config::bestLRMatches() )
            {
                // check if they are mutual best matches
                rl_tdx = pmatches_rl[lr_tdx][0].trainIdx;
            }
            else
                rl_tdx = lr_qdx;
            // check if they are mutual best matches and the minimum distance
            double dist_12 = pmatches_lr[i][0].distance / pmatches_lr[i][1].distance;
            //            std::cout << dist_12 << std::endl;
            //             if( lr_qdx == rl_tdx  && dist_12 > nn12_dist_th )
            if( lr_qdx == rl_tdx && dist_12 <= nn12_dist_th )
            {
                // check stereo epipolar constraint
                if( fabsf( points_l[lr_qdx].pt.y-points_r[lr_tdx].pt.y) <= Config::maxDistEpip() )
                {
                    // check minimal disparity
                    double disp_ = points_l[lr_qdx].pt.x - points_r[lr_tdx].pt.x;
                    if( disp_ >= Config::minDisp() ){

                        //                        const cv::KeyPoint &kpL = points_l[lr_qdx];
                        //                        const cv::KeyPoint &kpR = points_r[lr_tdx];
                        //                        float disp_sub_, bestuR_;
                        //                        subPixelStereoRefine_ORBSLAM(kpL, kpR, disp_sub_, bestuR_);
                        float disp_sub_ = disp_;

                        if(disp_sub_ >= 0) {
                            //                        PointFeature* point_;
                            Vector2d pl_; pl_ << points_l[lr_qdx].pt.x, points_l[lr_qdx].pt.y;
                            Vector3d P_;  P_ = cam->backProjection( pl_(0), pl_(1), disp_sub_);
                            pdesc_l_.push_back( pdesc_l.row(lr_qdx) );
                            stereo_pt.push_back( new PointFeature(pl_,disp_sub_,P_,pt_idx, points_l[lr_qdx].octave) );
                            //
                            points_r[lr_tdx].class_id = pt_idx;
                            image_pt_r.push_back(points_r[lr_tdx]);
                            points_l[lr_qdx].class_id = pt_idx;
                            image_pt_l.push_back(points_l[lr_qdx]);

                            pt_idx ++;
                        }
                    }
                }
            }
        }
        pdesc_l_.copyTo(pdesc_l);
    }

    // Line segments stereo matching
    if( Config::hasLines() && !lines_l.empty() && !lines_r.empty() )
    {
        stereo_ls.clear();
        BFMatcher* bfm = new BFMatcher( NORM_HAMMING2, false );
        vector<vector<DMatch>> lmatches_lr, lmatches_rl;
        Mat ldesc_l_;
        // LR and RL matches
        if( Config::bestLRMatches() )
        {
            if( Config::lrInParallel() )
            {
                auto match_l = std::async(std::launch::async, &StereoFrame::matchLineFeatures, this, bfm, ldesc_l, ldesc_r, ref(lmatches_lr) );
                auto match_r = std::async(std::launch::async, &StereoFrame::matchLineFeatures, this, bfm, ldesc_r, ldesc_l, ref(lmatches_rl) );
                match_l.wait();
                match_r.wait();
            }
            else
            {
                bfm->knnMatch( ldesc_l,ldesc_r, lmatches_lr, 2);
                bfm->knnMatch( ldesc_r,ldesc_l, lmatches_rl, 2);
            }
        }
        else
            bfm->knnMatch( ldesc_l,ldesc_r, lmatches_lr, 2);
        // sort matches by the distance between the best and second best matches
        double nn_dist_th, nn12_dist_th;
        lineDescriptorMAD(lmatches_lr,nn_dist_th, nn12_dist_th);
        nn12_dist_th  = nn12_dist_th * Config::descThL();
        // bucle around pmatches
        sort( lmatches_lr.begin(), lmatches_lr.end(), sort_descriptor_by_queryIdx() );
        if( Config::bestLRMatches() )
            sort( lmatches_rl.begin(), lmatches_rl.end(), sort_descriptor_by_queryIdx() );
        int n_matches;
        if( Config::bestLRMatches() )
            n_matches = min(lmatches_lr.size(),lmatches_rl.size());
        else
            n_matches = lmatches_lr.size();
        for( int i = 0; i < n_matches; i++ )
        {
            // check if they are mutual best matches ( if bestLRMatches() )
            int lr_qdx = lmatches_lr[i][0].queryIdx;
            int lr_tdx = lmatches_lr[i][0].trainIdx;
            int rl_tdx;
            if( Config::bestLRMatches() )
                rl_tdx = lmatches_rl[lr_tdx][0].trainIdx;
            else
                rl_tdx = lr_qdx;
            // check if they are mutual best matches and the minimum distance
            double dist_12 = lmatches_lr[i][1].distance - lmatches_lr[i][0].distance;
            double length  = lines_r[lr_tdx].lineLength;
            if( lr_qdx == rl_tdx && dist_12 > nn12_dist_th )
            {
                // estimate the disparity of the endpoints
                Vector3d sp_l; sp_l << lines_l[lr_qdx].startPointX, lines_l[lr_qdx].startPointY, 1.0;
                Vector3d ep_l; ep_l << lines_l[lr_qdx].endPointX,   lines_l[lr_qdx].endPointY,   1.0;
                Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
                Vector3d sp_r; sp_r << lines_r[lr_tdx].startPointX, lines_r[lr_tdx].startPointY, 1.0;
                Vector3d ep_r; ep_r << lines_r[lr_tdx].endPointX,   lines_r[lr_tdx].endPointY,   1.0;
                Vector3d le_r; le_r << sp_r.cross(ep_r);
                double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );
                sp_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0) , lines_l[lr_qdx].startPointY ,  1.0;
                ep_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0) , lines_l[lr_qdx].endPointY ,    1.0;
                double disp_s = lines_l[lr_qdx].startPointX - sp_r(0);
                double disp_e = lines_l[lr_qdx].endPointX   - ep_r(0);
                // check minimal disparity
                if( disp_s >= Config::minDisp() && disp_e >= Config::minDisp()
                        // && fabsf(le_r(0)) > Config::lineHorizTh()
                        && fabsf(le_l(0)) > Config::lineHorizTh()
                        && overlap > Config::stereoOverlapTh() )
                {
                    Vector3d sP_; sP_ = cam->backProjection( sp_l(0), sp_l(1), disp_s);
                    Vector3d eP_; eP_ = cam->backProjection( ep_l(0), ep_l(1), disp_e);
                    double angle_l = lines_l[lr_qdx].angle;
                    //----------------- DEBUG: 24/05/2016 ----------------------
                    // estimate the uncertainty of the endpoints
                    double cx = cam->getCx();
                    double cy = cam->getCy();
                    double f  = cam->getFx();
                    // - start point
                    double px_hat = sp_l(0) - cx;
                    double py_hat = sp_l(1) - cy;
                    double disp   = disp_s;
                    double disp2  = disp * disp;
                    Matrix3d covS_an;
                    covS_an(0,0) = disp2+2.f*px_hat*px_hat;
                    covS_an(0,1) = 2.f*px_hat*py_hat;
                    covS_an(0,2) = 2.f*f*px_hat;
                    covS_an(1,1) = disp2+2.f*py_hat*py_hat;
                    covS_an(1,2) = 2.f*f*py_hat;
                    covS_an(2,2) = 2.f*f*f;
                    covS_an(1,0) = covS_an(0,1);
                    covS_an(2,0) = covS_an(0,2);
                    covS_an(2,1) = covS_an(1,2);
                    covS_an << covS_an * cam->getB() * cam->getB() / (disp2*disp2);
                    // - end point
                    px_hat = ep_l(0) - cam->getCx();
                    py_hat = ep_l(1) - cam->getCy();
                    disp   = disp_e;
                    disp2  = disp * disp;
                    Matrix3d covE_an;
                    covE_an(0,0) = disp2+2.f*px_hat*px_hat;
                    covE_an(0,1) = 2.f*px_hat*py_hat;
                    covE_an(0,2) = 2.f*f*px_hat;
                    covE_an(1,1) = disp2+2.f*py_hat*py_hat;
                    covE_an(1,2) = 2.f*f*py_hat;
                    covE_an(2,2) = 2.f*f*f;
                    covE_an(1,0) = covE_an(0,1);
                    covE_an(2,0) = covE_an(0,2);
                    covE_an(2,1) = covE_an(1,2);
                    covE_an << covE_an * cam->getB() * cam->getB() / (disp2*disp2);
                    // - estimate eigenvalues
                    Vector3d S_eigen, E_eigen;
                    SelfAdjointEigenSolver<Matrix3d> eigensolver_s(covS_an);
                    S_eigen = eigensolver_s.eigenvalues();
                    SelfAdjointEigenSolver<Matrix3d> eigensolver_e(covE_an);
                    E_eigen = eigensolver_e.eigenvalues();
                    double max_eig = max( S_eigen(2),E_eigen(2) );
                    // - dbg plot
                    if(max_eig < Config::lineCovTh())
                    {
                        stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0),sp_l(1)),disp_s,sP_,
                                                             Vector2d(ep_l(0),ep_l(1)),disp_e,eP_,
                                                             le_l,angle_l,-1,lines_l[lr_qdx].octave) );
                        image_ls_r.push_back(lines_r[lr_tdx]);
                        image_ls_l.push_back(lines_l[lr_qdx]);
                        ldesc_l_.push_back( ldesc_l.row(lr_qdx) );
                    }
                    //----------------------------------------------------------
                }
            }
        }
        ldesc_l_.copyTo(ldesc_l);
    }

}


void StereoFrame::detectFeatures(double min_line_length, int fast_th, int isRight) {

    if( Config::plInParallel() && Config::hasPoints() && Config::hasLines() )
    {
        if (isRight == 0) {
            std::thread threadLine(&StereoFrame::detectLineFeatures,  this,
                                   this->rgbImg_l, min_line_length, 0);
            std::thread threadPoint(&StereoFrame::detectPointFeatures, this,
                                    this->gryImg_l, fast_th, 0);
            threadLine.join();
            threadPoint.join();
            //            auto detect_l = std::async(std::launch::async, &StereoFrame::detectLineFeatures,  this,
            //                                  this->rgbImg_l, min_line_length, 0 );
            //            auto detect_p = std::async(std::launch::async, &StereoFrame::detectPointFeatures, this,
            //                                  this->gryImg_l, fast_th, 0 );
            //            detect_l.wait();
            //            detect_p.wait();
        }
        else {
            std::thread threadLine(&StereoFrame::detectLineFeatures,  this,
                                   this->rgbImg_r, min_line_length, 1);
            std::thread threadPoint(&StereoFrame::detectPointFeatures, this,
                                    this->gryImg_r, fast_th, 1);
            threadLine.join();
            threadPoint.join();
            //            auto detect_l = std::async(std::launch::async, &StereoFrame::detectLineFeatures,  this,
            //                                  this->rgbImg_r, min_line_length, 1 );
            //            auto detect_p = std::async(std::launch::async, &StereoFrame::detectPointFeatures, this,
            //                                  this->gryImg_r, fast_th, 1 );
            //            detect_l.wait();
            //            detect_p.wait();
        }
    }
    else
    {
        // Detect point features
        if( Config::hasPoints() )
        {
            //            int fast_th_ = Config::orbFastTh();
            //            if( fast_th != 0 )
            //                fast_th_ = fast_th;
            //            Ptr<ORB> orb = ORB::create( Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
            //                                        Config::orbEdgeTh(), 0, Config::orbWtaK(), Config::orbScore(),
            //                                        Config::orbPatchSize(), fast_th_ );
            //            orbExtractor->detectAndCompute( img, Mat(), points, pdesc, false);
            if (isRight == 0) {
                //                orbExtractor_l->SetFASTThres(fast_th);
                (*orbExtractor_l)(this->gryImg_l, cv::Mat(), points_l, pdesc_l);
            }
            else {
                //                orbExtractor_r->SetFASTThres(fast_th);
                (*orbExtractor_r)(this->gryImg_r, cv::Mat(), points_r, pdesc_r);
            }
        }
        // Detect line features
        //        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
        if( Config::hasLines() )
        {
            //            Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
            // lsd parameters
            line_descriptor::LSDDetectorC::LSDOptions opts;
            opts.refine       = Config::lsdRefine();
            opts.scale        = Config::lsdScale();
            opts.sigma_scale  = Config::lsdSigmaScale();
            opts.quant        = Config::lsdQuant();
            opts.ang_th       = Config::lsdAngTh();
            opts.log_eps      = Config::lsdLogEps();
            opts.density_th   = Config::lsdDensityTh();
            opts.n_bins       = Config::lsdNBins();
            opts.min_length   = min_line_length;
            //
            if (isRight == 0) {
                lsdExtractor_l->detect( this->rgbImg_l, lines_l, Config::lsdScale(), Config::lsdOctaveNum(), opts);
                // filter lines
                if( lines_l.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
                {
                    // sort lines by their response
                    sort( lines_l.begin(), lines_l.end(), sort_lines_by_response() );
                    //sort( lines.begin(), lines.end(), sort_lines_by_length() );
                    lines_l.resize(Config::lsdNFeatures());
                    // reassign index
                    for( int i = 0; i < Config::lsdNFeatures(); i++  )
                        lines_l[i].class_id = i;
                }
                lbdExtractor_l->compute( this->rgbImg_l, lines_l, ldesc_l);
            }
            else {
                lsdExtractor_r->detect( this->rgbImg_r, lines_r, Config::lsdScale(), Config::lsdOctaveNum(), opts);
                // filter lines
                if( lines_r.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
                {
                    // sort lines by their response
                    sort( lines_r.begin(), lines_r.end(), sort_lines_by_response() );
                    //sort( lines.begin(), lines.end(), sort_lines_by_length() );
                    lines_r.resize(Config::lsdNFeatures());
                    // reassign index
                    for( int i = 0; i < Config::lsdNFeatures(); i++  )
                        lines_r[i].class_id = i;
                }
                //
                lbdExtractor_r->compute( this->rgbImg_r, lines_r, ldesc_r);
            }
        }
    }
}

void StereoFrame::detectPointFeatures(const Mat & img, int fast_th, int isRight ) {

    // Detect point features
    if( Config::hasPoints() )
    {
        double startTime = clock();
        //        int fast_th_ = Config::orbFastTh();
        //        if( fast_th != 0 )
        //            fast_th_ = fast_th;
        //        Ptr<ORB> orb = ORB::create( Config::orbNFeatures(), Config::orbScaleFactor(), Config::orbNLevels(),
        //                                    Config::orbEdgeTh(), 0, Config::orbWtaK(), Config::orbScore(),
        //                                    Config::orbPatchSize(), fast_th_ );
        //        orbExtractor->detectAndCompute( img, Mat(), points, pdesc, false);
        //        std::cout << "calling orb extractor" << std::endl;
        if (isRight == 0) {
            //            orbExtractor_l->SetFASTThres(fast_th);
            (*orbExtractor_l)(img, cv::Mat(), points_l, pdesc_l);
        }
        else {
            //            orbExtractor_r->SetFASTThres(fast_th);
            (*orbExtractor_r)(img, cv::Mat(), points_r, pdesc_r);
        }
        log_.num_pt_detect = points_l.size();
        log_.time_pt_extract = double(clock() - startTime) / double(CLOCKS_PER_SEC);
#ifdef TIMECOST_VERBOSE
        std::cout << "func detectPointFeatures: time cost of ORB = " << log_.time_pt_extract << std::endl;
#endif
    }
}

void StereoFrame::detectLineFeatures(const Mat & img, double min_line_length, int isRight ) {
    // Detect line features
    //    Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
    if( Config::hasLines() )
    {
        double startTime = clock();
        //        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
        // lsd parameters
        line_descriptor::LSDDetectorC::LSDOptions opts;
        opts.refine       = Config::lsdRefine();
        opts.scale        = Config::lsdScale();
        opts.sigma_scale  = Config::lsdSigmaScale();
        opts.quant        = Config::lsdQuant();
        opts.ang_th       = Config::lsdAngTh();
        opts.log_eps      = Config::lsdLogEps();
        opts.density_th   = Config::lsdDensityTh();
        opts.n_bins       = Config::lsdNBins();
        opts.min_length   = min_line_length;
        //
        if (isRight == 0) {
            lsdExtractor_l->detect( img, lines_l, Config::lsdScale(), Config::lsdOctaveNum(), opts);
            // filter lines
            if( lines_l.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
            {
                // sort lines by their response
                sort( lines_l.begin(), lines_l.end(), sort_lines_by_response() );
                //sort( lines.begin(), lines.end(), sort_lines_by_length() );
                lines_l.resize(Config::lsdNFeatures());
                // reassign index
                for( int i = 0; i < Config::lsdNFeatures(); i++  )
                    lines_l[i].class_id = i;
            }
            log_.num_ln_detect = lines_l.size();
            log_.time_ln_detect = double(clock() - startTime) / double(CLOCKS_PER_SEC);
#ifdef TIMECOST_VERBOSE
        std::cout << "func detectLineFeatures: time cost of LSD = " << log_.time_ln_detect << std::endl;
#endif
            //
            startTime = clock();
            lbdExtractor_l->compute( img, lines_l, ldesc_l);
            log_.time_ln_descri = double(clock() - startTime) / double(CLOCKS_PER_SEC);
#ifdef TIMECOST_VERBOSE
        std::cout << "func detectLineFeatures: time cost of LBD = " << log_.time_ln_descri << std::endl;
#endif
        }
        else {
            lsdExtractor_r->detect( img, lines_r, Config::lsdScale(), Config::lsdOctaveNum(), opts);
            // filter lines
            if( lines_r.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
            {
                // sort lines by their response
                sort( lines_r.begin(), lines_r.end(), sort_lines_by_response() );
                //sort( lines.begin(), lines.end(), sort_lines_by_length() );
                lines_r.resize(Config::lsdNFeatures());
                // reassign index
                for( int i = 0; i < Config::lsdNFeatures(); i++  )
                    lines_r[i].class_id = i;
            }
            log_.num_ln_detect = lines_r.size();
            log_.time_ln_detect = double(clock() - startTime) / double(CLOCKS_PER_SEC);
#ifdef TIMECOST_VERBOSE
        std::cout << "func detectLineFeatures: time cost of LSD = " << log_.time_ln_detect << std::endl;
#endif
            //
            startTime = clock();
            lbdExtractor_r->compute( img, lines_r, ldesc_r);
            log_.time_ln_descri = double(clock() - startTime) / double(CLOCKS_PER_SEC);
#ifdef TIMECOST_VERBOSE
        std::cout << "func detectLineFeatures: time cost of LBD = " << log_.time_ln_descri << std::endl;
#endif
        }
    }
}

void StereoFrame::matchPointFeatures(BFMatcher* bfm,
                                     const Mat & pdesc_1, const Mat & pdesc_2,
                                     vector<vector<DMatch>> &pmatches_12  )
{
    bfm->knnMatch( pdesc_1, pdesc_2, pmatches_12, 2);
}

void StereoFrame::matchLineFeatures(BFMatcher* bfm,
                                    const Mat & ldesc_1, const Mat & ldesc_2,
                                    vector<vector<DMatch>> &lmatches_12  )
{
    bfm->knnMatch( ldesc_1, ldesc_2, lmatches_12, 2);
}

void StereoFrame::matchPointFeatures_radius(BFMatcher* bfm,
                                            const Mat & pdesc_1,
                                            const Mat & pdesc_2,
                                            vector<vector<DMatch>> &pmatches_12  )
{
    // bfm->knnMatch( pdesc_1, pdesc_2, pmatches_12, 6);
    bfm->radiusMatch( pdesc_1, pdesc_2, pmatches_12, Config::pointMatchRadius() );
}

void StereoFrame::matchLineFeatures_radius(BFMatcher* bfm,
                                           const Mat & ldesc_1, const Mat & ldesc_2,
                                           vector<vector<DMatch>> &lmatches_12  )
{
    bfm->radiusMatch( ldesc_1, ldesc_2, lmatches_12, Config::lineMatchRadius() );
}

void StereoFrame::pointDescriptorMAD( const vector<vector<DMatch>> matches,
                                      double &nn_mad, double &nn12_mad )
{

    vector<vector<DMatch>> matches_nn, matches_12;
    matches_nn = matches;
    matches_12 = matches;

    // estimate the NN's distance standard deviation
    double nn_dist_median;
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist() );
    nn_mad = matches_nn[int(matches_nn.size()/2)][0].distance;
    for( int j = 0; j < matches_nn.size(); j++)
        matches_nn[j][0].distance = fabsf( matches_nn[j][0].distance - nn_dist_median );
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist() );
    nn_mad = 1.4826 * matches_nn[int(matches_nn.size()/2)][0].distance;

    // estimate the NN's 12 distance standard deviation
    double nn12_dist_median;
    sort( matches_12.begin(), matches_12.end(), compare_descriptor_by_NN12_ratio() );
    nn_dist_median = matches_12[int(matches_12.size()/2)][0].distance / matches_12[int(matches_12.size()/2)][1].distance;
    for( int j = 0; j < matches_12.size(); j++)
        matches_12[j][0].distance = fabsf( matches_12[j][0].distance / matches_12[j][1].distance - nn_dist_median );
    sort( matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist() );
    nn12_mad =  1.4826 * matches_12[int(matches_12.size()/2)][0].distance;

}

void StereoFrame::lineDescriptorMAD( const vector<vector<DMatch>> matches,
                                     double &nn_mad, double &nn12_mad )
{

    vector<vector<DMatch>> matches_nn, matches_12;
    matches_nn = matches;
    matches_12 = matches;

    // estimate the NN's distance standard deviation
    double nn_dist_median;
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist() );
    nn_mad = matches_nn[int(matches_nn.size()/2)][0].distance;
    for( int j = 0; j < matches_nn.size(); j++)
        matches_nn[j][0].distance = fabsf( matches_nn[j][0].distance - nn_dist_median );
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist() );
    nn_mad = 1.4826 * matches_nn[int(matches_nn.size()/2)][0].distance;

    // estimate the NN's 12 distance standard deviation
    //    double nn12_dist_median;
    sort( matches_12.begin(), matches_12.end(), compare_descriptor_by_NN12_dist() );
    nn12_mad = matches_12[int(matches_12.size()/2)][1].distance - matches_12[int(matches_12.size()/2)][0].distance;
    for( int j = 0; j < matches_12.size(); j++)
        matches_12[j][0].distance = fabsf( matches_12[j][1].distance - matches_12[j][0].distance - nn_dist_median );
    sort( matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist() );
    nn12_mad =  1.4826 * matches_12[int(matches_12.size()/2)][0].distance;

}

void StereoFrame::pointDescriptorBudgetThres( const vector<vector<DMatch>> matches,
                                              double &thres_budget )
{
    vector<vector<DMatch>> matches_nn;
    matches_nn = matches;

    // estimate the NN's distance standard deviation
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist() );
    size_t budget_index = min( size_t(Config::maxPointMatchNum()), matches_nn.size() ) - 1;
    //    std::cout << "func pointDescriptorBudgetThres: " << budget_index << "; " << matches_nn.size();
    thres_budget = matches_nn[budget_index][0].distance;

}

void StereoFrame::lineDescriptorBudgetThres( const vector<vector<DMatch>> matches,
                                             double &thres_budget )
{
    vector<vector<DMatch>> matches_nn;
    matches_nn = matches;

    // estimate the NN's distance standard deviation
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist() );
    size_t budget_index = min( size_t(Config::maxLineMatchNum()), matches_nn.size() ) - 1;
    //    std::cout << "func lineDescriptorBudgetThres: " << budget_index << "; " << matches_nn.size();
    thres_budget = matches_nn[budget_index][0].distance;

}

double StereoFrame::lineSegmentOverlapStereo( double spl_obs, double epl_obs, double spl_proj, double epl_proj  )
{

    double sln    = min(spl_obs,  epl_obs);
    double eln    = max(spl_obs,  epl_obs);
    double spn    = min(spl_proj, epl_proj);
    double epn    = max(spl_proj, epl_proj);

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

//
//
void StereoFrame::getJacob2D_3D(const double &u, const double &v, const double &d,
                                Matrix3d & jacMat) {
    //
    double b = this->cam->getB();
    double d_2  = d * d;
    //
    jacMat(0, 0) = b/d;
    jacMat(1, 0) = 0.0f;
    jacMat(2, 0) = 0.0f;
    //
    jacMat(0, 1) = 0.0f;
    jacMat(1, 1) = b/d;
    jacMat(2, 1) = 0.0f;
    //
    jacMat(0, 2) = -(u - this->cam->getCx()) * b / d_2;
    jacMat(1, 2) = -(v - this->cam->getCy()) * b / d_2;
    jacMat(2, 2) = -this->cam->getFx() * b / d_2;
}

void StereoFrame::getJacob3D_2D(const double &px, const double &py, const double &pz,
                                Matrix3d & jacMat) {

    //
    double f = this->cam->getFx();
    double b = this->cam->getB();
    double pz_2  = pz * pz;
    //
    jacMat(0, 0) = f/pz;
    jacMat(1, 0) = 0.0f;
    jacMat(2, 0) = 0.0f;
    //
    jacMat(0, 1) = 0.0f;
    jacMat(1, 1) = f/pz;
    jacMat(2, 1) = 0.0f;
    //
    jacMat(0, 2) = - f * px / pz_2;
    jacMat(1, 2) = - f * py / pz_2;
    jacMat(2, 2) = - f * b / pz_2;

    //
    // NOTE (when integrated with ML codebase ONLY)
    // instead of computing Jacobian wrt the image frame, we are doing it on normailzed camera frame,
    // which is identical with the code used for line residual & pose optimization
    //
    //    double pz_2  = pz * pz;
    //    //
    //    jacMat(0, 0) = 1.0f/pz;
    //    jacMat(1, 0) = 0.0f;
    //    jacMat(2, 0) = 0.0f;
    //    //
    //    jacMat(0, 1) = 0.0f;
    //    jacMat(1, 1) = 1.0f/pz;
    //    jacMat(2, 1) = 0.0f;
    //    //
    //    jacMat(0, 2) = - px / pz_2;
    //    jacMat(1, 2) = - py / pz_2;
    //    jacMat(2, 2) = 0.0f;
}

void StereoFrame::getCovMat2D_3D(const double &u, const double &v, const double &u_std,
                                 const double &d, const double &d_std, Matrix3d & covMat) {
    //
    Matrix3d cov2D;
    cov2D(0, 0) = pow( u_std, 2 );
    cov2D(1, 1) = pow( u_std, 2 );
    cov2D(2, 2) = pow( d_std, 2 );
    //
    Matrix3d jacMat;
    getJacob2D_3D(u, v, d, jacMat);
    //
    covMat = jacMat * cov2D * jacMat.transpose();
}

void StereoFrame::estimateStereoUncertainty() {
    //
    if (stereo_ls.size() == 0)
        return ;
    //
    for (int i=0; i<stereo_ls.size(); ++i) {
        //
        double spl_std = 1.0;  // 2.0; //
        double epl_std = 1.0;  // 2.0; //
        //        double spl_std = Config::ratioDispSTD() / 0.15, epl_std = Config::ratioDispSTD() / 0.15;
        //
        // TODO
        // fit the coefficient with synthetic data
        double sdisp_std, edisp_std;
        if (fabs(stereo_ls[i]->le[0]) > 0.15) {
            sdisp_std = Config::ratioDispSTD() * stereo_ls[i]->sdisp;
            edisp_std = Config::ratioDispSTD() * stereo_ls[i]->edisp;
            //            sdisp_cov = this->cam->getFx() * this->cam->getB() / stereo_ls[i]->sP(2) * 0.2;
            //            edisp_cov = this->cam->getFx() * this->cam->getB() / stereo_ls[i]->eP(2) * 0.2;
        }
        else {
            sdisp_std = Config::ratioDispSTDHor() * stereo_ls[i]->sdisp;
            edisp_std = Config::ratioDispSTDHor() * stereo_ls[i]->edisp;
        }


        // back project the 2d cov to 3d cov
        getCovMat2D_3D(stereo_ls[i]->spl[0], stereo_ls[i]->spl[1], spl_std,
                stereo_ls[i]->sdisp, sdisp_std, stereo_ls[i]->covSpt3D);
        getCovMat2D_3D(stereo_ls[i]->epl[0], stereo_ls[i]->epl[1], epl_std,
                stereo_ls[i]->edisp, edisp_std, stereo_ls[i]->covEpt3D);

        //                // debug
        //                std::cout << stereo_ls[i]->covSpt3D << std::endl;
        //                std::cout << stereo_ls[i]->covEpt3D << std::endl;
    }
}

void StereoFrame::getLineCoeff(const Vector2d lsp,
                               const Vector2d lep,
                               Vector3d & coeff) {
    //
    Vector3d sp_;
    sp_ << lsp(0), lsp(1), 1.0;
    Vector3d ep_;
    ep_ << lep(0), lep(1), 1.0;
    //
    coeff << sp_.cross(ep_);
    coeff = coeff / sqrt( coeff(0)*coeff(0) + coeff(1)*coeff(1) );
    //    line_seg.coeff[2] = -(le_(0) * line_seg.startPointX + le_(1) * line_seg.startPointY);
    //    std::cout << le_(2) << "; " << line_seg.coeff[2] << std::endl;
}

bool StereoFrame::isOutOfFrame(const int col, const int row) {
    if (col < 0 || col >= this->gryImg_l.cols)
        return true;
    if (row < 0 || row >= this->gryImg_l.rows)
        return true;
    return false;
}

double StereoFrame::pointToLineDist(const Vector3d l_coe,
                                    const Vector2d pt) {
    Vector3d pt_;
    pt_ << pt(0), pt(1), 1.0;
    //
    return fabs( l_coe.dot(pt_) );
}

//bool StereoFrame::withinPointRadius(const Vector2d p1,
//                                    const Vector2d p2,
//                                    const double rng_included) {
//    return (p1 - p2).norm() <= rng_included;
//}

bool StereoFrame::withinLineSegment(const Vector2d lsp,
                                    const Vector2d lep,
                                    const Vector2d pt,
                                    const double rng_included) {
    // find the projection of query point to line segment
    Vector2d line_dir = (lep - lsp);
    double line_length = line_dir.norm();
    line_dir /= line_length;
    //
    double len_proj = (pt - lep).dot(line_dir);
    if (len_proj < -rng_included || len_proj > line_length + rng_included)
        return false;
    else
        return true;
}

bool StereoFrame::withinPointWindow(const Vector2d pt_proj,
                                    const double u,
                                    const double v,
                                    const double rng_included) {
    //
    if ( fabs( pt_proj(0) - u ) <= rng_included && fabs( pt_proj(1) - v ) <= rng_included )
        return true;
    else
        return false;
}

void StereoFrame::projectPrev3DPoint(const Matrix4d & prev_Tfw,
                                     const PointFeature * point_prev,
                                     Vector2d & point_proj_l,
                                     Vector2d & point_proj_r) {
    //
    Vector4d pt_world;
    Vector3d pt_cam;
    // start point projection
    pt_world << point_prev->P, 1;
    pt_world = prev_Tfw * pt_world;
    pt_world = this->Tfw.inverse() * pt_world;
    //
    pt_cam << (pt_world)(0),
            (pt_world)(1),
            (pt_world)(2);
    point_proj_l = cam->projection(pt_cam);
    //
    double disp = cam->getDisparity(pt_cam(2));
    point_proj_r = point_proj_l;
    point_proj_r(0) += disp;
}

void StereoFrame::projectPrev3DLine(const Matrix4d & prev_Tfw,
                                    const LineFeature * line_seg,
                                    Vector2d & line_proj_sp,
                                    Vector2d & line_proj_ep) {
    //
    Vector4d pt_world;
    Vector3d pt_cam;
    // start point projection
    pt_world << line_seg->sP, 1;
    pt_world = prev_Tfw * pt_world;
    pt_world = this->Tfw.inverse() * pt_world;
    //
    pt_cam << (pt_world)(0),
            (pt_world)(1),
            (pt_world)(2);
    line_proj_sp = cam->projection(pt_cam);

    // end point projection
    pt_world << line_seg->eP, 1;
    pt_world = prev_Tfw * pt_world;
    pt_world = this->Tfw.inverse() * pt_world;
    //
    pt_cam << (pt_world)(0),
            (pt_world)(1),
            (pt_world)(2);
    line_proj_ep = cam->projection(pt_cam);

}

//
//
void StereoFrame::plotStereoFrameLeft( bool save_screen_shot )
{
    // create new image to modify it
    cv::Mat img_l_aux;
    this->rgbImg_l.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);

    // Variables
    unsigned int    r = 0, g, b = 0;
    cv::Point2f         p,q;
    double          thick = 1.5;
    int             k = 0, radius  = 3;

    // plot point features
    for( vector<PointFeature*>::iterator pt_it = stereo_pt.begin(); pt_it != stereo_pt.end(); pt_it++)
    {
        if( (*pt_it)->inlier )
        {
            g = 200;
            p = cv::Point( int((*pt_it)->pl(0)), int((*pt_it)->pl(1)) );
            circle( img_l_aux, p, radius, Scalar(b,g,r), thick);
        }
    }

    // plot line segment features
    for( vector<LineFeature*>::iterator ls_it = stereo_ls.begin(); ls_it != stereo_ls.end(); ls_it++)
    {
        if( (*ls_it)->inlier )
        {
            g = 200;
            p = cv::Point( int((*ls_it)->spl(0)), int((*ls_it)->spl(1)) );
            q = cv::Point( int((*ls_it)->epl(0)), int((*ls_it)->epl(1)) );
            line( img_l_aux, p, q, Scalar(b,g,r), thick);
        }
    }

    this->stereo_frame_match = img_l_aux;

    //
    if (save_screen_shot) {
        std::string frame_wid = save_path + "/stereo/";
        frame_wid += std::to_string(this->frame_idx);
        frame_wid += ".png";
        imwrite( frame_wid, this->stereo_frame_match );
    }

}

void StereoFrame::plotStereoFrameBoth( bool save_screen_shot )
{
    // create new image to modify it
    cv::Mat img_l_aux,img_l_aux1,img_l_aux2;
    //cv::Mat img_l_aux;
    this->rgbImg_l.copyTo( img_l_aux2 );
    this->rgbImg_r.copyTo( img_l_aux1 );
    if( img_l_aux1.channels() == 1 )
        cvtColor(img_l_aux1, img_l_aux1, CV_GRAY2BGR);
    if( img_l_aux2.channels() == 1 )
        cvtColor(img_l_aux2, img_l_aux2, CV_GRAY2BGR);
    //
    assert(image_ls_l.size() == image_ls_r.size());
    cv::RNG rand_gen( 0xFFFFFFFF );
    for (size_t i=0; i<image_ls_l.size(); ++i) {
        //        std::cout << i << " " << std::endl;
        int icolor = (unsigned) rand_gen;
        const Scalar color_rnd = Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
        //
        vector<KeyLine> kLine_tmp_l = { image_ls_l[i] };
        drawKeylines(img_l_aux2, kLine_tmp_l, img_l_aux2,
                     cv::Scalar(0,0,0),
                     DrawLinesMatchesFlags::DEFAULT );
        //
        vector<KeyLine> kLine_tmp_r = { image_ls_r[i] };
        drawKeylines(img_l_aux1, kLine_tmp_r, img_l_aux1,
                     cv::Scalar(0,0,0),
                     DrawLinesMatchesFlags::DEFAULT );
        //
        cv::putText(img_l_aux2,
                    std::to_string(image_ls_l[i].class_id),
                    cv::Point(image_ls_l[i].startPointX,
                              image_ls_l[i].startPointY), // Coordinates
                    cv::FONT_HERSHEY_PLAIN, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    color_rnd, // Color
                    1); // Thickness
        //
        cv::putText(img_l_aux1,
                    std::to_string(image_ls_r[i].class_id),
                    cv::Point(image_ls_r[i].startPointX,
                              image_ls_r[i].startPointY), // Coordinates
                    cv::FONT_HERSHEY_PLAIN, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    color_rnd, // Color
                    1); // Thickness

        stereo_ls[i]->color = color_rnd;
    }

    assert(image_pt_l.size() == image_pt_r.size());
    for (size_t i=0; i<image_pt_l.size(); ++i) {
        int icolor = (unsigned) rand_gen;
        const Scalar color_rnd = Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
        //
        circle( img_l_aux2, image_pt_l[i].pt, 3.0, cv::Scalar(0,0,0), 1.5);
        cv::putText(img_l_aux2,
                    std::to_string(image_pt_l[i].class_id),
                    image_pt_l[i].pt, // Coordinates
                    cv::FONT_HERSHEY_PLAIN, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    color_rnd, // Color
                    1); // Thickness
        //
        circle( img_l_aux1, image_pt_r[i].pt, 3.0, cv::Scalar(0,0,0), 1.5);
        cv::putText(img_l_aux1,
                    std::to_string(image_pt_r[i].class_id),
                    image_pt_r[i].pt, // Coordinates
                    cv::FONT_HERSHEY_PLAIN, // Font
                    1.0, // Scale. 2.0 = 2x bigger
                    color_rnd, // Color
                    1); // Thickness
    }

    hconcat(img_l_aux2,img_l_aux1,img_l_aux);
    //    if( img_l_aux.channels() == 1 )
    //        cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);
    cv::putText(img_l_aux,
                std::to_string((image_ls_l.size())),
                cv::Point(25, 25), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                2.0, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2); // Thickness

    this->stereo_frame_match = img_l_aux;

    //
    if (save_screen_shot) {
        std::string frame_wid = save_path + "/stereo/";
        frame_wid += std::to_string(this->frame_idx);
        frame_wid += ".png";
        imwrite( frame_wid, this->stereo_frame_match );
    }
}

}
