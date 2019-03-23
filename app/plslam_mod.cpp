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

#ifdef HAS_MRPT
#include <slamScene.h>
#include <mrpt/utils/CTicTac.h>
#endif

#include <ctime>
#include <algorithm>

#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include <ctime>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include <mapFeatures.h>
#include <mapHandler.h>

using namespace StVO;
using namespace PLSLAM;


// comparison, not case sensitive.
bool compareTimestamp (const std::string& first, const std::string& second)
{
    long time_stamp_l = std::stol( first );
    long time_stamp_r = std::stol( second );
    return ( time_stamp_l < time_stamp_r );
}


int main(int argc, char **argv) {

    // read inputs
    //    int frame_offset = 0, frame_number = 0, frame_step = 1;

    std::cout << std::endl << "% PL-SLAM Version 1.00" << std::endl;
    if(argc != 5)
    {
        cerr << endl
             << "Usage: ./plslam_mod benchmark path_to_settings "
             << "path_to_image path_to_results" << endl;
        return -1;
    }

    // Load Settings and Check
    string strSettingsFile = argv[2];
    std::cout << "Load settings and check " << strSettingsFile << std::endl;
    // read content of the .yaml dataset configuration file
    YAML::Node dset_config = YAML::LoadFile( strSettingsFile.c_str() );

    // setup camera
    YAML::Node cam_config = dset_config["cam0"];
    string camera_model = cam_config["cam_model"].as<string>();
    PinholeStereoCamera*  cam_pin;
    bool rectify = false;
    if( camera_model == "Pinhole" )
    {
        // if EuRoC or Falcon yaml file
        if( cam_config["Kl"].IsDefined() )
        {
            rectify = true;
            Mat Kl, Kr, Dl, Dr, R, t;
            vector<double> Kl_ = cam_config["Kl"].as<vector<double>>();
            vector<double> Kr_ = cam_config["Kr"].as<vector<double>>();
            vector<double> Dl_ = cam_config["Dl"].as<vector<double>>();
            vector<double> Dr_ = cam_config["Dr"].as<vector<double>>();
            Kl = ( Mat_<float>(3,3) << Kl_[0], 0.0, Kl_[2], 0.0, Kl_[1], Kl_[3], 0.0, 0.0, 1.0 );
            Kr = ( Mat_<float>(3,3) << Kr_[0], 0.0, Kr_[2], 0.0, Kr_[1], Kr_[3], 0.0, 0.0, 1.0 );
            // load rotation and translation
            vector<double> R_ = cam_config["R"].as<vector<double>>();
            vector<double> t_ = cam_config["t"].as<vector<double>>();
            R = Mat::eye(3,3,CV_64F);
            t = Mat::eye(3,1,CV_64F);
            int k = 0;
            for( int i = 0; i < 3; i++ )
            {
                t.at<double>(i,0) = t_[i];
                for( int j = 0; j < 3; j++, k++ )
                    R.at<double>(i,j) = R_[k];
            }
            // load distortion parameters
            int Nd = Dl_.size();
            Dl = Mat::eye(1,Nd,CV_64F);
            Dr = Mat::eye(1,Nd,CV_64F);
            for( int i = 0; i < Nd; i++ )
            {
                Dl.at<double>(0,i) = Dl_[i];
                Dr.at<double>(0,i) = Dr_[i];
            }
            // if dtype is equidistant (now it is default)
            if( cam_config["dtype"].IsDefined() )
            {
                cam_pin = new PinholeStereoCamera(
                            cam_config["cam_width"].as<double>(),
                        cam_config["cam_height"].as<double>(),
                        cam_config["cam_bl"].as<double>(),
                        Kl, Kr, R, t, Dl, Dr, true);

            }
            else {
                // create camera object for EuRoC
                cam_pin = new PinholeStereoCamera(
                            cam_config["cam_width"].as<double>(),
                        cam_config["cam_height"].as<double>(),
                        cam_config["cam_bl"].as<double>(),
                        Kl, Kr, R, t, Dl, Dr, false);
            }
        }
        else {
            cam_pin = new PinholeStereoCamera(
                        cam_config["cam_width"].as<double>(),
                    cam_config["cam_height"].as<double>(),
                    fabs(cam_config["cam_fx"].as<double>()),
                    fabs(cam_config["cam_fy"].as<double>()),
                    cam_config["cam_cx"].as<double>(),
                    cam_config["cam_cy"].as<double>(),
                    cam_config["cam_bl"].as<double>(),
                    cam_config["cam_d0"].as<double>(),
                    cam_config["cam_d1"].as<double>(),
                    cam_config["cam_d2"].as<double>(),
                    cam_config["cam_d3"].as<double>()  );
        }

    }
    else
    {
        cout << endl << "Not implemented yet." << endl;
        return -1;
    }

    // setup image directories
    string img_dir = string(argv[3]);
    string img_dir_l = img_dir + "/" + dset_config["images_subfolder_l"].as<string>();
    string img_dir_r = img_dir + "/" + dset_config["images_subfolder_r"].as<string>();

    // get a sorted list of files in the img directories
    boost::filesystem::path img_dir_path_l(img_dir_l.c_str());
    if (!boost::filesystem::exists(img_dir_path_l))
    {
        cout << endl << "Left image directory does not exist: \t" << img_dir_l << endl;
        return -1;
    }
    boost::filesystem::path img_dir_path_r(img_dir_r.c_str());
    if (!boost::filesystem::exists(img_dir_path_r))
    {
        cout << endl << "Right image directory does not exist: \t" << img_dir_r << endl;
        return -1;
    }

    // get all files in the img directories
    size_t max_len_l = 0;
    std::list<std::string> imgs_l;
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator file(img_dir_path_l); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                (filename_path.extension() == ".png"  ||
                 filename_path.extension() == ".jpg"  ||
                 filename_path.extension() == ".jpeg" ||
                 filename_path.extension() == ".pnm"  ||
                 filename_path.extension() == ".tiff") )
        {
            std::string filename(filename_path.string());
            imgs_l.push_back(filename);
            max_len_l = max(max_len_l, filename.length());
        }
    }
    size_t max_len_r = 0;
    std::list<std::string> imgs_r;
    for (boost::filesystem::directory_iterator file(img_dir_path_r); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                (filename_path.extension() == ".png"  ||
                 filename_path.extension() == ".jpg"  ||
                 filename_path.extension() == ".jpeg" ||
                 filename_path.extension() == ".pnm"  ||
                 filename_path.extension() == ".tiff") )
        {
            std::string filename(filename_path.string());
            imgs_r.push_back(filename);
            max_len_r = max(max_len_r, filename.length());
        }
    }

    imgs_l.sort(compareTimestamp);
    imgs_r.sort(compareTimestamp);

    //    // extract the image files with common name
    //    std::vector<std::string> img_lr_pair; // (n_imgs_l + n_imgs_r);
    //    std::set_intersection (imgs_l.begin(), imgs_l.end(),
    //                           imgs_r.begin(), imgs_r.end(),
    //                           std::back_inserter(img_lr_pair));
    //    std::cout << std::endl << "Number of stereo image pairs: " << img_lr_pair.size() << std::endl;

    std::vector<std::string> imgs_l_paired;
    std::vector<std::string> imgs_r_paired;
    std::list<std::string>::iterator it_r = imgs_r.begin();
    for (std::list<std::string>::iterator it_l = imgs_l.begin(); it_l != imgs_l.end(); ++ it_l) {
        while (it_r != imgs_r.end()) {
            // compare the timestamp between i & j
            long time_stamp_l = std::stol( *it_l );
            long time_stamp_r = std::stol( *it_r );
            if (fabs((double)((long double)time_stamp_l / 1000000000) - (double)((long double)time_stamp_r / 1000000000)) < 0.003) {
                // pair left & right frames
                imgs_l_paired.push_back( *it_l );
                imgs_r_paired.push_back( *it_r );
                // move on to the next left index
                it_r ++;
                break ;
            }
            else if ((double)((long double)time_stamp_l / 1000000000) < (double)((long double)time_stamp_r / 1000000000)) {
                // skip current left index
                break ;
            }
            else {
                // move on to the next right index
                it_r ++;
            }
        }
    }
    assert(imgs_l_paired.size() == imgs_r_paired.size());
    std::cout << std::endl << "Number of stereo image pairs: " << imgs_r_paired.size() << std::endl;

    // create scene
#ifdef HAS_MRPT
    string scene_cfg_name;
    string strBenchmark = argv[1];
    if( strBenchmark.compare("kitti") == 0 || strBenchmark.compare("malaga") == 0 )
        scene_cfg_name = "../config/scene_config.ini";
    else
        scene_cfg_name = "../config/scene_config_indoor.ini";
    slamScene scene(scene_cfg_name);
    Matrix4d Tcw, Tfw = Matrix4d::Identity(), Tfw_prev = Matrix4d::Identity(), T_inc;
    Vector6d cov_eig;
    Matrix6d cov;
    Tcw = Matrix4d::Identity();
    scene.setStereoCalibration( cam_pin->getK(), cam_pin->getB() );
    scene.initializeScene(Tfw);
#endif


    vector<double> time_list;
    //
    // if ( strBenchmark.compare("kitti") == 0 ) {

    //     // load the time.txt for KITTI
    //     string time_fname = img_dir + "/" + std::string("times.txt");
    //     ifstream timeloader;
    //     timeloader.open (time_fname);
    //     double timetemp;
    //     while ( timeloader >> timetemp ) {
    //         time_list.push_back(timetemp);
    //     }
    //     timeloader.close();
    //     //
    //     assert(imgs_l_paired.size() == time_list.size());
    //     std::cout << std::endl << "Number of time record loaded: " << time_list.size() << std::endl;

    // }


    // save the pose per frame into text
    ofstream fAllFrameTrack;
    std::string fNameAllFrameTrack = std::string(argv[4]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to " << fNameAllFrameTrack << std::endl;
    fAllFrameTrack.open(fNameAllFrameTrack.c_str());
    fAllFrameTrack << fixed;
    fAllFrameTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;


    ofstream fLog;
    std::string fNameLog = std::string(argv[4]) + "_Log.txt";
    std::cout << std::endl << "Saving Log Info to " << fNameLog << std::endl;
    fLog.open(fNameLog.c_str());
    fLog << fixed;
    fLog << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;



    // create PLSLAM object
    PLSLAM::MapHandler* map = new PLSLAM::MapHandler(cam_pin);

    cout << "created MapHandler!" << endl;

    // initialize and run PL-StVO
    int frame_counter = 0;
    StereoFrameHandler* StVO = new StereoFrameHandler(cam_pin);

    cout << "created StereoFrameHandler!" << endl;

    //    for (std::map<std::string, std::string>::iterator it_l = sorted_imgs_l.begin(), it_r = sorted_imgs_r.begin();
    //         it_l != sorted_imgs_l.end(), it_r != sorted_imgs_r.end(); ++it_l, ++it_r, frame_counter++) {
    for (int i = 0; i < imgs_l_paired.size(); ++ i, frame_counter++) {

        std::cout << "------------------------------------------   Frame #" << frame_counter
                  << "   ----------------------------------------" << std::endl;

        //        if (frame_counter >= 100)
        //            break ;
        clock_t startTime = clock();

        // load images
        boost::filesystem::path img_path_l = img_dir_path_l / boost::filesystem::path(imgs_l_paired[i].c_str());
        boost::filesystem::path img_path_r = img_dir_path_r / boost::filesystem::path(imgs_r_paired[i].c_str());
        Mat img_l( imread(img_path_l.string(), CV_LOAD_IMAGE_UNCHANGED) );  assert(!img_l.empty());
        Mat img_r( imread(img_path_r.string(), CV_LOAD_IMAGE_UNCHANGED) );  assert(!img_r.empty());

        // grab the time stamp from image file name
        double time_stamp;
        if ( false /*strBenchmark.compare("kitti") == 0 */ ) {
            // set time_stamp with the time.txt record
            time_stamp = time_list[i];
        }
        else {
            // set time_stamp with the image file name
            long time_stamp_l = std::stol( imgs_l_paired[i] );
            long time_stamp_r = std::stol( imgs_r_paired[i] );
            time_stamp = ( (double)((long double)time_stamp_l / 1000000000) +
                           (double)((long double)time_stamp_r / 1000000000) ) / 2.0;
        }

        // if images are distorted
        if( rectify )
        {
            Mat img_l_rec, img_r_rec;
            cam_pin->rectifyImagesLR(img_l,img_l_rec,img_r,img_r_rec);
            img_l = img_l_rec;
            img_r = img_r_rec;
        }


#ifdef SIMU_MOTION_BLUR

    // Update kernel size for a normalized box filter
    int kernel_size = 5; // 9;
    cv::Mat kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

    cv::filter2D(img_l, img_l, -1, kernel);
    cv::filter2D(img_r, img_r, -1, kernel);

#endif


        double costTime = double(clock() - startTime) / double(CLOCKS_PER_SEC);
#ifdef TIMECOST_VERBOSE
        std::cout << "s1. image loading time = " << costTime << std::endl;
#endif

        // initialize
        if( frame_counter == 0 )
        {
            StVO->initialize(img_l,img_r,0, time_stamp);
            PLSLAM::KeyFrame* kf = new PLSLAM::KeyFrame( StVO->prev_frame, 0 );
            map->initialize( kf );
            //scene.initViewports( img_l.cols, img_r.rows );
        }
        // run
        else
        {
            // PL-StVO
            startTime = clock();
            StVO->insertStereoPair( img_l, img_r, frame_counter, time_stamp );
            StVO->curr_frame->log_.frame_time_stamp = time_stamp;
            StVO->curr_frame->log_.time_track = double(clock() - startTime) / double(CLOCKS_PER_SEC);
            #ifdef TIMECOST_VERBOSE
            std::cout << "s2. total time on data association = " << StVO->curr_frame->log_.time_track << std::endl;
#endif
            //
            // NOTE
            // When only line features are used, optimizing the pose from scratch will lead to better results
            // The reason could be that, line feature matching are not reliable at this moment, therefore leads
            // to highly unstable pose estimation as well as prediction;
            // On the contrary, the robustness of pose tracking will be dramatically improved with pose prediction
            // when point feature matchings are used
            // In the future, it's worth upgrading the line matching code (based on my previous work done at MagicLeap)
            //
            // if (Config::hasPoints() == false && Config::hasLines() == true)
            //     StVO->optimizePose();
            // else
            //     StVO->optimizePose(StVO->prev_frame->DT);
            //            StVO->optimizePose();
            startTime = clock();
            StVO->optimizePose(StVO->prev_frame->DT);
            StVO->curr_frame->log_.time_pose_optim = double(clock() - startTime) / double(CLOCKS_PER_SEC);
            #ifdef TIMECOST_VERBOSE
            std::cout << "s3. pose optimization time = " << StVO->curr_frame->log_.time_pose_optim << std::endl;
#endif
            StVO->curr_frame->log_.time_track += StVO->curr_frame->log_.time_pose_optim;

            //
            std::cout << "numFrameLoss = " << StVO->numFrameLoss << std::endl;
            if (StVO->numFrameLoss > Config::maxNumFrameLoss()) {
                // early termination
                std::cout << "Early termination due to track loss!" << std::endl;
                break ;
            }


            KeyFrame* prev_kf = map->map_keyframes.back();
            Matrix4d T_base = prev_kf->T_kf_w;

            // viz only
            #ifdef DO_VIZ
            // StVO->plotMatchedLines( true, true );
     // StVO->plotMatchedPointLine(false);
     StVO->appendPlotCutLine(true);
     // StVO->appendPlotOptLine(true);
#endif
            //
            // check if a new keyframe is needed
            if( StVO->needNewKF() )
            {
                startTime = clock();

                std::cout <<         "#KeyFrame:     " << map->max_kf_idx + 1;
                std::cout << std::endl << "#Points:       " << map->map_points.size();
                std::cout << std::endl << "#Segments:     " << map->map_lines.size();
                std::cout << std::endl << std::endl;
                // grab StF and update KF in StVO (the StVO thread can continue after this point)
                PLSLAM::KeyFrame* curr_kf = new PLSLAM::KeyFrame( StVO->curr_frame );
                // update KF in StVO
                StVO->currFrameIsKF();

                // NOTE
                // add by Yipu to fix the null x issue
                // scene.setText(frame_counter, time_stamp,
                //               StVO->n_inliers_pt, StVO->matched_pt.size(),
                //               StVO->n_inliers_ls, StVO->matched_ls.size());
                // scene.setPose( curr_kf->T_kf_w );

                // add keyframe and features to map
                map->addKeyFrame( curr_kf );

                costTime = double(clock() - startTime) / double(CLOCKS_PER_SEC);
                std::cout << "local BA time = " << costTime << std::endl;

                // update scene
#ifdef HAS_MRPT
                StVO->curr_frame->plotStereoFrameLeft( false );
                scene.setImage( StVO->curr_frame->stereo_frame_match );
                //                imwrite("../config/aux/img_aux.png", StVO->curr_frame->stereo_frame_match);
                //                scene.setImage( "../config/aux/img_aux.png" );
                //
                scene.updateScene( map );
#endif
            }

            //            StVO->curr_frame->plotStereoFrameLeft( true );
            //            scene.setImage( StVO->curr_frame->stereo_frame_match );

            // update StVO
            StVO->updateFrame_ECCV18( T_base );


            // real time tracking results logging
            if (StVO->vec_all_frame_pose.size() > 0) {
                //double time_stamp = StVO->T_allF[StVO->T_allF.size() - 1].first;
                Matrix4d Tfw = StVO->vec_all_frame_pose[StVO->vec_all_frame_pose.size() - 1];

                Matrix3d R = Tfw.block(0,0,3,3).transpose();
                vector<float> q = toQuaternion(R);

                fAllFrameTrack << /*setprecision(6) << time_stamp << */setprecision(7) << " "
                               << Tfw(0, 3) << " " << Tfw(1, 3) << " " << Tfw(2, 3) << " "
                               << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            }

            //
            fLog << setprecision(6)
                 << StVO->prev_frame->log_.frame_time_stamp << " "
                 << setprecision(7)
                 << StVO->prev_frame->log_.time_track << " "
                 << StVO->prev_frame->log_.time_pt_extract << " "
                 << StVO->prev_frame->log_.time_ln_detect << " "
                 << StVO->prev_frame->log_.time_ln_descri << " "
                 << StVO->prev_frame->log_.time_pt_stereo << " "
                 << StVO->prev_frame->log_.time_ln_stereo << " "
                 << StVO->prev_frame->log_.time_pt_cross << " "
                 << StVO->prev_frame->log_.time_ln_cross << " "
                 << StVO->prev_frame->log_.time_ln_cut << " "
                 << StVO->prev_frame->log_.time_pose_optim << " "
                 << setprecision(0)
                 << StVO->prev_frame->log_.num_pt_detect << " "
                 << StVO->prev_frame->log_.num_ln_detect << " "
                 << StVO->prev_frame->log_.num_pt_stereo << " "
                 << StVO->prev_frame->log_.num_ln_stereo << " "
                 << StVO->prev_frame->log_.num_pt_cross << " "
                 << StVO->prev_frame->log_.num_ln_cross << endl;
        }
    }

    // finish SLAM
    map->finishSLAM();
    fAllFrameTrack.close();


//#ifdef HAS_MRPT
//    scene.setImage( StVO->curr_frame->stereo_frame_match );
//    //    scene.setImage( "../config/aux/img_aux.png" );
//    scene.updateSceneGraphs( map );
//#endif
//    //    getchar();
//    // perform GBA
//    cout << endl << "Performing Global Bundle Adjustment..." ;
//    map->globalBundleAdjustment();
//#ifdef HAS_MRPT
//    scene.setImage( StVO->curr_frame->stereo_frame_match );
//    //    scene.setImage( "../config/aux/img_aux.png" );
//    scene.updateSceneGraphs( map );
//#endif


    // save the pose per frame into text
    std::cout << std::endl << "Saving KeyFrame Trajectory to KeyFrameTrajectory.txt" << std::endl;
    ofstream fKeyFrameTrack;
    std::string fNameKeyFrameTrack = std::string(argv[4]) + "_KeyFrameTrajectory.txt";
    fKeyFrameTrack.open(fNameKeyFrameTrack.c_str());
    fKeyFrameTrack << fixed;
    fKeyFrameTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;

    for(size_t i=0; i<map->map_keyframes.size(); i++)
    {
        KeyFrame * cur_kF = map->map_keyframes[i];

        if (cur_kF != NULL) {

            double time_stamp = cur_kF->stereo_frame->time_stamp;
            Matrix4d T_kf_w = cur_kF->T_kf_w;

            //            Matrix3d R = T_kf_w.block(0,0,3,3).transpose();
            Matrix3d R = T_kf_w.block(0,0,3,3);
            vector<float> q = toQuaternion(R);

            fKeyFrameTrack << setprecision(6) << time_stamp << setprecision(7) << " "
                           << T_kf_w(0, 3) << " " << T_kf_w(1, 3) << " " << T_kf_w(2, 3) << " "
                           << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
    }

    fKeyFrameTrack.close();

    std::cout << " ... done." << std::endl;

    // wait until the scene is closed
    #ifdef HAS_MRPT
       while( scene.isOpen() );
    #endif

    return 0;

}
