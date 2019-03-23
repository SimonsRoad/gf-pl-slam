// Copyright 2005, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// A sample program demonstrating using Google C++ testing framework.
//
// Author: wan@google.com (Zhanyong Wan)


// In this example, we use a more advanced feature of Google Test called
// test fixture.
//
// A test fixture is a place to hold objects and functions shared by
// all tests in a test case.  Using a test fixture avoids duplicating
// the test code necessary to initialize and cleanup those common
// objects for each test.  It is also useful for defining sub-routines
// that your tests need to invoke a lot.
//
// <TechnicalDetails>
//
// The tests share the test fixture in the sense of code sharing, not
// data sharing.  Each test is given its own fresh copy of the
// fixture.  You cannot expect the data modified by one test to be
// passed on to another test, which is a bad idea.
//
// The reason for this design is that tests should be independent and
// repeatable.  In particular, a test should not fail as the result of
// another test's failure.  If one test depends on info produced by
// another test, then the two tests should really be one big test.
//
// The macros for indicating the success/failure of a test
// (EXPECT_TRUE, FAIL, etc) need to know what the current test is
// (when Google Test prints the test result, it tells you which test
// each failure belongs to).  Technically, these macros invoke a
// member function of the Test class.  Therefore, you cannot use them
// in a global function.  That's why you should put test sub-routines
// in a test fixture.
//
// </TechnicalDetails>

#include <yaml-cpp/yaml.h>
#include "stereoFrameHandler.h"
#include "gtest/gtest.h"
//#include "gtest/gmock.h"

using namespace StVO;

namespace {
// To use a test fixture, derive a class from testing::Test.
class TestLineCut : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {

        // Load Settings and Check
        string strSettingsFile = "..//config//gazebo_params.yaml";
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
                else
                    // create camera object for EuRoC
                    cam_pin = new PinholeStereoCamera(
                                cam_config["cam_width"].as<double>(),
                            cam_config["cam_height"].as<double>(),
                            cam_config["cam_bl"].as<double>(),
                            Kl, Kr, R, t, Dl, Dr,false);
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


        StVO = new StereoFrameHandler(cam_pin);
        //        StVO->initialize(img_l, img_r, 0, time_stamp);
        //        StVO->insertStereoPair( img_l, img_r, frame_counter, time_stamp );

        // set stereo feature at previous frame

        // set motion prediction and motion ground truth
        prev_frame->Tfw * prev_frame->DT;
        StVO->predictFramePose();

        // set stereo feature at current frame


        // set pose


    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    StereoFrameHandler* StVO;
};

// When you have a test fixture, you define a test using TEST_F
// instead of TEST.
// Tests the Kinematic/System Jacobian.
TEST_F(TestLineCut, maxVolCut) {
    //
    double rngCutRatio[2] = {0, 1.0};
    StVO->estimateProjUncertainty_submodular( 0.05, rngCutRatio );

    StVO->optimizePose(StVO->prev_frame->DT);

    StVO->curr_frame->Tfw

            EXPECT_NEAR(arma::norm(obs_->kinematic[i].F_Q - F_Q, "inf"), 0, 0.002);

}

}  // namespace
