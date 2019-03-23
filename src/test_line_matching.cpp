#include <stereoFrameHandler.h>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>

int main() {
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

    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
    t.at<float>(2, 0) = 1.0;

    PinholeStereoCamera* camera =
            new PinholeStereoCamera (int(640),int(480),double(0.12),K,K,R,t,D,D,false);
    StVO::StereoFrameHandler* Stvo =
            new StVO::StereoFrameHandler(camera);

    Stvo->setLeftCamExtrinCalib(0, 0, 0, 0, 0, 0);

    std::string path = "/home/pang/data/dataset/euroc/MH_01_easy/mav0/cam0/data";
    std::vector<std::string> img_fname_list;
    DIR* dirp = opendir(path.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        if (dp->d_type == DT_REG) {
            img_fname_list.push_back(dp->d_name);
        }
    }
    closedir(dirp);

    //
    std::sort(img_fname_list.begin(), img_fname_list.end());

    for (int i=0; i<img_fname_list.size(); ++i) {

        std::cout << "proc img: " << img_fname_list[i] << std::endl;

        cv::Mat img = cv::imread(path + std::string(img_fname_list[i]));
        //cvtColor(img, img, CV_BGR2GRAY);

        if (i == 0)
            Stvo->initialize(img, img, i, uint64_t(i));
        else {
            Stvo->insertStereoPair(img, img, i, uint64_t(i));
            cv::imshow("line detection", Stvo->curr_frame->stereo_frame_detect);
            cv::imshow("line stereo", Stvo->curr_frame->stereo_frame_match);
            cv::imshow("line matching", Stvo->canvas_match_frames);
//            cv::waitKey(0);
        }
        Stvo->updateFrame();
    }
}

