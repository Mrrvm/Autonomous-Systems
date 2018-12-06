#include "defs.hpp"
#include <iostream>
#include <fstream> 
#include <sstream>

#define N_SAMPLES 810
#define SAMPLES_DIR "../../../../../../arucos/ArucosGT1/*.jpg"  
#define MarkersSide 0.15 //15 cm

void GetCalibration(Mat& intrinsics, Mat& distCoeffs) {
    FileStorage fs("intrinsics.xml", FileStorage::READ);
    if (fs.isOpened()) {
      fs["intrinsics"] >> intrinsics;
      fs["distCoeffs"] >> distCoeffs;
      fs.release();
    }
    else
        exit(0);
}

string FileName(const string& str) {
  size_t slash = str.find_last_of("/\\");
  size_t dot = str.find_last_of(".\\");
  size_t len = dot-slash;
  string name = str.substr(slash+1, len-1);
  return name;
}

int main(int argc, char const *argv[]) {

    ofstream outfile ("landmark.txt");

    cv::String path(SAMPLES_DIR);
    vector<cv::String> fn;
    vector<cv::Mat> data;
    cv::glob(path,fn,true);
    string timestamp;

    Mat inputImage, marker;
    vector<int> markerIds;
	vector<vector<cv::Point2f>> markerCorners;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    vector<cv::Vec3d> rvecs, tvecs;
    Mat intrinsics, distCoeffs;
    GetCalibration(intrinsics, distCoeffs);

    double teta;
    double distance;
    Mat rMatrix;
    Mat tvecsCam;
    
    int i = 0, j = 0;
    while(i < N_SAMPLES) {
        
        timestamp = FileName(fn[i]);

        undistort(imread(fn[i]), inputImage, intrinsics, distCoeffs);
        //imshow("inputImage", inputImage);
        //waitKey(0);
        
        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);
        
        cout << "Detected " << markerIds.size() << " arucos" << endl;
        
        if (markerIds.size() > 0) {
            cout << "IDs: " << markerIds[0] << endl;
            
            // The returned transformation is the one that transforms points from each marker coordinate 
            //  system to the camera coordinate system
            cv::aruco::estimatePoseSingleMarkers(markerCorners, MarkersSide, intrinsics, distCoeffs, rvecs, tvecs);
            
            outfile << timestamp << " " << markerIds.size() << " ";
            cout << timestamp << endl;

            cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);
            
            for(j=0; j<markerIds.size(); j++) {

                cout << "Landmark[" << j << "]:" << endl;
                                
                cv::Rodrigues(rvecs[j], rMatrix);
                tvecsCam = -rMatrix.t()*Mat(tvecs[j]);

                cout << "Translation vector: " << endl << tvecsCam << endl;

                double z = tvecsCam.at<double>(2);
                double x = tvecsCam.at<double>(0);
                
                // Computation of distance to aruco through sqrt(a^2 + b^2)
                distance = sqrt(z*z + x*x);
                teta = atan2(z,x) - 1.570796327;

                outfile << markerIds[j] << " " << teta << " " << distance << " ";
                cout << "[id teta distance]" << endl << "[" << markerIds[j] << " " << teta << " " << distance << "] " << endl;

                //cv::aruco::drawAxis(inputImage, intrinsics, distCoeffs, rvecs[i], tvecs[i], 0.1);
                //cv::imshow("OutputImage", inputImage);
                //waitKey(0);

                rMatrix.release();
                tvecsCam.release();

            }

            rvecs.clear();
            tvecs.clear();
            outfile << endl;
            cout << endl;

        }
        i++;
        inputImage.release();
    }
	
	return 0;
}