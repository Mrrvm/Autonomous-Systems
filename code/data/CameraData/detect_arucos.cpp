#include "defs.hpp"
#include <iostream>
#include <fstream> 
#include <sstream>

#define N_SAMPLES 810
#define SAMPLES_DIR "../../../../../../SA_arucos2/*.jpg"  
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
    Mat rMatrix;
    Mat tCamAruco, tCamCenter;

    double tCamCenterNorm;
    double tCamArucoNorm;
    double dotProduct;
    double teta;
    double distance;
    
    int i = 0, j = 0;
    while(i < N_SAMPLES) {
        
        timestamp = FileName(fn[i]);

        undistort(imread(fn[i]), inputImage, intrinsics, distCoeffs);
        //imshow("inputImage", inputImage);
        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);
        //waitKey(0);
        cout << "Detected " << markerIds.size() << " arucos" << endl;
        if (markerIds.size() > 0) {
            cout << "IDs: " << markerIds[0] << endl;
            
            // The returned transformation is the one that transforms points from each marker coordinate 
            //  system to the camera coordinate system
            cv::aruco::estimatePoseSingleMarkers(markerCorners, MarkersSide, intrinsics, distCoeffs, rvecs, tvecs);
            
            outfile << timestamp << " " << markerIds.size() << " ";
            cout << timestamp << " " << markerIds.size() << " ";
            

            for(j=0; j<markerIds.size(); j++) {

                //cout << "Landmark[" << j << "]:" << endl;
                //cout << "Rotation vector: " << rvecs[j] << endl << "Translation vector: "  << tvecs[j] << endl;
                
                // Get rotation matrix
                cv::Rodrigues(rvecs[j], rMatrix);
                //cout << "Rotation matrix: " << endl << rMatrix << endl;

                // Get translation vector into camera's reference point
                tCamAruco = rMatrix*Mat(tvecs[j]);
                //cout << "Translation vector in Camera's reference point " << endl << tCamAruco << endl;
                
                // Image center from camera's reference point
                tCamCenter = (Mat_<double>(1, 3) << 0, 0, tCamAruco.at<double>(2));
               
                // Computation of teta between vector through |a|.|b| = ||a||x||b||xcos(a,b)
                tCamCenterNorm = norm(tCamCenter, NORM_L2);
                tCamArucoNorm = norm(tCamAruco, NORM_L2);
                dotProduct = tCamAruco.at<double>(2)*tCamCenter.at<double>(2);
                teta = acos(dotProduct/(tCamArucoNorm*tCamCenterNorm));
                
                // Computation of distance to aruco through sqrt(a^2 + b^2)
                distance = sqrt(tCamCenter.at<double>(0)*tCamCenter.at<double>(0) + tCamCenter.at<double>(2)*tCamCenter.at<double>(2));

                //cout << "teta is " << teta << endl;
                //cout << "distance is " << distance << endl;

                outfile << "[" << markerIds[j] << " " << teta << " " << distance << "] ";
                cout << "[" << teta << " " << distance << "] ";

                rMatrix.release();
                rvecs.clear();
                tvecs.clear();
                tCamAruco.release();
                tCamCenter.release();
            }

            outfile << endl;
            cout << endl;

        }
        i++;
    }
	
	return 0;
}