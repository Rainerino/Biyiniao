#include "opencv2/highgui/highgui.hpp"
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>


using namespace cv;
using namespace std;


cv::Mat cameraMatrix, distCoeffs;

// camera parameters are read from somewhere
int readCameraParameters(string path) {
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open " << path << endl;
        return 1;
    }

    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    cout << cameraMatrix << endl;
    cout << distCoeffs << endl;
    return 0;
}

double eulidDistance(Point2f p1, Point2f p2) {
    return sqrt(pow(abs(p1.x - p2.x), 2) + pow(abs(p1.y - p2.y), 2));
}

int main(int argc, char **argv) {
/*
 *
    //create image marker
    cv::Mat markerImage;
    const Ptr<aruco::Dictionary> &dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
*/

    readCameraParameters(
            "/home/yiyi/Documents/Project/Biyiniao/Project_Program/Drafts/catkin_ws/src/mavros_takeoff/out_camera_data.xml");


    VideoCapture inputVideo(1);

    const Ptr<aruco::Dictionary> &dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat image, imageCopy, src;
    while (1) {
//        inputVideo >> image;
        inputVideo >> src;
        image = src.clone();
        undistort(src, image, cameraMatrix, distCoeffs);
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;

        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        // if at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
//            for (vector<vector<cv::Point2f>>::const_iterator i = corners.begin();  i < corners.end(); i ++){
//                for ( vector<cv::Point2f>::const_iterator j  = i.begin(); j < i.end(); j ++){
//                    cout << *j << endl;
//                }
//
//            }

//TODO bugged: Need to order points
            float p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, a1, a2, b1, b2, intx, inty;

            for (unsigned i = 0; i < corners.size(); i++) {

                p1x = corners[i][0].x;
                p2x = corners[i][1].x;
                p3x = corners[i][2].x;
                p4x = corners[i][3].x;

                p1y = corners[i][0].y;
                p2y = corners[i][1].y;
                p3y = corners[i][2].y;
                p4y = corners[i][3].y;

                a1 = (p2y - p4y) / (p2x - p4x);
                a2 = (p1y - p2y) / (p1x - p3x);

                b1 = p4y - a1 * p4x;
                b2 = p3y - a2 * p3x;

                intx = (b2 - b1) / (a1 - a2);
                inty = a1 * intx + b1;

//                cout << "Intersect at: x:"<< intx << " y:" << inty << endl;

                circle(imageCopy, Point((int) intx, (int) inty), 5, cv::Scalar(0, 0, 255), 2);

            }

//            //TODO: get the average of the length: F ~ 94 pix
            //TODO: maybe get a calibration scrpt. Nay, only use it once

//
            float l1, l2, l3, l4, lavg;
            l1 = (float) eulidDistance(Point2f(p1x, p1y), Point2f(p2x, p2y));
            l2 = (float) eulidDistance(Point2f(p2x, p2y), Point2f(p3x, p3y));
            l3 = (float) eulidDistance(Point2f(p3x, p3y), Point2f(p4x, p4y));
            l4 = (float) eulidDistance(Point2f(p4x, p4y), Point2f(p1x, p1y));

            lavg = (l1 + l2 + l3 + l4) / 4;
//
//            //F = (P(pix) x  D(distance)) / W(physical width) = 94 pix
//            //D = F * W / P

            double D = 94.0 * 15 / lavg;

            cout << "Distance is about " << D << endl;
            cout << lavg << endl;

            std::ostringstream strs;
            strs << D;
            std::string str = strs.str();
            cv::putText(imageCopy, "Distance is: " + str, cv::Point(30,30),  FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(255,255,250), 1);


            vector<Vec3d> rvecs, tvecs;

            //0.05 m
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            //draw axis for each marker
//            for (int i = 0; i < ids.size(); i++)
//                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

        }


        cv::imshow("out", imageCopy);


        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        }

    }

    inputVideo.release();
    destroyAllWindows();
}

