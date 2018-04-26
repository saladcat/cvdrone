#include "ardrone/ardrone.h"
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <opencv2\aruco.hpp>
#include "pid.hpp"
#include <cmath>
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

using namespace std;
using namespace cv;

const double markerLength = 0.123;
int main(int argc, char *argv[])
{
	PIDManager PID("pid.yaml");	
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }
	FileStorage fs("camera.xml", FileStorage::READ);
	// Load camera parameters
	cv::Mat cameraMatrix, distCoeffs;
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;
		cv::Mat image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
			if (ardrone.onGround()) { 
				ardrone.takeoff(); 
			}
			else {
				ardrone.landing();
			}
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);
        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);


		if (key == -1) {
			// implement your autopilot algorithm here
			// only need to modify vx, vy, vz, vr

			// Get an image
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
			cv::aruco::detectMarkers(image, dictionary, corners, ids);
			std::vector<cv::Vec3d> rvecs, tvecs;
			if (ids.size() != 0) {//if dect marker
				cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
//				cout << tvecs[0] << endl;
				aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
				//2 是前后
				//1 是上下
				//0 是左右
				Mat error(4, 1, CV_64F);
				Mat move_dir(4, 1, CV_64F);
				error.at<double>(1,0) = tvecs[0][0];
				error.at<double>(2,0) = tvecs[0][1];
				error.at<double>(0,0) = tvecs[0][2] - 0.8	;
				error.at<double>(3,0) = rvecs[0][2];
				double tem = rvecs[0][0] * rvecs[0][2];
				if (tem >=0) { 
					error.at<double>(3, 0) =  fabs(rvecs[0][2]);
				} else {
					error.at<double>(3, 0) = -fabs(rvecs[0][2]);
				}
				cout << error << endl;
				if (error.at<double>(3, 0) < 0.3&& error.at<double>(3, 0) > -0.3) {
					error.at<double>(3, 0) = 0;
				}
				
				//cout << rvecs[0] << endl;
				PID.getCommand(error, move_dir);
				cout << move_dir<<endl;
				//vx = 0;
				vx = 10.0 * move_dir.at<double>(0, 0);
				//vy = 0;
				vy = -3.0*move_dir.at<double>(1,0);
				vz = 0;
				//vz = -move_dir.at<double>(2,0);
				vr = move_dir.at<double>(3	,0);
				//vr = 0;
				// VX是前后
				// VY是左右
				// VZ是上下
				cout << endl;
			}

		}

		ardrone.move3D(vx, vy, vz, vr);

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}
