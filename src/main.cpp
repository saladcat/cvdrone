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
const double markerLength = 0.094;

int getMarkID(int ID,vector<int> ids) {
	for (int i = 0; i < ids.size(); i++) {
		if (ID == ids[i]) {
			return i;
		}
	}
	return -1;
}

bool find_mark(vector<int> ids,Mat & move_dir) {
	move_dir.at<double>(0, 0) = 0;
	move_dir.at<double>(1, 0) = 0;
	move_dir.at<double>(2, 0) = 0;
	move_dir.at<double>(3, 0) = 1;

	if (ids.size() <= 0) {
		return false;
	}
	if (ids[0] == 1) {
		return true;
	} else {
		return false;
	}
}
void face_ahead(Mat &error,Mat &output) {
	output.at<double>(0, 0) = 0;
	output.at<double>(1, 0) = 0;
	output.at<double>(2, 0) = 0;
	output.at<double>(3, 0) = error.at<double>(3, 0);
}
Mat & go_ahead(Mat & error, double y_offset);
int main(int argc, char *argv[]) {
	PIDManager PID("pid.yaml");
	// AR.Drone class
	ARDrone ardrone;
	Mat error(4, 1, CV_64F);
	Mat move_dir(4, 1, CV_64F);
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
			} else {
				ardrone.landing();
			}
		}

		// Move
		double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
		if (key == 'i' || key == CV_VK_UP)    vx = 1.0;
		if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
		if (key == 'u' || key == CV_VK_LEFT)  vr = 1.0;
		if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
		if (key == 'j') vy = 1.0;
		if (key == 'l') vy = -1.0;
		if (key == 'q') vz = 1.0;
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
				//drawAxis会导致多个mark出BUG
				aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1);
				//tvecs的
				//2 是前后
				//1 是上下
				//0 是左右
				error.at<double>(0, 0) = tvecs[0][2];
				error.at<double>(1, 0) = tvecs[0][0];
				error.at<double>(2, 0) = tvecs[0][1];
				error.at<double>(3, 0) = rvecs[0][2];
				double tem = rvecs[0][0] * rvecs[0][2];
				if (tem >= 0) {
					error.at<double>(3, 0) = fabs(rvecs[0][2]);
				} else {
					error.at<double>(3, 0) = -fabs(rvecs[0][2]);
				}

				error.at<double>(3, 0) += 0.20;

				if (!find_mark(ids, move_dir)) {
					//do nothing 
				} else {
					face_ahead(error, move_dir);
				}

				vx = move_dir.at<double>(0, 0);
				vy = -move_dir.at<double>(1, 0);
				vz = -move_dir.at<double>(2, 0);
				vr = move_dir.at<double>(3, 0);
				//vr = 0;
				// VX是前后
				// VY是左右
				// VZ是上下
				cout << error << endl;
				cout << move_dir << endl;
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

Mat & go_ahead(Mat & error, double y_offset) {
	Mat output = Mat::zeros(4, 1, CV_64F);
	error.copyTo(output);
	output.at<double>(0, 1) -= y_offset;
}
