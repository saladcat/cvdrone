#pragma once
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
#include <list>
using namespace std;
using namespace cv;
const double markerLength = 0.094;
class myDrone {
public:
	myDrone();
	~myDrone();
	void do_run();
	ARDrone _ardrone;

private:
	int _stage;
	int _markIndex;
	Mat _image;
	Mat _move_dir;
	Mat _error;
	vector<int> _ids;
	vector<std::vector<cv::Point2f>> _corners;
	vector<Vec3d> _rvecs, _tvecs;
	Mat cameraMatrix, distCoeffs;
	cv::aruco::Dictionary _dictionary;
	list<vector<double>> lastFiveError;
	int _mode;
	//set move_dir
	//vx,vy,vz,vr
	void setMoveDir(double vx, double vy,double vz,double vr);
	//get the index in ids by the ID of marker you want 
	//if there existn't ret -1;
	int getMarkerID(int ID);
	//get input from keyboard and change the move_dir
	//if there are no input, return false;
	bool getInput(void);
	//move follow move_dir;
	void move(void);
	//now front->botton ret true;
	//now botton->front ret false;
	bool changeCamera(void);
	//detect marker
	//if there exsit return true, otherwise false;
	bool detectMark();
	//if it's facing front return true,otherwise change move_dir to face and return false;
	bool face_ahead(void);
	//if (fact_distance - dist)<0.2 for all error in last 5 errors, return true;
	bool go_head(double dist, int id);

	void getError(int markerIndex);
	
};

