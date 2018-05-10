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
#include"myDrone.h"
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
	myDrone mydrone;
	mydrone.do_run();

	return 0;
}
