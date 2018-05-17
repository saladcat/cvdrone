#include "myDrone.h"

myDrone::myDrone() :_mode(0), _stage(-1), _error(4, 1, CV_64F), _move_dir(4, 1, CV_64F) {
	PIDManager PID("pid.yaml");
	if (!_ardrone.open()) {
		std::cout << "Failed to initialize." << std::endl;
	}
	FileStorage fs("camera.xml", FileStorage::READ);
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;
	_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	std::cout << "Battery = " << _ardrone.getBatteryPercentage() << "[%]" << std::endl;

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
}

myDrone::~myDrone() {
	_ardrone.landing();
	_ardrone.close();
} 

void myDrone::do_run() {
	bool flag;
	int waitTimeFaceMarker3 = 1000 / 33;
	int waitTimeFaceMarker4 = 1000 / 33;
	while (1) {
		_image = _ardrone.getImage();
		switch (_stage) {
		case (0):
			if (detectMark()) {
				/*if (getMarkerID(2) != -1) {     //see mark2
					_stage = 2;
				} else */
				if (getMarkerID(1) != -1) {   //see mark1
					_stage = 1;
				} else {
					setMoveDir(0, 0, 0, -0.1);
				}
			} else {
				setMoveDir(0, 0, 0, -0.1);
			}
			break;
		case (1):
			if (detectMark()) {
				if (getMarkerID(1) != -1) {
					_markIndex = getMarkerID(1);
					getError(_markIndex);
					if (face_ahead()) {
						_stage = 2;
						break;
					} else {
						setMoveDir(0, 0, 0, 0.2*_error.at<double>(3, 0));
					}
				}
				if (getMarkerID(2) != -1) {
					_markIndex = getMarkerID(2);
					_stage = 2;
				}

			} else {
				_stage = 0;
				lastFiveError.clear();
				setMoveDir(0, 0, 0, 0);
			}
			break;
		case(2)://go ahead,until see mark two
			if (detectMark() && getMarkerID(2) != -1) {
				_markIndex = getMarkerID(2);
				_stage = 3;
			} else {
				setMoveDir(0.4, 0, 0, 0);
			}
			break;
		case(3)://see mark two,go until distance equal to 1 meter
			setMoveDir(0, 0, 0, 0);
			flag = go_head(1.0, 2);
			if (flag) {
				_stage = 4;
				setMoveDir(0, 0, 0, 0);
			}
			if (!detectMark()) {
				if (lastFiveError.back()[0] > 1.8) {
					setMoveDir(0.05*(lastFiveError.back()[0] - 1), 0, 0, 0);
				} else {
					setMoveDir(-0.2, -0.1*lastFiveError.back()[1], 0, 0);
				}
			}

			break;
		case(4)://turn around untill see mark three
			if (detectMark() && getMarkerID(3) != -1) {
				_markIndex = getMarkerID(3);
				_stage = 5;

			} else {
				setMoveDir(0, 0, 0, -0.08);

			}
			break;
		case(5)://face to mark three
			if (detectMark() && getMarkerID(3) == -1) {
				_stage = 4;
				setMoveDir(0, 0, 0, 0);
				break;
			} else {
				_markIndex = getMarkerID(3);
				if (_markIndex != -1) {
					if (face_ahead()) {
						_stage = 6;
						break;
					} else {
						setMoveDir(0, 0, 0, 0.1*_error.at<double>(3, 0));
					}
				} else {
					setMoveDir(0.2, 0, 0, 0);
				}
			}
			break;
		case(6)://go ahead untill distance =1;
			if (waitTimeFaceMarker3 != 0) {
				waitTimeFaceMarker3 -= 1;
				setMoveDir(0, 0, 0, 0);
				cout << "************** " <<waitTimeFaceMarker3<< endl;
				move();
				break;
			}
			setMoveDir(0, 0, 0, 0);
			flag = go_head(1.0, 3);
			if (flag) {
				_stage = 7;
				setMoveDir(0, 0, 0, 0);
			}
			if (!detectMark()) {
				printf("aaaaaaaaaaaaaaaaaaaaa\n");
				if (lastFiveError.back()[0] > 1.8) {
					setMoveDir(0.05*(lastFiveError.back()[0] - 1), 0, 0, 0);
				} else {
					setMoveDir(-0.2, -0.1*lastFiveError.back()[1], 0, 0);
				}
			}
			break;
		case(7):// turn around untill see mark four;
			if (detectMark() && getMarkerID(4) != -1) {
				_markIndex = getMarkerID(4);
				_stage = 8;
			} else {
				setMoveDir(0, 0, 0, -0.08);
			}
			break;
		case (8)://face to mark four 
			if (detectMark() && getMarkerID(4) == -1) {
				_stage = 7;
				setMoveDir(0, 0, 0, 0);
				break;
			} else {
				_markIndex = getMarkerID(4);
				if (_markIndex != -1) {
					if (face_ahead()) {
						_stage = 9;
						break;
					} else {

						setMoveDir(0, 0, 0, 0.1*_error.at<double>(3, 0));
					}
				} else {
					setMoveDir(0.2, 0, 0, 0);
				}
			}
			break;
		case(9)://go ahead untill distance =1;
			if (waitTimeFaceMarker4 != 0) {
				waitTimeFaceMarker4 -= 1;
				setMoveDir(0, 0, 0, 0);
				move();
				break;
			}
			setMoveDir(0, 0, 0, 0);
			flag = go_head(1.0, 4);
			if (flag) {
				_stage = 10;
				setMoveDir(0, 0, 0, 0);
			}
			if (!detectMark()) {
				if (lastFiveError.back()[0] > 1.8) {
					setMoveDir(0.05*(lastFiveError.back()[0] - 1), 0, 0, 0);
				} else {
					setMoveDir(-0.2, 0, 0, 0);
				}
			}
			break;
		case (10):// prepare landing or can't find the mark 5;
			if (_mode % 2 == 0) {
				_mode++;
				_ardrone.setCamera(_mode % 4);
				_stage = 11;
			} else {
				setMoveDir(0, 0, 0.4, 0.5);
				_stage = 11;
			}
			break;
		case(11):// do landing;
			if (detectMark()) {
				if (getMarkerID(5) == -1) {//can't find mark 5
					_stage = 10;
				} else {
					_markIndex = getMarkerID(5);
					getError(_markIndex);
					double temz = _error.at<double>(0, 0);
					double vx = -0.8*temz* _error.at<double>(2, 0);
					double vy = -1.6*temz*_error.at<double>(1, 0);
					double vz = 0;
					double vr = 0;
					if (fabs(_error.at<double>(1, 0)) < 0.1 * 2 && fabs(_error.at<double>(2, 0)) < 0.06 * 2) {
						vx = 0;
						vy = 0;
						vz = -0.2;
						if (_error.at<double>(0, 0) < 0.85) {
							_ardrone.landing();
						}
					}
					setMoveDir(vx, vy, vz, vr);//todo
				}
			} else {
				setMoveDir(0, 0, 0, -0.2);
			}
			break;
		default:
			cout << "waiting " << endl;
		}
		//save error
		cout << _stage << endl;
		if (_markIndex != -1) {
			if (lastFiveError.size() > 5) {
				lastFiveError.pop_front();
			}
			vector<double> temError;
			temError.push_back(_error.at<double>(0, 0));
			temError.push_back(_error.at<double>(1, 0));
			temError.push_back(_error.at<double>(2, 0));
			temError.push_back(_error.at<double>(3, 0));
			lastFiveError.push_back(temError);
		}

		getInput();
		move();
		cv::imshow("camera", _image);
	}
}

void myDrone::setMoveDir(double vx, double vy, double vz, double vr) {
	_move_dir.at<double>(0, 0) = vx;
	_move_dir.at<double>(1, 0) = vy;
	_move_dir.at<double>(2, 0) = vz;
	_move_dir.at<double>(3, 0) = vr;
}

int myDrone::getMarkerID(int ID) {
	for (int index = 0; index < _ids.size(); index++) {
		if (ID == _ids[index]) {
			return index;
		}
	}
	return -1;
}

bool myDrone::getInput(void) {
	int key = cv::waitKey(33);
	if (key == 0x1b) {
		delete this;//todo 
		return false;
	}
	if (key == ' ') {
		if (_ardrone.onGround()) {
			if (_stage == -1) {
				_stage = 0;
			}
			_ardrone.takeoff();
		} else {
			_ardrone.landing();
		}
	}

	double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	if (key == 'i' || key == CV_VK_UP)    vx = 1.0;
	if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
	if (key == 'u' || key == CV_VK_LEFT)  vr = 1.0;
	if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
	if (key == 'j') vy = 1.0;
	if (key == 'l') vy = -1.0;
	if (key == 'q') vz = 1.0;
	if (key == 'a') vz = -1.0;
	if (key == 'c') _ardrone.setCamera(++_mode % 4);

	if (key != -1) {
		setMoveDir(vx, vy, vz, vr);
		return true;
	} else {
		return false;
	}
}

void myDrone::move(void) {
	_ardrone.move3D(
		_move_dir.at<double>(0, 0),
		_move_dir.at<double>(1, 0),
		_move_dir.at<double>(2, 0),
		_move_dir.at<double>(3, 0));
	if (!_ardrone.onGround()) {
		printf("%f, %f, %f, %f \n", _move_dir.at<double>(0, 0), _move_dir.at<double>(1, 0),
			_move_dir.at<double>(2, 0),
			_move_dir.at<double>(3, 0));
	}
}

bool myDrone::changeCamera(void) {
	if (_mode % 2 == 0) {
		_mode++;
		return true;
	} else {
		_mode++;
		return false;
	}
}

bool myDrone::detectMark() {
	cv::aruco::detectMarkers(_image, _dictionary, _corners, _ids);
	if (_ids.size() > 0) {
		cv::aruco::estimatePoseSingleMarkers(_corners, markerLength, cameraMatrix, distCoeffs, _rvecs, _tvecs);
		return true;
	} else {
		return false;
	}
}
void myDrone::getError(int makerIndex) {
	cv::aruco::estimatePoseSingleMarkers(_corners, markerLength, cameraMatrix, distCoeffs, _rvecs, _tvecs);
	//tvecs的
	//2 是前后
	//1 是上下
	//0 是左右
	_error.at<double>(1, 0) = _tvecs[_markIndex][0];
	_error.at<double>(2, 0) = _tvecs[_markIndex][1];
	_error.at<double>(0, 0) = _tvecs[_markIndex][2];
	_error.at<double>(3, 0) = _rvecs[_markIndex][2];
	double tem = _rvecs[_markIndex][0] * _rvecs[_markIndex][2];
	if (tem >= 0) {
		_error.at<double>(3, 0) = fabs(_rvecs[_markIndex][2]);
	} else {
		_error.at<double>(3, 0) = -fabs(_rvecs[_markIndex][2]);
	}
	_error.at<double>(3, 0) += 0.20;//todo 

	return;

}

bool myDrone::face_ahead(void) {
	getError(_markIndex);
	if (fabs(_error.at<double>(3, 0)) < 0.05) {
		return true;
	} else {
		setMoveDir(0, 0, 0, _error.at<double>(3, 0));
		return false;
	}
}

bool myDrone::go_head(double dist, int id) {

	bool flag = true;
	if (detectMark() && ((_markIndex = getMarkerID(id)) != -1)) {
		getError(_markIndex);
		double temVX = _error.at<double>(0, 0) - dist;

		if (fabs(temVX) < 0.08) {
			temVX = 0;
		}
		setMoveDir(0, 0, 0, 0);
		_move_dir.at<double>(0, 0) = 0.17*temVX;//without using setMoveDir() 'cause face_ahead //modify todo 0.2->0.1
		_move_dir.at<double>(3, 0) = _error.at<double>(3, 0);

		_move_dir.at<double>(1, 0) = -0.10*_error.at<double>(1, 0);//modify todo 0.2->0.1
//		if (_move_dir.at<double>(1, 0) <= 0.01)
//			_move_dir.at<double>(1, 0) = 0;
		cout << _error.at<double>(0, 0) << endl;
		cout << _error.at<double>(1, 0) << endl;

		if (fabs(_error.at<double>(0, 0) - dist) > 0.08)
			flag = false;
		for (auto &item : lastFiveError) {
			if (fabs(item[0] - dist) > 0.1) {
				flag = false;
			}
		}
		return flag;
	} else {
		return false;
	}
}
