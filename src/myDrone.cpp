#include "myDrone.h"
#define SPEED 0.6666666666

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
	//--------------------------------------------
	//face dect
	String face_cascade_name = "haarcascade_frontalface_alt.xml";
	if (!face_cascade.load(face_cascade_name)) {
		printf("load classifier error\n");
	}
	//**********************************************
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
					_markIndex = getMarkerID(11);
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
				cout << "************** " << waitTimeFaceMarker3 << endl;
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
					setMoveDir(0.2, -0.1*lastFiveError.back()[1], 0, 0);
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
				setMoveDir(0, 0, 0, -0.2);
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

void myDrone::run_final() {
	int stage_time = 0;
	int times = 0;
	bool flag;
	int waitTimeFaceMarker3 = 1000 / 33;
	int waitTimeFaceMarker4 = 1000 / 33;
	bool tmp_flag;
	_stage = -99;
	while (1) {
		_image = _ardrone.getImage();
		times++;
		setMoveDir(0, 0, 0, 0);
		switch (_stage) {
		case(0):
			if (detectMark()) {
				if (getMarkerID(11) != -1) {   //see mark1
					_stage = -1;
				} else {
					setMoveDir(0, 0, 0, -0.1);
				}
			} else {
				setMoveDir(0, 0, 0, -0.1);
			}
			break;
		case (-1):

			if (detectMark()) {
				if (getMarkerID(11) != -1) {
					_markIndex = getMarkerID(11);
					getError(_markIndex);
					if (face_ahead()) {
						_stage = 1;
						break;
					} else {
						setMoveDir(0, 0, 0, 0.2*_error.at<double>(3, 0));
					}
				}

			} else {
				_stage = 0;
				lastFiveError.clear();
				setMoveDir(0, 0, 0, 0);
			}
			break;
		case(1):// see LBJ
			tmp_flag = detectMark();
			if (times % 3 == 0 && dectFace()) {
				if (_pic_size > 70) {
					cout << "start turn right" << endl;
					stage_time = 2000 / 33;
					_stage = 2;
				} else {
					if (tmp_flag) {
						//_markIndex = 0;
						getError(0);
						setMoveDir(SPEED, 0, 0, 0);
						//setMoveDir(0.2, 0, 0, 0.2*_error.at<double>(3, 0));
					} else {
						setMoveDir(SPEED, 0, 0, 0);
					}
				}
			} else {
				if (tmp_flag) {
					//_markIndex = 0;
					if (getMarkerID(2) != -1) {
						_markIndex = getMarkerID(2);
						getError(_markIndex);
						if (_error.at<double>(0, 0) < 2.0) {
							_stage = 6;
							break;
						}

					}
					getError(0);
					//setMoveDir(0.2, 0, 0, 0.2*_error.at<double>(3, 0));
					setMoveDir(SPEED, 0, 0, 0);
				} else {
					setMoveDir(SPEED, 0, 0, 0);
				}
			}
			break;
		case(2)://shift right 1 sec
			if (stage_time > 0) {
				stage_time--;
				if (stage_time > 1000 / 33) {
					if (stage_time > 1600 / 33) {
						setMoveDir(-0.8, 0, 0, 0);
					} else {
						setMoveDir(0, 0, 0, 0);
					}
				} else {
					setMoveDir(0, -1.0, 0, 0);
				}

			} else {
				_stage = 3;
				stage_time = 3000 / 33;
				cout << "start go ahead " << endl;
			}
			break;
		case(3)://go ahead 2 sec
			if (stage_time > 0) {
				stage_time--;
				if (stage_time > 1300 / 33) {
					setMoveDir(0, 0, 0, 0);
				} else {
					setMoveDir(0.8, 0, 0, 0);
				}
			} else {
				_stage = 4;
				stage_time = 2500 / 33;
				cout << "start turn left " << endl;

			}
			break;
		case(4)://shift left 1 sec 
			if (stage_time > 0) {
				stage_time--;
				if (stage_time > 1500 / 33) {
					setMoveDir(0, 0, 0, 0);
				} else {
					setMoveDir(0, 0.5, 0, 0);
				}
			} else {
				_stage = 5;
			}
			break;
			//****************************************************
		case(5)://go ahead,until see mark two
			if (detectMark() && getMarkerID(2) != -1) {
				_markIndex = getMarkerID(2);
				_stage = 6;
			} else {
				setMoveDir(0.4, 0, 0, 0);
			}
			break;
		case(6):// set mark 2 
			setMoveDir(0, 0, 0, 0);
			flag = go_head(1.0, 2);
			if (flag) {
				_stage = 7;
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
		case(7)://turn around untill see mark three
			if (detectMark() && getMarkerID(3) != -1) {
				_markIndex = getMarkerID(3);
				_stage = 8;

			} else {
				setMoveDir(0, 0, 0, -0.08);

			}
			break;
		case(8)://face to mark three
			if (!detectMark() || getMarkerID(3) == -1) {
				_stage = 7;
				setMoveDir(0, 0, 0, 0);
				break;
			} else {
				_markIndex = getMarkerID(3);
				if (face_ahead()) {
					_stage = 9;
					setMoveDir(0, 0, 0, 0);
					break;
				}
			}
			break;
		case(9)://go ahead untill distance =1;
				/*
				if (waitTimeFaceMarker3 != 0) {
				waitTimeFaceMarker3 -= 1;
				setMoveDir(0, 0, 0, 0);
				cout << "************** " << waitTimeFaceMarker3 << endl;
				move();
				break;
				}
				*/
			setMoveDir(0, 0, 0, 0);
			flag = go_head(1.0, 3);
			if (flag) {
				_stage = 10;
				setMoveDir(0, 0, 0, 0);
				break;
			}
			if (!detectMark() || getMarkerID(3) == -1) {
				printf("aaaaaaaaaaaaaaaaaaaaa\n");
				if (lastFiveError.back()[0] > 1.8) {
					setMoveDir(0.05*(lastFiveError.back()[0] - 1), 0, 0, 0);
				} else {
					setMoveDir(-0.2, -0.1*lastFiveError.back()[1], 0, 0);
				}
			}
			break;
		case(10):// turn around untill see mark 21;
			if (detectMark() && getMarkerID(21) != -1) {
				_markIndex = getMarkerID(21);
				_stage = 11;
			} else {
				setMoveDir(0, 0, 0, -0.08);
			}
			break;
		case (11)://face to mark 21
			if (detectMark()) {
				if (getMarkerID(21) != -1) {
					_markIndex = getMarkerID(21);
					getError(_markIndex);
					if (face_ahead()) {
						_stage = 12;
						break;
					} else {
						setMoveDir(0, 0, 0, 0.2*_error.at<double>(3, 0));
					}
				} else {
					_stage = 10;
				}

			} else {
				_stage = 10;
				lastFiveError.clear();
				setMoveDir(0, 0, 0, 0);
			}
			break;
		case(12):// see LBJ
			tmp_flag = detectMark();
			if (times % 5 == 0 && dectFace()) {
				if (_pic_size > 70) {
					cout << "start turn right" << endl;
					stage_time = 1000 / 33;
					_stage = 13;
				} else {
					if (tmp_flag) {
						//_markIndex = 0;
						getError(0);
						setMoveDir(0.3, 0, 0, 0.2*_error.at<double>(3, 0));
					} else {
						setMoveDir(0.3, 0, 0, 0);
					}
				}
			} else {
				if (tmp_flag) {
					//_markIndex = 0;
					if (getMarkerID(4) != -1) {
						_markIndex = getMarkerID(4);
						getError(_markIndex);
						if (_error.at<double>(0, 0) < 2.0) {
							_stage = 16;
							break;
						}
					}
					getError(0);
					setMoveDir(0.3, 0, 0, 0.2*_error.at<double>(3, 0));
				} else {
					setMoveDir(0.3, 0, 0, 0);
				}
			}
			break;
		case(13)://shift right 1 sec
			if (stage_time > 0) {
				stage_time--;
				setMoveDir(0, -0.3, 0, 0);
			} else {
				_stage = 14;
				stage_time = 3000 / 33;
				cout << "start go ahead " << endl;
			}
			break;
		case(14)://go ahead 2 sec
			if (stage_time > 0) {
				stage_time--;
				if (stage_time > 2000 / 33) {
					setMoveDir(0, 0, 0, 0);
				} else {
					setMoveDir(0.5, 0, 0, 0);
				}
			} else {
				_stage = 15;
				stage_time = 2500 / 33;
				cout << "start turn left " << endl;

			}
			break;
		case(15)://shift left 1 sec 
			if (stage_time > 0) {
				stage_time--;
				if (stage_time > 1500 / 33) {
					setMoveDir(0, 0, 0, 0);
				} else {
					setMoveDir(0, 0.5, 0, 0);
				}
			} else {
				_stage = 16;
			}
			break;
			//****************************************************
		case(16)://go ahead,until see mark four              need to refer to marker2X
			if (detectMark() && getMarkerID(4) != -1) {
				_markIndex = getMarkerID(4);
				_stage = 17;
			} else {
				setMoveDir(0.4, 0, 0, 0);
			}
			break;
		case(17):// set mark four
			setMoveDir(0, 0, 0, 0);
			flag = go_head(1.0, 4);
			if (flag) {
				_stage = 18;
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
		case (18):// prepare landing or can't find the mark 5;
			if (_mode % 2 == 0) {
				_mode++;
				_ardrone.setCamera(_mode % 4);
				_stage = 19;
			} else {
				setMoveDir(0, 0, 0, -0.2);
				_stage = 19;
			}
			break;
		case(19):// do landing;
			if (detectMark()) {
				if (getMarkerID(5) == -1) {//can't find mark 5
					_stage = 18;
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
		case(-10):
			setMoveDir(0, 0, 0, 0);
			break;
		default:
			break;
		}
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
	if (key == 't') _stage = 1;
	if (key == 'w') {
		last_stage = _stage;
		_stage = -10;
	}
	if (key == 'e') {
		_stage = last_stage;
	}
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
	if (fabs(_error.at<double>(3, 0)) < 0.03) {
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

		_move_dir.at<double>(1, 0) = -0.11*_error.at<double>(1, 0);//modify todo 0.2->0.1
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

bool myDrone::dectFace(void) {
	Mat img_gray;
	cvtColor(_image, img_gray, CV_BGR2GRAY);
	equalizeHist(img_gray, img_gray);

	vector<Rect> faces;
	face_cascade.detectMultiScale(img_gray, faces, 1.1, 3, 0, Size(20, 20));
	if (faces.size() > 0) {
		_pic_size = 0;
		for (size_t i = 0; i < faces.size(); i++) {
			if (faces[i].height > 0 && faces[i].width > 0) {
				rectangle(_image, faces[i], Scalar(0, 0, 255), 3, 8, 0);
			}
			cout << faces[i] << endl;
			_pic_size = max(_pic_size, faces[i].height);
		}
		return true;
	} else {
		return false;
	}

}
