#include "Header.hpp"

RobotSensing::RobotSensing(string leftMotor, string rightMotor,
	string frontDist1, string frontDist2,
	string leftDist1, string leftDist2,
	string rightDist1, string rightDist2,
	string backDist1, string backDist2,
	string colorSensorName, string leftCamera, string rightCamera,
	string inertialName, string gpsName, string lidarName,
	string emitterName, string receiverName)
{
	robot = new Robot();

	timeStep = (int)robot->getBasicTimeStep();
	cout << "timeStep " << timeStep << endl;

	//Motors
	lMotor = robot->getMotor(leftMotor);
	rMotor = robot->getMotor(rightMotor);
	//Distance Sensors
	fDist1 = robot->getDistanceSensor(frontDist1);
	fDist2 = robot->getDistanceSensor(frontDist2);
	lDist1 = robot->getDistanceSensor(leftDist1);
	lDist2 = robot->getDistanceSensor(leftDist2);
	rDist1 = robot->getDistanceSensor(rightDist1);
	rDist2 = robot->getDistanceSensor(rightDist2);
	bDist1 = robot->getDistanceSensor(backDist1);
	bDist2 = robot->getDistanceSensor(backDist2);

	//Color Sensor + Cameras
	colorSensor = robot->getCamera("colourSensor");
	lCam = robot->getCamera(leftCamera);
	rCam = robot->getCamera(rightCamera);
	//Inertial+GPS+Lidar
	inertial = robot->getInertialUnit(inertialName);
	gps = robot->getGPS(gpsName);

	//Emitter+Receiver
	emitter = robot->getEmitter(emitterName);
	receiver = robot->getReceiver(receiverName);

	cout << "Lidar name is " << lidarName << endl;

	lidar = robot->getLidar(lidarName);

	lMotor->setPosition(INFINITY);
	rMotor->setPosition(INFINITY);
	lMotor->setVelocity(0.0);
	rMotor->setVelocity(0.0);

	colorSensor->enable(timeStep);
	rCam->enable(timeStep);
	lCam->enable(timeStep);
	lDist1->enable(timeStep);
	lDist2->enable(timeStep);
	rDist1->enable(timeStep);
	rDist2->enable(timeStep);
	fDist1->enable(timeStep);
	fDist2->enable(timeStep);
	bDist1->enable(timeStep);
	bDist2->enable(timeStep);
	inertial->enable(timeStep);
	lidar->enable(timeStep);
	lidar->enablePointCloud();
	
	gps->enable(timeStep);
	receiver->enable(timeStep);

	getTimeStep();

	startX = gps->getValues()[0];
	startZ = gps->getValues()[2];
}

Coordinate RobotSensing::getCoords()
{
	Coordinate retCoord;
	retCoord.x = gps->getValues()[0];
	retCoord.y = gps->getValues()[1]; //actually z value
	retCoord.z = gps->getValues()[2]; //actually y value

	return	retCoord;
}

double RobotSensing::getDist(Direction dir) {

	double value;

	switch (dir)
	{
	case Up:
		value = std::min(fDist1->getValue(), fDist2->getValue());
		break;
	case Left:
		value = std::min(lDist1->getValue(), lDist2->getValue());
		break;
	case Right:
		value = std::min(rDist1->getValue(), rDist2->getValue());
		break;
	case Down:
		value = std::min(bDist1->getValue(), bDist2->getValue());
		break;
	default:
		value = -1;
		break;
	}

	if (value < 0)
	{
		return 0;
	}
	return value * 100; // cm
}

Color RobotSensing::getColor()
{
	const unsigned char* image = colorSensor->getImage();
	int r = colorSensor->imageGetRed(image, colorSensor->getWidth(), colorSensor->getWidth() / 2, colorSensor->getHeight() / 2);
	int g = colorSensor->imageGetGreen(image, colorSensor->getWidth(), colorSensor->getWidth() / 2, colorSensor->getHeight() / 2);
	int b = colorSensor->imageGetBlue(image, colorSensor->getWidth(), colorSensor->getWidth() / 2, colorSensor->getHeight() / 2);
	//cout << r << "  " << g << "  " << b << endl;
	if ((r > 150) && (g > 150) && (b > 150))
	{
		return White;
	}
	else if ((r < 60) && (g < 60) && (b < 60))
	{
		return Black;
	}
	else if ((r > 230) && (g < 50) && (b < 50))
	{
		return Red;
	}
	else if ((r < 50) && (g > 230) && (b < 50))
	{
		return Green;
	}
	else if ((r < 50) && (g < 50) && (b > 230))
	{
		return Blue;
	}
	else if ((r > 100) && (g < 50) && (b > 180))
	{
		return Purple;
	}
	else if ((r > 80) && (g > 80) && (b > 80))
	{
		return Gray;
	}
	else if ((r > 150) && (g < 140) && (b < 80))
	{
		return Sand;
	}
	else
	{
		return NoColor;
	}


}

const char* RobotSensing::printColor(Color col)
{
	switch (col)
	{
	case White:
		return "White";
	case Black:
		return "Black";
	case Red:
		return "Red";
	case Blue:
		return "Blue";
	case Purple:
		return "Purple";
	case Gray:
		return "Gray";
	case Sand:
		return "Sand";
	case NoColor:
		return "Unknown Color";
	}
}

signs_and_victims RobotSensing::getSign(Direction dir)
{
	Camera* cam;
	switch (dir)
	{
	case Left:
		cam = lCam;
		break;
	case Right:
		cam = rCam;
		break;
	default:
		return NoSign;
	}
	bool match_found = 0;
	Mat frame(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage());
	cvtColor(frame, frame, COLOR_BGR2GRAY);
	//thresholding
	threshold(frame, frame, 0, 255, 1);
	//contours and roi
	vector<vector<Point>> contours;
	findContours(frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	vector<Vec4i> hierarchy;
	vector<Rect> boundRect(contours.size());
	int i;
	Mat roi;
	auto redcolor = Scalar(0, 0, 255);
	Mat frame_rgb(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage()), frame_hsv, thresholded_img;
	vector<vector<Point>> contours_hsv;
	cvtColor(frame_rgb, frame_hsv, COLOR_BGR2HSV);
	if (match_found == 0)
	{
		inRange(frame_hsv, Scalar(25, 127, 127), Scalar(40, 255, 255), thresholded_img); //yellow (organic peroxide)
		findContours(thresholded_img, contours_hsv, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (i = 0; i < contours_hsv.size(); i++)
		{
			if (contourArea(contours_hsv[i]) > 50.0)
			{
				match_found = 1;
				return Peroxide;
			}
		}
	}
	if (match_found == 0)
	{
		inRange(frame_hsv, Scalar(160, 0, 0), Scalar(180, 255, 255), thresholded_img); //red (flammable gas)
		findContours(thresholded_img, contours_hsv, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (i = 0; i < contours_hsv.size(); i++)
		{
			if (contourArea(contours_hsv[i]) > 5.0)
			{
				match_found = 1;
				return Flammable;
			}
		}
	}
	if (match_found == 0)
	{
		inRange(frame_hsv, Scalar(0, 0, 0), Scalar(0, 0, 0), thresholded_img); //black (corrosive)
		findContours(thresholded_img, contours_hsv, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours_hsv.size(); i++)
		{
			if (contourArea(contours_hsv[i]) > 25.0)
			{
				inRange(frame_hsv, Scalar(0, 0, 175), Scalar(0, 0, 255), thresholded_img); //white
				findContours(thresholded_img, contours_hsv, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				for (i = 0; i < contours_hsv.size(); i++)
				{
					if (contourArea(contours_hsv[i]) < 10.0)
					{
						match_found = 1; //figure out corrosive conditon
						return Corrosive;
					}
				}
			}
		}
	}
	if (match_found == 0)
	{
		for (i = 0; i < contours.size(); i++)
		{
			boundRect[i] = boundingRect(contours[i]);
			roi = Mat(frame, boundRect[i]); 
		}
		int height = roi.rows;
		int width = roi.cols;

		Rect toprect(0, 0, width, height / 3);
		Rect midrect(0, height / 3, width, height / 3);
		Rect botrect(0, 2 * height / 3, width, height / 3);

		Mat topRoi(roi, toprect);
		Mat midRoi(roi, midrect);
		Mat botRoi(roi, botrect);

		vector<vector<Point>> subContours;

		findContours(topRoi, subContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		int numTopContours = subContours.size();
		subContours.clear();
		findContours(midRoi, subContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		int numMidContours = subContours.size();
		subContours.clear();
		findContours(botRoi, subContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		int numBotContours = subContours.size();
		if ((numTopContours == 2) && (numMidContours == 2) && (numBotContours == 1))
		{
			return U;
			match_found = 1;
		}
		else if ((numTopContours == 1) && (numMidContours == 1) && (numBotContours == 1))
		{
			return S;
			match_found = 1;
		}
		else if ((numTopContours == 2) && (numMidContours == 1) && (numBotContours == 2))
		{
			return H;
			match_found = 1;
		}
		if (match_found == 0)
		{
			inRange(frame_hsv, Scalar(0, 0, 200), Scalar(0, 0, 255), thresholded_img); //white/gray (poison)
			findContours(thresholded_img, contours_hsv, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			for (i = 0; i < contours_hsv.size(); i++)
			{
				if (contourArea(contours_hsv[i]) > 5.0)
				{
					match_found = 1;
					return Poison;
				}
			}
		}
		}	
}


const char* RobotSensing::printSign(signs_and_victims hazard)
{
	switch (hazard)
	{
	case Flammable:
		return "Flammable";
	case Poison:
		return "Poison";
	case Corrosive:
		return "Corrosive";
	case Peroxide:
		return "Peroxide";
	case H:
		return "H";
	case S:
		return "S";
	case U:
		return "U";
	default:
		return "Unknown Sign";
	}
}


//char RobotSensing::getLetter(Direction dir)
//{
//	Camera* cam;
//	switch (dir)
//	{
//	case Left:
//		cam = lCam;
//		break;
//	case Right:
//		cam = rCam;
//		break;
//	default:
//		return '0';
//	}
//	//Unfinished. Will change types, add return values, and in general modify the function later on.
//		//Convert frame to grayscale
//	Mat frame(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage());
//	cvtColor(frame, frame, COLOR_BGR2GRAY);
//	//thresholding
//	threshold(frame, frame, 0, 255, 1);
//	//contours and roi
//	vector<vector<Point>> contours;
//	findContours(frame, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
//	vector<Vec4i> hierarchy;
//	vector<Rect> boundRect(contours.size());
//	int i;
//	Mat roi;
//	auto redcolor = Scalar(0, 0, 255);
//	for (i = 0; i < contours.size(); i++)
//	{
//		boundRect[i] = boundingRect(contours[i]);
//		roi = Mat(frame, boundRect[i]);
//		imshow("roi", roi);
//		waitKey(3);
//	}
//
//	int height = roi.rows;
//	int width = roi.cols;
//
//	Rect toprect(0, 0, width, height / 3);
//	Rect midrect(0, height / 3, width, height / 3);
//	Rect botrect(0, 2 * height / 3, width, height / 3);
//
//	Mat topRoi(roi, toprect);
//	Mat midRoi(roi, midrect);
//	Mat botRoi(roi, botrect);
//
//	vector<vector<Point>> subContours;
//
//	findContours(topRoi, subContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//	int numTopContours = subContours.size();
//	subContours.clear();
//	findContours(midRoi, subContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//	int numMidContours = subContours.size();
//	subContours.clear();
//	findContours(botRoi, subContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//	int numBotContours = subContours.size();
//
//	cout << "Top Cont.: " << numTopContours << endl;
//	cout << "Mid Cont.: " << numMidContours << endl;
//	cout << "Bot Cont.: " << numBotContours << endl;
//
//	if ((numTopContours == 2) && (numMidContours == 2) && (numBotContours == 1))
//		return 'U';
//	else if ((numTopContours == 1) && (numMidContours == 1) && (numBotContours == 1))
//		return 'S';
//	else if ((numTopContours == 2) && (numMidContours == 1) && (numBotContours == 2))
//		return 'H';
//	else
//		return '0';
//}

int RobotSensing::getTimeStep()
{
	return robot->step(timeStep);
}



void RobotSensing::delay(int ms)
{
	// get the start time
	float init = robot->getTime();
	while (robot->step(timeStep) != -1)
	{
		if ((robot->getTime() - init) * 1000 > ms)
			break;
	}
}

double RobotSensing::getYaw() {
	return inertial->getRollPitchYaw()[2];
}

// finds difference between two angles
double angleDiff(double angle, double target) {
	double out = angle - target;
	if (out > 180) out -= 360;
	else if (out < -180) out += 360;
	return out;
}

double degToRad(double degrees) {
	return degrees * (PI * 2) / 360;
}

double radToDeg(double radians) {
	return radians * 360 / (PI * 2);
}

// clamps the value between min and max
// if in is negative, between -min and -max
double clampMagnitude(double in, double min, double max) {
	if (in < 0) {
		if (in > -min) in = -min;
		else if (in < -max) in = -max;
	}
	else {
		if (in < min) in = min;
		else if (in > max) in = max;
	}
	return in;
}

void RobotSensing::turn(double degrees) {
	// round degrees 
	double yaw = radToDeg(getYaw());
	double nearest90 = round(yaw / 90) * 90;
	double target = nearest90 + degrees;

	const double thresh = 0.001, min = 0.0;
	const double kp = 0.73, kd = 0.6;

	double error = angleDiff(yaw, target);
	double prevError = error;

	while (robot->step(1) != -1) {
		double val = pid(prevError, error, kp, kd);
		prevError = error;
		error = angleDiff(radToDeg(getYaw()), target);

		if (error < thresh && error > -thresh) break;

		lMotor->setVelocity(clampMagnitude(val, min, maxSpeed));
		rMotor->setVelocity(clampMagnitude(-val, min, maxSpeed));
	}

	lMotor->setVelocity(0.0);
	rMotor->setVelocity(0.0);
}

StraightReturn RobotSensing::straight(const int tiles, Maze &maze, bool checkBlackHole) {
	int fwd = tiles > 0 ? 1 : -1;
	const double tileSize = 0.12;
	Coordinate coords = getCoords();
	int dir = round(getYaw() / (PI / 2));

	// values to multiply by
	int x = 0, z = 0;
	switch (dir) {
		// 180
	case -2: case 2: z = 1; break;
		// 0
	case 0: z = -1; break;
		// 90
	case 1: x = -1; break;
		// -90
	case -1: x = 1; break;
	default: cerr << "error RobotFuncs.cpp:419";
	}
	double xTarget = coords.x + tileSize * tiles * x;
	double zTarget = coords.z + tileSize * tiles * z;
	double targetAngle = round(radToDeg(getYaw()) / 90) * 90;
	// round to the nearest tile center
	xTarget = round((xTarget - startX) / tileSize) * tileSize + startX;
	zTarget = round((zTarget - startZ) / tileSize) * tileSize + startZ;

	const double thresh = 0.0005, min = 0.0;
	const double kp = 10.0, kd = 15.0;
	const double padding = 1;

	auto getError = [&] {
		if (z != 0) return (getCoords().z - zTarget) * z * 100.0;
		else if (x != 0) return (getCoords().x - xTarget) * x * 100.0;
		else cerr << "error RobotFuncs.cpp:431";
	};

	// major / minor just means where were facing vs the sides (major vs minor axis)
	// we want to correct for small deviations in the minor axis by turning slightly
	double error = getError();
	double prevError = error;

	while (robot->step(1) != -1) {
		// how fast to go forward
		double val = pid(prevError, error, kp, kd);
		double correction = - angleDiff(radToDeg(getYaw()), targetAngle);
		prevError = error;
		error = getError();

		//cout << "correction: " << correction << endl;

		if (error < thresh && error > -thresh) break;

		double l = clampMagnitude(-val, min, maxSpeed - padding) - correction;
		double r = clampMagnitude(-val, min, maxSpeed - padding) + correction;
		lMotor->setVelocity(clampMagnitude(l, min, maxSpeed));
		rMotor->setVelocity(clampMagnitude(r, min, maxSpeed));
		Pos p = dirToPos(maze.tracker.direction);
		if ((getColor() == Black) && checkBlackHole) {
			cout << "BLACK" << endl;
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].blackHole = true;
			cout << "Tile marked as black hole: ";
			cout << "(" << maze.tracker.y << ", " << maze.tracker.x << ")" << endl;
			straight(0, maze, false);
			return BlackHole;
		}
		if ((getColor() == Red) && checkBlackHole) {
			cout << "RED" << endl;
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].blackHole = true;
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].redTile = true;
			cout << "Tile marked as red tile: ";
			cout << "(" << maze.tracker.y << ", " << maze.tracker.x << ")" << endl;
			straight(0, maze, false);
			return BlackHole;
		}
		if (getColor() == Green) {
			cout << "GREEN" << endl;
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].greenTile = true;
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].blackHole = true;
			cout << "Tile marked as green tile: ";
			cout << "(" << maze.tracker.y << ", " << maze.tracker.x << ")" << endl;
			straight(0, maze, false);
			return BlackHole;
		}
		if (getColor() == Sand) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].sandTile = true;
			return SandTile;
		}
		if (getColor() == Blue) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].blueTile = true;
			return BlueTile;
		}
		if (getColor() == Purple) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].purpleTile = true;
			return PurpleTile;
		}
		if (getColor() == Gray) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].grayTile = true;
			return GrayTile;
		}
		/*
		if ((getSign(Left) == Flammable) || (getSign(Right) == Flammable)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].flammableSign = true;
			return Normal;
		}
		if ((getSign(Left) == Poison) || (getSign(Right) == Poison)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].poisonSign = true;
			return Normal;
		}
		if ((getSign(Left) == Corrosive) || (getSign(Right) == Corrosive)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].corrosiveSign = true;
			return Normal;
		}
		if ((getSign(Left) == Peroxide) || (getSign(Right) == Peroxide)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].peroxideSign = true;
			return Normal;
		}
		if ((getSign(Left) == H) || (getSign(Right) == H)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].HSign = true;
			return Normal;
		}
		if ((getSign(Left) == S) || (getSign(Right) == S)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].SSign = true;
			return Normal;
		}
		if ((getSign(Left) == U) || (getSign(Right) == U)) {
			maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].USign = true;
			return Normal;
		}*/
	}
	turn(0);
	return Normal;
}


//void RobotSensing::lidarFuncs()
//{
//	const float* rangeImage = lidar->getRangeImage(); // Step 4: Retrieve the range image
//	for (int i = 0; i < 10; i++) {
//
//		// Print the first 10 values of the range image.
//		// The range image stores the distances from left to right, from first to last layer
//		cout << rangeImage[i] << " ";
//	}
//}

void RobotSensing::transmission(char victim)
{
	Coordinate coords = getCoords();
	double x = coords.x * 100, y = coords.z * 100;
	int round_x = round(x), round_y = round(y);
	printf("%d, %d \n", round_x, round_y);
	char message[9];
	int victim_pos[2] = { round_x, round_y };
	memcpy(message, victim_pos, sizeof(victim_pos));
	delay(5000);
	message[8] = victim; 
	emitter->send(message, sizeof(message));
}

void RobotSensing::exit_maze()
{
	char message = 'E';
	emitter->send(&message, 1);
}


const LidarPoint* RobotSensing::lidarFuncs()
{

	const LidarPoint* lidarPoints = lidar->getPointCloud();

	LidarPoint layerValues[360];

	int layer = 2;

	for (int i = 0; i < 360; i++)
	{
		layerValues[i] = lidarPoints[layer * lidar->getHorizontalResolution() + i];
	}
	
	return layerValues; // Get the point cloud

	
	

}

void RobotSensing::submit_maze(Maze maze)
{
	const int width = 401, height = 401;
	char mazeArray[401][401], newMazeArray[401][401];
	int max_x = 50, max_y = 50, min_x = 50, min_y = 50;
	/*mazeArray[201][201] = '5';
	mazeArray[201][203] = '5';
	mazeArray[203][201] = '5';
	mazeArray[203][203] = '5';*/
	vector<vector<char>> mazeVect;
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 100; j++)
		{
			if (((4 * i + 4) <= 400) && ((4 * j + 4) <= 400))
			{
				if (maze.map[i][j].visited == true)
				{
					if (i > max_y)
						max_y = i;
					if (j > max_x)
						max_x = j;
					if (i < min_y)
						min_y = i;
					if (j < min_x)
						min_x = j;
					//mazeArray[4 * i + 1][4 * j + 2] = '0';
					//mazeArray[4 * i + 2][4 * j + 1] = '0';
					//mazeArray[4 * i + 2][4 * j + 3] = '0';
					//mazeArray[4 * i + 3][4 * j + 2] = '0';
					//if (maze.map[i][j].right == Wall)
					//{
					//	mazeArray[4 * i + 1][4 * j + 4] = '1';
					//	mazeArray[4 * i + 2][4 * j + 4] = '1';
					//	mazeArray[4 * i + 3][4 * j + 4] = '1';
					//}
					//else if (maze.map[i][j].right == Empty)
					//{
					//	mazeArray[4 * i + 1][4 * j + 4] = '0';
					//	mazeArray[4 * i + 2][4 * j + 4] = '0';
					//	mazeArray[4 * i + 3][4 * j + 4] = '0';
					//}
					//else
					//{
					//	mazeArray[4 * i + 1][4 * j + 4] = '-';
					//	mazeArray[4 * i + 2][4 * j + 4] = '-';
					//	mazeArray[4 * i + 3][4 * j + 4] = '-';
					//}
					////top
					//if (maze.map[i][j].top == Wall)
					//{
					//	mazeArray[4 * i][4 * j + 1] = '1';
					//	mazeArray[4 * i][4 * j + 2] = '1';
					//	mazeArray[4 * i][4 * j + 3] = '1';
					//}
					//else if (maze.map[i][j].top == Empty)
					//{
					//	mazeArray[4 * i][4 * j + 1] = '0';
					//	mazeArray[4 * i][4 * j + 2] = '0';
					//	mazeArray[4 * i][4 * j + 3] = '0';
					//}
					//else
					//{
					//	mazeArray[4 * i][4 * j + 1] = '-';
					//	mazeArray[4 * i][4 * j + 2] = '-';
					//	mazeArray[4 * i][4 * j + 3] = '-';
					//}
					////bottom
					//if (maze.map[i][j].bottom == Wall)
					//{
					//	mazeArray[4 * i + 4][4 * j + 1] = '1';
					//	mazeArray[4 * i + 4][4 * j + 2] = '1';
					//	mazeArray[4 * i + 4][4 * j + 3] = '1';
					//}
					//else if (maze.map[i][j].bottom == Empty)
					//{
					//	mazeArray[4 * i + 4][4 * j + 1] = '0';
					//	mazeArray[4 * i + 4][4 * j + 2] = '0';
					//	mazeArray[4 * i + 4][4 * j + 3] = '0';
					//}
					//else
					//{
					//	mazeArray[4 * i + 4][4 * j + 1] = '-';
					//	mazeArray[4 * i + 4][4 * j + 2] = '-';
					//	mazeArray[4 * i + 4][4 * j + 3] = '-';
					//}
					////left
					//if (maze.map[i][j].left == Wall)
					//{
					//	mazeArray[4 * i + 1][4 * j] = '1';
					//	mazeArray[4 * i + 2][4 * j] = '1';
					//	mazeArray[4 * i + 3][4 * j] = '1';
					//}
					//else if (maze.map[i][j].left == Empty)
					//{
					//	mazeArray[4 * i + 1][4 * j] = '0';
					//	mazeArray[4 * i + 2][4 * j] = '0';
					//	mazeArray[4 * i + 3][4 * j] = '0';
					//}
					//else
					//{
					//	mazeArray[4 * i + 1][4 * j] = '-';
					//	mazeArray[4 * i + 2][4 * j] = '-';
					//	mazeArray[4 * i + 3][4 * j] = '-';
					//}
					//if (maze.map[i][j].blackHole == true)
					//{
					//	if (maze.map[i][j].redTile == true)
					//	{
					//		mazeArray[4 * i + 1][4 * j + 1] = '8';
					//		mazeArray[4 * i + 1][4 * j + 3] = '8';
					//		mazeArray[4 * i + 3][4 * j + 1] = '8';
					//		mazeArray[4 * i + 3][4 * j + 3] = '8';
					//	}
					//	else if (maze.map[i][j].greenTile == true)
					//	{
					//		mazeArray[4 * i + 1][4 * j + 1] = '9';
					//		mazeArray[4 * i + 1][4 * j + 3] = '9';
					//		mazeArray[4 * i + 3][4 * j + 1] = '9';
					//		mazeArray[4 * i + 3][4 * j + 3] = '9';
					//	}
					//	else
					//	{
					//		mazeArray[4 * i + 1][4 * j + 1] = '2';
					//		mazeArray[4 * i + 1][4 * j + 3] = '2';
					//		mazeArray[4 * i + 3][4 * j + 1] = '2';
					//		mazeArray[4 * i + 3][4 * j + 3] = '2';
					//	}
					//}
					//else if (maze.map[i][j].blueTile == true)
					//{
					//	mazeArray[4 * i + 1][4 * j + 1] = '6';
					//	mazeArray[4 * i + 1][4 * j + 3] = '6';
					//	mazeArray[4 * i + 3][4 * j + 1] = '6';
					//	mazeArray[4 * i + 3][4 * j + 3] = '6';
					//}
					//if (maze.map[i][j].purpleTile == true)
					//{
					//	mazeArray[4 * i + 1][4 * j + 1] = '7';
					//	mazeArray[4 * i + 1][4 * j + 3] = '7';
					//	mazeArray[4 * i + 3][4 * j + 1] = '7';
					//	mazeArray[4 * i + 3][4 * j + 3] = '7';
					//}
					//else if (maze.map[i][j].sandTile == true)
					//{
					//	mazeArray[4 * i + 1][4 * j + 1] = '3';
					//	mazeArray[4 * i + 1][4 * j + 3] = '3';
					//	mazeArray[4 * i + 3][4 * j + 1] = '3';
					//	mazeArray[4 * i + 3][4 * j + 3] = '3';
					//}
					//else if (maze.map[i][j].grayTile == true)
					//{
					//	mazeArray[4 * i + 1][4 * j + 1] = '4';
					//	mazeArray[4 * i + 1][4 * j + 3] = '4';
					//	mazeArray[4 * i + 3][4 * j + 1] = '4';
					//	mazeArray[4 * i + 3][4 * j + 3] = '4';
					//}
					//else
					//{
					//	mazeArray[4 * i + 1][4 * j + 1] = '0';
					//	mazeArray[4 * i + 1][4 * j + 3] = '0';
					//	mazeArray[4 * i + 3][4 * j + 1] = '0';
					//	mazeArray[4 * i + 3][4 * j + 3] = '0';
					//}
				}
			}
		}
	}
	int vectRows = max_y - min_y + 1, vectCols = max_x - min_x + 1, mapRows = vectRows * 4 + 1, mapCols = vectCols * 4 + 1;
	printf("%d %d \n", mapRows, mapCols);
	for (int i = 0; i < mapRows; i++)
	{
		for (int j = 0; j < mapCols; j++)
		{
			newMazeArray[i][j] = mazeArray[4 * min_y + i][4 * min_x + j];
		}
		//printf("\n");
	}
	vector <char> flattened;
	for (int i = 0; i < mapRows; i++) {
		for (int j = 0; j < mapCols; j++) {

			flattened.push_back(newMazeArray[i][j]); // Flatten the array with comma separators
			flattened.push_back(',');
		}
	}

	flattened.pop_back(); // Remove the last unnecessary comma
	//vector <char> message;
	//message[0] = vectRows;
	//message.push_back(vectCols);
	//message.erase(message.begin());
	//for (int i = 0; i < (mapRows * mapCols * 2 - 1); i++)
	//{
	//	message.push_back(flattened[i]);
	//}
	//
	//emitter->send(&message, sizeof(message));
	//char msg = 'M'; // Send map evaluate request
	//emitter->send(&msg, sizeof(msg));

	//char* message = (char*)malloc(8 + flattened.size());

	//memcpy(message, &width, sizeof(width)); // The first 2 integers in the message array are width, height
	//memcpy(&message[4], &height, sizeof(height));

	//memcpy(&message[8], flattened.c_str(), flattened.size()); // Copy in the flattened map afterwards

	//while (robot->step(timeStep) != -1) {

	//	emitter->send(message, sizeof(message)); // Send map data

	//	char msg = 'M'; // Send map evaluate request
	//	emitter->send(&msg, sizeof(msg));

	//	msg = 'E'; // Send an Exit message to get Map Bonus
	//	emitter->send(&msg, sizeof(msg));
	//}

	//delete robot;
}
/*
	const int width = 401, height = 401;
	char mazeArray[height][width];
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 100; j++)
		{
			//right
			if (maze.map[i][j].right == Wall)
			{
				mazeArray[4 * i + 1][4 * j + 4] = 1;
				mazeArray[4 * i + 2][4 * j + 4] = 1;
				mazeArray[4 * i + 3][4 * j + 4] = 1;
			}
			else if (maze.map[i][j].right == Empty)
			{
				mazeArray[4 * i + 1][4 * j + 4] = 0;
				mazeArray[4 * i + 2][4 * j + 4] = 0;
				mazeArray[4 * i + 3][4 * j + 4] = 0;
			}
			else
			{
				mazeArray[4 * i + 1][4 * j + 4] = '-';
				mazeArray[4 * i + 2][4 * j + 4] = '-';
				mazeArray[4 * i + 3][4 * j + 4] = '-';
			}
			//top
			if (maze.map[i][j].top == Wall)
			{
				mazeArray[4 * i][4 * j + 1] = 1;
				mazeArray[4 * i][4 * j + 2] = 1;
				mazeArray[4 * i][4 * j + 3] = 1;
			}
			else if (maze.map[i][j].top == Empty)
			{
				mazeArray[4 * i][4 * j + 1] = 0;
				mazeArray[4 * i][4 * j + 2] = 0;
				mazeArray[4 * i][4 * j + 3] = 0;
			}
			else
			{
				mazeArray[4 * i][4 * j + 1] = '-';
				mazeArray[4 * i][4 * j + 2] = '-';
				mazeArray[4 * i][4 * j + 3] = '-';
			}
			//bottom
			if (maze.map[i][j].top == Wall)
			{
				mazeArray[4 * i + 4][4 * j + 1] = 1;
				mazeArray[4 * i + 4][4 * j + 2] = 1;
				mazeArray[4 * i + 4][4 * j + 3] = 1;
			}
			else if (maze.map[i][j].top == Empty)
			{
				mazeArray[4 * i + 2][4 * j + 1] = 0;
				mazeArray[4 * i + 2][4 * j + 2] = 0;
				mazeArray[4 * i + 2][4 * j + 3] = 0;
			}
			else
			{
				mazeArray[4 * i + 2][4 * j + 1] = '-';
				mazeArray[4 * i + 2][4 * j + 2] = '-';
				mazeArray[4 * i + 2][4 * j + 3] = '-';
			}
			//left
			if (maze.map[i][j].right == Wall)
			{
				mazeArray[4 * i + 1][4 * j] = 1;
				mazeArray[4 * i + 2][4 * j] = 1;
				mazeArray[4 * i + 3][4 * j] = 1;
			}
			else if (maze.map[i][j].right == Empty)
			{
				mazeArray[4 * i + 1][4 * j] = 0;
				mazeArray[4 * i + 2][4 * j] = 0;
				mazeArray[4 * i + 3][4 * j] = 0;
			}
			else
			{
				mazeArray[4 * i + 1][4 * j] = '-';
				mazeArray[4 * i + 2][4 * j] = '-';
				mazeArray[4 * i + 3][4 * j] = '-';
			}
		}

*/