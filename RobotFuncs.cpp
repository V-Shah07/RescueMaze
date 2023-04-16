#include "Header.hpp"

#define PI acos(-1)


RobotSensing::RobotSensing(string leftMotor, string rightMotor,
	string frontDist, string leftDist, string rightDist, string backDist,
	string colorSensorName, string leftCamera, string rightCamera,
	string inertialName, string gpsName, string lidarName)
{
	robot = new Robot();

	timeStep = (int)robot->getBasicTimeStep();
	cout << "timeStep " << timeStep << endl;

	//Motors
	lMotor = robot->getMotor(leftMotor);
	rMotor = robot->getMotor(rightMotor);
	//Distance Sensors
	fDist = robot->getDistanceSensor(frontDist);
	lDist = robot->getDistanceSensor(leftDist);
	rDist = robot->getDistanceSensor(rightDist);
	bDist = robot->getDistanceSensor(backDist);
	//Color Sensor + Cameras
	colorSensor = robot->getCamera("colourSensor");
	lCam = robot->getCamera(leftCamera);
	rCam = robot->getCamera(rightCamera);
	//Inertial+GPS+Lidar
	inertial = robot->getInertialUnit(inertialName);
	gps = robot->getGPS(gpsName);
	//lidar = robot->getLidar(lidarName);

	lMotor->setPosition(INFINITY);
	rMotor->setPosition(INFINITY);
	lMotor->setVelocity(0.0);
	rMotor->setVelocity(0.0);

	colorSensor->enable(timeStep);
	rCam->enable(timeStep);
	lCam->enable(timeStep);
	lDist->enable(timeStep);
	rDist->enable(timeStep);
	fDist->enable(timeStep);
	bDist->enable(timeStep);
	inertial->enable(timeStep);
	//lidar->enable(timeStep);
	gps->enable(timeStep);

	getTimeStep();
}

Coordinate RobotSensing::getCoords()
{
	Coordinate retCoord;
	retCoord.x = gps->getValues()[0];
	retCoord.y = gps->getValues()[1];
	retCoord.z = gps->getValues()[2];

	return	retCoord;
}

double RobotSensing::getDist(Direction dir) {

	DistanceSensor* distSens;

	switch (dir)
	{
	case Up:
		distSens = fDist;
		break;
	case Left:
		distSens = lDist;
		break;
	case Right:
		distSens = rDist;
		break;
	case Down:
		distSens = bDist;
		break;
	default:
		distSens = NULL;
		break;
	}

	if (distSens != NULL)
	{
		return distSens->getValue() * 100;// gets it in cm 
	}
	return 0;
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
	else if ((r < 40) && (g < 40) && (b < 40))
	{
		return Black;
	}
	else if ((r > 230) && (g < 50) && (b < 50))
	{
		return Red;
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
		return "Unkown Color";
	}
}

Hazard RobotSensing::getSign(Direction dir)
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
	Mat frame_rgb(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage()), frame_hsv, thresholded_img;
	vector<vector<Point>> contours;
	bool match_found = 0;
	cvtColor(frame_rgb, frame_hsv, COLOR_BGR2HSV);
	inRange(frame_hsv, Scalar(15, 127, 127), Scalar(35, 255, 255), thresholded_img); //yellow (organic peroxide)
	findContours(thresholded_img, contours, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) > 400.0)
		{
			printf("organic peroxide \n");
			match_found = 1;
			return Peroxide;
		}
	}
	if (match_found == 0)
	{
		inRange(frame_hsv, Scalar(160, 0, 0), Scalar(170, 255, 255), thresholded_img); //red (flammable gas)
		findContours(thresholded_img, contours, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > 500.0)
			{
				printf("flammable gas \n");
				match_found = 1;
				return Flammable;
			}
		}
	}
	if (match_found == 0)
	{
		inRange(frame_hsv, Scalar(0, 0, 0), Scalar(0, 0, 0), thresholded_img); //black (corrosive)
		findContours(thresholded_img, contours, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > 400.0)
			{
				printf("corrosive \n");
				match_found = 1;
				return Corrosive;
			}
		}
	}
	if (match_found == 0)
	{
		inRange(frame_hsv, Scalar(0, 0, 200), Scalar(0, 0, 255), thresholded_img); //white/gray (poison)
		findContours(thresholded_img, contours, noArray(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > 1000.0)
			{
				printf("poison \n");
				match_found = 1;
				return Poison;
			}
		}
	}

	return NoSign;
}

const char* RobotSensing::printSign(Hazard hazard)
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
	case NoSign:
		return "Unkown sign";
	}
}


char RobotSensing::getLetter(Direction dir)
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
		return '0';
	}
	//get the frame
	Mat frame(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)cam->getImage());

	//finds contours
	vector<vector<Point>> contours;
	vector<Vec4i> heirarchy;
	findContours(frame, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

	//array of bounding rect for different contours
	vector<Rect> boundRect(contours.size());

	auto redcolor = Scalar(0, 0, 255);
	Mat roi;
	for (int i = 0; i < contours.size(); ++i) //loops through all contours
	{
		boundRect[i] = boundingRect(contours[i]); //makes rect for contour
		rectangle(frame, boundRect[i].tl(), boundRect[i].br(), redcolor, 2); //draws rectangle

		if (boundRect[i].area() >= 270) //if size is good, it gets letter
		{

			roi = Mat(frame, boundRect[i]); //POSSIBLY CAUSE CRASH
			imshow("roi", roi);

		}
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

	findContours(topRoi, subContours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	int numTopContours = subContours.size();
	subContours.clear();
	findContours(midRoi, subContours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	int numMidContours = subContours.size();
	subContours.clear();
	findContours(botRoi, subContours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	int numBotContours = subContours.size();

	if ((numTopContours == 2) && (numMidContours == 2) && (numBotContours == 1))
		return 'U';
	else if ((numTopContours == 1) && (numMidContours == 1) && (numBotContours == 1))
		return 'S';
	else if ((numTopContours == 2) && (numMidContours == 1) && (numBotContours == 2))
		return 'H';
	else
		return '0';
}

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

void RobotSensing::straight(const int tiles) {
	int fwd = tiles > 0 ? 1 : -1;
	const double tileSize = 0.12, startX = -0.17999, startZ = -0.286802;
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

	const double thresh = 0.001, min = 0.0;
	const double kp = 15.0, kd = 12.0;
	const double padding = 0.2;

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
	}

	turn(0);
}
