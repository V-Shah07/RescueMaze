#include "Header.hpp"

#define PI acos(-1)


RobotSensing::RobotSensing(string leftMotor, string rightMotor,
	string frontDist, string leftDist, string rightDist, string backDist,
	string colorSensorName, string leftCamera, string rightCamera,
	string inertialName, string gpsName, string lidarName)
{
	robot = new Robot();

	timeStep = (int)robot->getBasicTimeStep();

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
	cout << r << "  " << g << "  " << b << endl;
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
void RobotSensing::turn(double degrees, double speed)
{
	bool init = 0, thresh_init = 0;
	double old_yaw, thresh;
	if (degrees > 0.0)
	{
		while (robot->step(timeStep) != -1) {
			if ((inertial->getRollPitchYaw()[2] != 0) && (init == 0))
			{
				old_yaw = inertial->getRollPitchYaw()[2];
				init = 1;
				//printf("%lf \n", old_yaw);
			}
			lMotor->setVelocity(speed);
			rMotor->setVelocity(-speed);
			if (thresh_init == 0)
			{
				thresh = old_yaw - degrees * PI / 180 - error;
				thresh_init = 1;
			}
			//printf("%lf %lf \n", inertial->getRollPitchYaw()[2], thresh);
			if (thresh <= (-PI + 0.01 * speed))
			{
				thresh = 2 * PI + thresh;
				while (robot->step(timeStep) != -1) {
					lMotor->setVelocity(speed);
					rMotor->setVelocity(-speed);
					if (inertial->getRollPitchYaw()[2] > 0)
						break;
				}
			}
			error = inertial->getRollPitchYaw()[2] - thresh;
			if (inertial->getRollPitchYaw()[2] <= thresh)
				break;
		}
	}
	else
	{
		while (robot->step(timeStep) != -1) {
			if ((inertial->getRollPitchYaw()[2] != 0) && (init == 0))
			{
				old_yaw = inertial->getRollPitchYaw()[2];
				init = 1;
				//printf("%lf \n", old_yaw);
			}
			lMotor->setVelocity(-speed);
			rMotor->setVelocity(speed);
			if (thresh_init == 0)
			{
				thresh = old_yaw - degrees * PI / 180 - error;
				thresh_init = 1;
			}
			//printf("%lf %lf \n", inertial->getRollPitchYaw()[2], thresh);
			if (thresh >= (PI - 0.01 * speed))
			{
				thresh = thresh - 2 * PI;
				while (robot->step(timeStep) != -1) {
					lMotor->setVelocity(-speed);
					rMotor->setVelocity(speed);
					if (inertial->getRollPitchYaw()[2] < 0)
						break;
				}
			}
			error = inertial->getRollPitchYaw()[2] - thresh;
			if (inertial->getRollPitchYaw()[2] >= thresh)
				break;
		}
	}
	lMotor->setVelocity(0.0);
	rMotor->setVelocity(0.0);
}

void RobotSensing::straight(double speed, int tiles)
{
	Coordinate current_coords = getCoords();
	double old_x = current_coords.x, old_y = current_coords.y, old_z = current_coords.z,
		current_x = current_coords.x, current_y = current_coords.y, current_z = current_coords.z,
		tile_test_length = 0.1184484;
	while (robot->step(timeStep) != -1)
	{
		current_coords = getCoords();
		current_x = current_coords.x;
		current_y = current_coords.y;
		current_z = current_coords.z;
		lMotor->setVelocity(speed);
		rMotor->setVelocity(speed);

		/*if (getColor() == Black)
		{

			break;
		}*/

		if (sqrt((current_x - old_x) * (current_x - old_x) + (current_y - old_y) * (current_y - old_y) + (current_z - old_z) * (current_z - old_z)) >= (tile_test_length * tiles))
			break;
	}
	lMotor->setVelocity(0);
	rMotor->setVelocity(0);
}
