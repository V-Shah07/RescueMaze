#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#define run robot->step(timeStep) != -1


#include <webots/Gyro.hpp>
#include <math.h>




// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace cv;
using namespace std;

enum Color { White, Black, Red, Blue, Purple, Gray, Sand, Unknown };
enum Direction { Left, Right };


Robot* robot = new Robot();
int timeStep = robot->getBasicTimeStep();
Motor* leftMotor, * rightMotor;

void delay(int ms)
{
	// get the start time
	float init = robot->getTime();
	while (robot->step(timeStep) != -1)
	{
		if ((robot->getTime() - init) * 1000 > ms)
			break;
	}
}



//gyro functions start
#define PI 2*asin(1)

Gyro* gyro = robot->getGyro("gyro");

double angle = 0;

void update_gyro()
{
	angle += (timeStep / 1000.0) * (gyro->getValues())[1];
}

void set_speed_and_delay(float LM_speed, float RM_speed, int delay_time)
{
	leftMotor->setVelocity(LM_speed);
	rightMotor->setVelocity(RM_speed);
	delay(delay_time);
	if (delay_time > 0)
	{
		leftMotor->setVelocity(0.0);
		rightMotor->setVelocity(0.0);
	}
}

void turn_right_gyro(float speed)
{
	angle = 0;
	while (angle < PI / 2)
	{
		set_speed_and_delay(speed, -speed, 0);
		update_gyro();
	}
}

void turn_left_gyro(float speed)
{
	angle = 0;
	while (angle > -PI / 2)
	{
		set_speed_and_delay(-speed, speed, 0);
		update_gyro();
	}
}

void turn_180_gyro(float speed)
{
	angle = 0;
	while (angle < PI)
	{
		set_speed_and_delay(speed, -speed, 0);
		update_gyro();
	}
}
//gyro functions end



Color getColor(int r, int g, int b)
{
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
		return Unknown;
	}
}

char getLetter(Mat roi)
{
	int height = roi.rows;
	int width = roi.cols;

	Rect toprect(0, 0, width, height / 3);
	Rect midrect(0, height / 3, width, height / 3);
	Rect botrect(0, 2 * height / 3, width, height / 3);

	Mat topRoi(roi, toprect);
	Mat midRoi(roi, midrect);
	Mat botRoi(roi, botrect);

	vector<vector<Point>> subContours;
	vector<Vec4i>heirarchy;
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

void stop()
{
	leftMotor->setVelocity(0);
	rightMotor->setVelocity(0);
	delay(1);
}
void turn(int deg, Direction dir)
{
	int rev = 1;
	if (dir == Left)
	{
		rev = -1;
	}
	leftMotor->setVelocity(rev * 5.0);
	rightMotor->setVelocity(rev * -5.0);
	delay(5.0 * deg);
	stop();

}
void straight(int tiles, bool forward)
{
	int rev = 1;
	if (!forward)
		rev = -1;
	int power = 5.0;

	leftMotor->setVelocity(power * rev);
	rightMotor->setVelocity(power * rev);
	delay(1200);
	stop();
}



void Detec(DistanceSensor* farright) {
	if (farright->getValue() * 100 < 10) {
		while ((farright->getValue() * 100) < 10 && run) {
			turn_right_gyro(1);


		}
		printf("\nDone");
	}
	else {
		//Foward function   
		straight(2, true);
	}
}


int main(int argc, char** argv) {




	Camera* colorSensor = robot->getCamera("colourSensor");
	Camera* upCam = robot->getCamera("cameraUp");

	GPS* gps = robot->getGPS("gps");

	leftMotor = robot->getMotor("wheel2 motor");
	rightMotor = robot->getMotor("wheel1 motor");

	DistanceSensor* distance = robot->getDistanceSensor("DistanceSensor");
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);

	colorSensor->enable(timeStep);
	upCam->enable(timeStep);
	distance->enable(timeStep);
	gps->enable(timeStep);

	leftMotor->setVelocity(0);
	rightMotor->setVelocity(0);
	robot->step(timeStep);

	delay(7000);
	straight(3, true);
	straight(3, false);



	/* Testing
	unsigned char r, g, b;
	Mat original;
	while (robot->step(timeStep) != -1)
	{
		//get color values
		const unsigned char* image = colorSensor->getImage();
		r = colorSensor->imageGetRed(image, colorSensor->getWidth(), colorSensor->getWidth()/2, colorSensor->getHeight()/2);
		g = colorSensor->imageGetGreen(image, colorSensor->getWidth(), colorSensor->getWidth()/2, colorSensor->getHeight()/2);
		b = colorSensor->imageGetBlue(image, colorSensor->getWidth(), colorSensor->getWidth()/2, colorSensor->getHeight()/2);
		const unsigned char* img = upCam->getImage();
		Mat frame(upCam->getHeight(), upCam->getWidth(), CV_8UC4, (void*)upCam->getImage());
		original = frame.clone();
		Mat hsv_frame;
		cvtColor(original, hsv_frame, COLOR_BGR2HSV);
		cvtColor(frame, frame, COLOR_BGR2GRAY);
		threshold(frame, frame, 20, 255, THRESH_BINARY_INV);

		vector<vector<Point>> contours;
		vector<Vec4i> heirarchy;
		findContours(frame, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		cout << "Number of contours: " << contours.size() << endl;

		vector<Rect> boundRect(contours.size());

		auto redcolor = Scalar(0, 0, 255);
		for (int i = 0; i < contours.size(); ++i)
		{
			boundRect[i] = boundingRect(contours[i]);
			rectangle(original, boundRect[i].tl(), boundRect[i].br(), redcolor, 2);
			cout << boundRect[i].area() << endl;
			if (boundRect[i].area() >= 270)
			{
				Mat roi(frame, boundRect[i]);
				imshow("roi", roi);
				cout << "Letter: " << getLetter(roi) << endl;
			}
			else
			{
				cout << "too small\n";
			}

		}
		Mat hsvThresh;
		inRange(hsv_frame, Scalar(0, 0, 60), Scalar(20, 255, 255), hsvThresh);
		imshow("hsv", hsv_frame);
		imshow("hsv thresh", hsvThresh);
		imshow("orig", original);
		imshow("thresh", frame);
		waitKey(1);
		<< "Location-xyz: (" << gps->getValues()[0] << ", " << gps->getValues()[1] << ", " << gps->getValues()[2] << ")" << endl;

	}
	*/
	robot;
	return 0;
}
