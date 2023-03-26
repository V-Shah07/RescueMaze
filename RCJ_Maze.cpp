#include "Header.hpp"


/*
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
*/


//gyro functions start
//#define PI 2*asin(1)
//
//double angle = 0;
//
//void update_gyro()
//{
//	angle += (timeStep / 1000.0) * (gyro->getValues())[1];
//}
//
//void set_speed_and_delay(float LM_speed, float RM_speed, int delay_time)
//{
//	leftMotor->setVelocity(LM_speed);
//	rightMotor->setVelocity(RM_speed);
//	delay(delay_time);
//	if (delay_time > 0)
//	{
//		leftMotor->setVelocity(0.0);
//		rightMotor->setVelocity(0.0);
//	}
//}
//
//void turn_right_gyro(float speed, float target_degrees)
//{
//	angle = 0;
//	while ((angle * 180 / PI) < target_degrees)
//	{
//		set_speed_and_delay(speed, -speed, 0);
//		update_gyro();
//	}
//}
//
//void turn_left_gyro(float speed, float target_degrees)
//{
//	angle = 0;
//	while ((angle * 180 / PI) > (-1 * target_degrees))
//	{
//		set_speed_and_delay(-speed, speed, 0);
//		update_gyro();
//	}
//}

//gyro functions end



/*Color getColor(int r, int g, int b)
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
*/
/*char getLetter(Mat roi)
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
	findContours(topRoi, subContours, heirarchy, RETR_TREE, CHAIN_APPRO
	_SIMPLE);
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
}*/

/*
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


*/




int main(int argc, char** argv) {
	/*
		Robot* robot = new Robot();
		int timeStep = robot->getBasicTimeStep();
		Motor* leftMotor, * rightMotor;

		Camera* colorSensor = robot->getCamera("colourSensor");
		Camera* rCam = robot->getCamera("RCam");
		Camera* lCam = robot->getCamera("LCam");
		DistanceSensor* distSensorRight = robot->getDistanceSensor("distanceSensorRight");
		DistanceSensor* distSensorLeft = robot->getDistanceSensor("distanceSensorLeft");
		DistanceSensor* distSensorFront = robot->getDistanceSensor("distanceSensorFront");
		DistanceSensor* distSensorBack = robot->getDistanceSensor("distanceSensorBack");
		InertialUnit* inertialSensor = robot->getInertialUnit("inertialSensor");
		//Lidar* lidarSensor = robot->getLidar("lidarSensor");
		GPS* gps = robot->getGPS("gps");
		//Gyro* gyro = robot->getGyro("gyro");

		leftMotor = robot->getMotor("wheel2 motor");
		rightMotor = robot->getMotor("wheel1 motor");

		*/
	
	Maze maze;

	while (maze.robot.getTimeStep() != -1)
	{
		//hazard_detection();
		//leftMotor->setVelocity(5);
		//rightMotor->setVelocity(5);
		//get color values

		//cout << "Color: " << maze.robot.printColor(maze.robot.getColor()) << endl;
		//maze.update();

		//maze.robot.straight(1, true);
		//maze.robot.delay(1000);
		//maze.robot.turn(90, Left);
		//maze.robot.delay(1000);

		vector<Direction> bfsRet = maze.BFS();

		for (int i = 0; i < bfsRet.size(); ++i)
		{
			cout << " This is what BFS Returns: " << endl;
			printDir(bfsRet[i]);
			maze.robot.delay(1000);
		}

		vector<Instruction> instructs = convertToInstruction(bfsRet, maze.tracker.direction);




		for (int i = 0; i < instructs.size(); i++)
		{
			string temp = "";
			switch (instructs[i])
			{
			case Forward:
				temp = "forward";
				maze.robot.straight(6.0, 1);
				switch (maze.tracker.direction)
				{
				case Left:
					maze.tracker.x--;
					break;
				case Right:
					maze.tracker.x++;
					break;
				case Up:
					maze.tracker.y--;
					break;
				case Down:
					maze.tracker.y++;
					break;
				}
				maze.map[maze.tracker.y][maze.tracker.x].visited = true;
				break;
			case TurnLeft:
				temp = "turn left";
				//maze.robot.turn(90, Right);
				maze.robot.turn(-90.0, 4.0);
				maze.tracker.direction = dirTurn(TurnLeft, maze.tracker.direction);
				break;
			case TurnRight:
				temp = "turn Right";
				//maze.robot.turn(90, Right);
				maze.robot.turn(90.0, 4.0);
				maze.tracker.direction = dirTurn(TurnRight, maze.tracker.direction);
				break;
			case Turn180:
				temp = "turn 180";
				//maze.robot.turn(180, Right);
				maze.robot.turn(180.0, 4.0);
				maze.tracker.direction = dirTurn(Turn180, maze.tracker.direction);
				break;
			}

			//cout << temp << endl;
			cout << "(" << maze.tracker.y << ", " << maze.tracker.x << ")" << endl;
			//cout << "Direction: ";

			//cout << maze.map[maze.tracker.y][maze.tracker.x] << endl;

			printDir(maze.tracker.direction);
			maze.robot.delay(1000);
		}
		


		//cout << "Sign: " << robot.printSign(robot.getSign(Left)) << endl;
		//cout << "Letter " << robot.getLetter(Left);
		/*
		const unsigned char* image = colorSensor->getImage();
		r = colorSensor->imageGetRed(image, colorSensor->getWidth(), colorSensor->getWidth() / 2, colorSensor->getHeight() / 2);
		g = colorSensor->imageGetGreen(image, colorSensor->getWidth(), colorSensor->getWidth() / 2, colorSensor->getHeight() / 2);
		b = colorSensor->imageGetBlue(image, colorSensor->getWidth(), colorSensor->getWidth() / 2, colorSensor->getHeight() / 2);

		const unsigned char* img = rCam->getImage();
		Mat frame(rCam->getHeight(), rCam->getWidth(), CV_8UC4, (void*)rCam->getImage());
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
				//cout << "Letter: " << getLetter(roi) << endl;

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
		*/

	}

	return 0;
}