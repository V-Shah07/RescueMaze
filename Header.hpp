#pragma once

#pragma warning(disable:26495)

#include <vector>
#include <assert.h>
#include <iostream>

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



#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <math.h>

#include "Direction.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace cv;
using namespace std;


const int mapSize = 100;

// A border of a tile
enum Border {
    Unknown,
    Empty,
    Wall,
    /*
    LetterH,
    LetterS,
    LetterU,
    SignFlammable,
    SignPoision,
    SignCorrosive,
    SignPero
    
    ide
*/
};

struct Coordinate
{
    double x, y, z;
};
struct Pos
{
    int y, x;

    friend bool operator<(const Pos& l, const Pos& r) {
        return memcmp(&l, &r, sizeof(Pos)) < 0;
    }
    friend ostream& operator<<(ostream& os, const Pos& pos)
    {
        os << "(" << pos.y << ", " << pos.x << ')';
        return os;
    }
   
};


enum Hazard
{
    Flammable, Poison, Corrosive, Peroxide, NoSign
};
enum Color { White, Black, Red, Blue, Purple, Gray, Sand, NoColor };



// a struct representing the state of a tile in the maze
struct Tile {
    Border right = Border::Unknown;
    Border top = Border::Unknown;
    Border left = Border::Unknown;
    Border bottom = Border::Unknown;
    bool visited = false;

    static string printer(Border b)
    {
        if (b == Unknown)
        {
            return "Unknown";
        }
        else if (b == Empty)
        {
            return "Empty";
        }
        return "Wall";
    }

    friend ostream& operator<<(ostream& os, const Tile& tile)
    {
        cout << "Top: " << printer(tile.top) << "\nRight: " << printer(tile.right)
            << "\nBottom: " << printer(tile.bottom) << "\nLeft: " << printer(tile.left) <<
            "\nVisited: " << tile.visited;
    }
};
class RobotSensing
{
private:
    Motor* lMotor;
    Motor* rMotor;

    DistanceSensor* fDist;
    DistanceSensor* lDist;
    DistanceSensor* rDist;
    DistanceSensor* bDist;

    Camera* colorSensor;
    Camera* lCam;
    Camera* rCam;

    GPS* gps;
    InertialUnit* inertial;
    Lidar* lidar;

    Robot* robot;
    int timeStep;

public:

    RobotSensing(string leftMotor, string rightMotor,
        string frontDist, string leftDist, string rightDist, string backDist,
        string colorSensorName, string leftCamera, string rightCamera,
        string inertialName, string gpsName, string lidarName);


    Coordinate getCoords();

    double getDist(Direction dir);


    Color getColor();
    const char* printColor(Color col);

    Hazard getSign(Direction dir);
    const char* printSign(Hazard hazard);

    char getLetter(Direction dir);

    int getTimeStep();

    void turn(double deg, double speed);

    void straight(double speed, int tiles);

    void delay(int time);

};
struct Tracker
{

    int x = mapSize / 2;
    int y = mapSize / 2;
    Direction direction = Right;
};

struct Maze
{


    RobotSensing robot;


    Tile map[100][100];

    Tracker tracker;

    void update();

    void insert_border(Border border, Direction dir);

    vector<Direction> BFS();

    Maze();
};


vector<Instruction> convertToInstruction(vector<Direction> directions, Direction heading);


void printDir(Direction dir);