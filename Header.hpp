#pragma once

#pragma warning(disable:26495)

#include <vector>
#include <assert.h>
#include <iostream>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "Direction.hpp"
#include "PID.hpp"

#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>

#define PI acos(-1)

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
    SignPeroxide
*/
};
enum StraightReturn
{
    Normal, BlackHole, RedTile, GreenTile
};
struct Coordinate
{
    double x, y, z;
};



enum signs_and_victims
{
    Flammable, Poison, Corrosive, Peroxide, H, S, U, NoSign
};
enum Color { White, Black, Red, Green, Blue, Purple, Gray, Sand, NoColor };


struct Maze;

// a struct representing the state of a tile in the maze
struct Tile {
    Border right = Border::Unknown;
    Border top = Border::Unknown;
    Border left = Border::Unknown;
    Border bottom = Border::Unknown;

    

    bool blackHole = false;
    bool redTile = false;
    bool greenTile = false;
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

    Border getBorder(Direction dir)
    {
        switch (dir)
        {
        case Up:
            return top;
        case Left:
            return left;
        case Down:
            return bottom;
        case Right:
            return right;
        }
    }
};

class RobotSensing
{
private:
    Motor* lMotor;
    Motor* rMotor;

    double maxSpeed = 6.28;

    DistanceSensor* fDist1;
    DistanceSensor* fDist2;
    DistanceSensor* lDist1;
    DistanceSensor* lDist2;
    DistanceSensor* rDist1;
    DistanceSensor* rDist2;
    DistanceSensor* bDist1;
    DistanceSensor* bDist2;

    Camera* colorSensor;
    Camera* lCam;
    Camera* rCam;

    GPS* gps;
    InertialUnit* inertial;
    Lidar* lidar;

    Emitter* emitter;
    Receiver* receiver;

    Robot* robot;
    int timeStep;

    double startX, startZ;

public:

    RobotSensing(string leftMotor, string rightMotor,
        string frontDist1, string frontDist2,
        string leftDist1, string leftDist2, 
        string rightDist1, string rightDist2,
        string backDist1, string backDist2,
        string colorSensorName, string leftCamera, string rightCamera,
        string inertialName, string gpsName, string lidarName,
        string emitterName, string receiverName);

    double getYaw();

    Coordinate getCoords();

    double getDist(Direction dir);


    Color getColor();
    const char* printColor(Color col);

    signs_and_victims getSign(Direction dir);
    const char* printSign(signs_and_victims hazard);

    char getLetter(Direction dir);

    int getTimeStep();

    void turn(double deg);

    StraightReturn straight(int tiles, Maze &maze, bool checkBlackHole=true);

    void delay(int time);

    const LidarPoint* lidarFuncs();

    void transmission(char victim);

    void exit_maze();

    void submit_maze(Maze maze);
};
double radToDeg(double radians);
double degToRad(double degrees);

struct Tracker
{
    int startX = mapSize / 2;
    int startY = mapSize / 2;
    int x = startX;
    int y = startY;
    Direction direction = Right;
};

struct Maze
{


    RobotSensing robot;


    Tile map[100][100];

    Tracker tracker;

    void update();

    void insert_border(Border border, Direction dir);

    vector<Direction> BFS(/*bool(*searchTarget)(Tile, int, int)*/);

    bool allTilesVisited = false;

    Maze();
};


vector<Instruction> convertToInstruction(vector<Direction> directions, Direction heading);


void printDir(Direction dir);