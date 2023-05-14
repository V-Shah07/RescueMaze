#include "Header.hpp"

#include <queue>  
#include <set>
#include <iostream>

using namespace std;

Maze::Maze() :robot("wheel2 motor", "wheel1 motor",
	"dFront1", "dFront2",
	"dLeft1", "dLeft2",
	"dRight1", "dRight2",
	"dBack1", "dBack2",
	"colourSensor", "LCam", "RCam", "inertialSensor", "gps", "lidar",
	"emitter", "receiver")
{
	tracker = Tracker();

}

void Maze::insert_border(Border border, Direction dir)
{
	int x = tracker.x;
	int y = tracker.y;

	map[y][x].visited = true;
	switch (dir)
	{
	case Up:
		map[y][x].top = border;
		map[y - 1][x].bottom = border;
		return;
	case Left:
		map[y][x].left = border;
		map[y][x - 1].right = border;
		return;
	case Right:
		map[y][x].right = border;
		map[y][x + 1].left = border;
		return;
	case Down:
		map[y][x].bottom = border;
		map[y + 1][x].top = border;
		return;
	}
}

int sin(Direction dir)
{
	switch (dir)
	{
	case Up:
		return 1;

	case Right:
		return 0;

	case Down:
		return -1;

	case Left:
		return 0;
	}
}
int cosin(Direction dir)
{
	switch (dir)
	{
	case Up:
		return 0;
	case Right:
		return 1;
	case Down:
		return 0;
	case Left:
		return -1;
	}
}

void translate(Direction dir)
{
	
}
void Maze::update()
{

	/*
	Border left = Empty, right = Empty, up = Empty, down = Empty;

	double upD, downD, leftD, rightD;
	upD = robot.getDist(relToAbs(tracker.direction, Up));
	downD = robot.getDist(relToAbs(tracker.direction, Down));
	leftD = robot.getDist(relToAbs(tracker.direction, Left));
	rightD = robot.getDist(relToAbs(tracker.direction, Right));

	if (upD < 12.0)
	{
		up = Wall;
	}
	if (downD < 12.0)
	{
		down = Wall;
	}
	if (leftD < 12.0)
	{
		left = Wall;
	}
	if (rightD < 12.0)
	{
		right = Wall;
	}

	insert_border(left, Left);
	insert_border(right, Right);
	insert_border(up, Up);
	insert_border(down, Down);

	cout << "Up Dist: " << upD << endl;
	cout << "Down Dist: " << downD << endl;
	cout << "Left Dist: " << leftD << endl;
	cout << "Right Dist: " << rightD << endl;
	robot.delay(1000);
	//cout << "Left: " << map[tracker.y][tracker.x].left << endl;
	*/


	const LidarPoint* lidarPoints = robot.lidarFuncs();

	for (int i = 0; i < 360; i++)
	{
		cout << "X: " << lidarPoints[i].x/0.12 << " Y: " << lidarPoints[i].y/0.12 << "Z: " << lidarPoints[i].z/0.12 << endl ;
	}


	int localX = tracker.x;
	int localY = tracker.y;

	Direction dir = tracker.direction;
	int translatedX = localX * cosin(dir) - localY * sin(dir);
	int translatedY = localX * sin(dir) + localY * cosin(dir);

	
}

bool traversable(Tile map[100][100], int x, int y, Direction dir)
{
	Pos p = dirToPos(dir);
	if ((map[y][x].getBorder(dir) == Empty) && !map[y + p.y][x + p.x].blackHole)
	{
		return true;
	}
	return false;
}
vector<Direction> Maze::BFS()
{
	
	struct State
	{
		Pos p;
		vector<Direction> dirs;
	};

	std::queue<State> toSearch;
	std::set<Pos> searched; //ensures that there are no duplicates in searchTiles

	Pos pos = Pos{ tracker.y, tracker.x };

	//cout << "BFS just started " << pos << endl;

	toSearch.push(State{ pos, {} });
	searched.insert(pos);

	//cout << "Before Update(): ";
	//cout << map[tracker.y][tracker.x] << endl;
	//robot.delay(1000);
	update(); //add surrounding information to map
	//cout << "After Update(): ";
	//cout << map[tracker.y][tracker.x] << endl;
	//robot.delay(1000);
	

	//robot.delay(1000);
	//cout << pos << endl;

	
	while (!toSearch.empty()) {
		State curState = toSearch.front(); //get tile to search
		Tile t = map[curState.p.y][curState.p.x];

		//cout << "Tile to search: " << t << endl;

		//if the tile ur searching for is not visited, or the tile ur searching for is not start tile
		if ((!allTilesVisited && !t.visited) || 
			(allTilesVisited && ((curState.p.x == tracker.startX) && (curState.p.y == tracker.startY)))) 
		{
			//cout << "Going to tile " << curState.p << endl;

			return curState.dirs;
		}
		//cout << "Pushed (" << curState.p.y << ", " << curState.p.x << ") to the searched queue" << endl;

		toSearch.pop(); //remove it, since we will now search it

		//add children nodes to search
		//cout << "New set of enqueing " << endl;
		Direction dir = Up;
		if (curState.dirs.size()) dir = curState.dirs[0];

		Direction dirsSearch[4] = { dir, dirTurn(TurnRight, dir),dirTurn(Turn180, dir), dirTurn(TurnLeft, dir) };

		for (int i = 0; i < 4; i++)
		{
			Direction curDir = dirsSearch[i];
			if (traversable(map, curState.p.x, curState.p.y, curDir))
			{
				Pos p = { curState.p.y + dirToPos(curDir).y, curState.p.x + dirToPos(curDir).x };
				if (searched.count(p) == 0)
				{
					vector<Direction> dirs = curState.dirs;
					dirs.push_back(curDir);
					toSearch.push(State{ p, dirs });
					searched.insert(p);
					printDir(curDir);
					//cout << "Pushed " << p << " to the searched queue" << endl;
				}
			}
		}

	}
	
	allTilesVisited = true;
	return {};
}