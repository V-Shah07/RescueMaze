#include "Header.hpp"

#include <queue>  
#include <set>
#include <iostream>

using namespace std;

Maze::Maze() :robot("wheel2 motor", "wheel1 motor",
	"dFront1", "dLeft1", "dRight1", "dBack1",
	"colourSensor", "LCam", "RCam", "inertialSensor", "gps", "lidarSensor")
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

void Maze::update()
{
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

	//cout << "Left: " << map[tracker.y][tracker.x].left << endl;
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
		State* lastState;
		Direction dir;
	};

	std::queue<State> toSearch;
	std::set<Pos> searched; //ensures that there are no duplicates in searchTiles
	vector<State> searchedStates;

	Pos pos = Pos{ tracker.y, tracker.x };

	//cout << "BFS just started " << pos << endl;

	toSearch.push(State{ pos, NULL });
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

		cout << "Tile to search: " << t << endl;

		//if the tile ur searching for is not visited, or the tile ur searching for is not start tile
		if ((!allTilesVisited && !t.visited) || 
			(allTilesVisited && ((curState.p.x == tracker.startX) && (curState.p.y == tracker.startY)))) 
		{
			vector<Direction> out;
			State* cur = &curState;
			//cout << "LAST STATE POSITION: " << curState.lastState->p << endl;
			State* prev = curState.lastState;
			while (prev != NULL) {
				int dx = cur->p.x - prev->p.x;
				int dy = cur->p.y - prev->p.y;

				// say: cur is 0,0 prev is 1,0 we had to travel left
				// dx: -1
				// dy: 0

				//cout << "Dx: " << dx << "  Dy: " << dy << endl;

				if (dx == -1 && dy == 0) out.push_back(Left);
				else if (dx == 1 && dy == 0) out.push_back(Right);
				else if (dx == 0 && dy == -1) out.push_back(Up);
				else if (dx == 0 && dy == 1) out.push_back(Down);
				else cout << "trackingInfo:153 invalid previous pos" << endl;

				cur = prev;
				prev = prev->lastState;
			}
			//cout << "Going to tile " << curState.p << endl;

			return out;
		}

		searchedStates.push_back(curState);
		cout << "Pushed (" << curState.p.y << ", " << curState.p.x << ") to the searched queue" << endl;

		State* prevState = &searchedStates[searchedStates.size() - 1];
		cout << "Prevstate: " << prevState->p << endl;
		curState.lastState = prevState;


		toSearch.pop(); //remove it, since we will now search it

		curState.lastState = prevState;

		curState.dir = tracker.direction;

		//add children nodes to search
		cout << "New set of enqueing " << endl;

		Direction dirsSearch[4] = { curState.dir, dirTurn(TurnRight, curState.dir),dirTurn(Turn180, curState.dir), dirTurn(TurnLeft, curState.dir) };

		for (int i = 0; i < 4; i++)
		{
			Direction curDir = dirsSearch[i];
			if (traversable(map, curState.p.x, curState.p.y, curDir))
			{
				Pos p = { curState.p.y + dirToPos(curDir).y, curState.p.x + dirToPos(curDir).x };
				if (searched.count(p) == 0)
				{
					toSearch.push(State{ p, prevState, curDir });
					searched.insert(p);
					printDir(curDir);
					cout << "Pushed " << p << " to the searched queue" << endl;
				}
			}
		}

	}
	
	allTilesVisited = true;
	return {};
}