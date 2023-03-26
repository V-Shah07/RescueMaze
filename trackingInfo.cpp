#include "Header.hpp"

#include <queue>  
#include <set>
#include <iostream>

using namespace std;

Maze::Maze() :robot("wheel2 motor", "wheel1 motor",
	"distanceSensorFront", "distanceSensorLeft", "distanceSensorRight", "distanceSensorBack",
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
	//assumer current direction is left
	switch (dir)
	{
	case Up:
		if (map[y][x].top == Empty /* && !map[y][x].visited*/)
		{
			return true;
		}
		break;
	case Left:
		if (map[y][x].left == Empty/*&&  && !map[y][x].visited*/)
		{
			return true;
		}
		break;
	case Right:
		if (map[y][x].right == Empty/*&& /* && !map[y][x].visited*/)
		{
			return true;
		}
		break;
	case Down:
		if (map[y][x].bottom == Empty/* && /* && !map[y][x].visited*/)
		{
			return true;
		}
		break;
	}
	return false;
}
vector<Direction> Maze::BFS()
{
	
	struct State
	{
		Pos p;
		State* lastState;
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
		
		//cout << t << endl;

		//cout << "Current state position: " << curState.p << endl;
		if (curState.lastState != NULL)
		{
			//cout << "Previous state position: " << curState.lastState->p << endl;

		}
		else
		{
			//cout << "Previous state is null " << endl;

		}
		
		if (!t.visited) {
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
		//cout << "Pushed (" << curState.p.y << ", " << curState.p.x << ") to the searched queue" << endl;

		State* prevState = &searchedStates[searchedStates.size() - 1];
		//cout << "Prevstate: " << prevState->p << endl;
		curState.lastState = prevState;


		toSearch.pop(); //remove it, since we will now search it

		curState.lastState = prevState;

		//add children nodes to search
		cout << "New set of enqueing " << endl;

		Pos curSearchPos = { curState.p.y, curState.p.x };
		if (traversable(map, curState.p.x, curState.p.y, Up))
		{
			toSearch.push(State{ Pos {curState.p.y - 1, curState.p.x}, prevState});
			cout << "(Up)Pushed (" << curState.p.y - 1 << ", " << curState.p.x << ") to the searched queue" << endl;

		}
		if (traversable(map, curState.p.x, curState.p.y, Right))
		{
			toSearch.push(State{ Pos {curState.p.y, curState.p.x + 1}, prevState });
			cout << "(Right)Pushed (" << curState.p.y << ", " << curState.p.x + 1<< ") to the searched queue" << endl;

		}
		if (traversable(map, curState.p.x, curState.p.y, Down))
		{
			toSearch.push(State{ Pos {curState.p.y + 1, curState.p.x}, prevState });
			cout << "(Down)Pushed (" << curState.p.y + 1<< ", " << curState.p.x << ") to the searched queue" << endl;

		}
		if (traversable(map, curState.p.x, curState.p.y, Left))
		{
			toSearch.push(State{ Pos {curState.p.y, curState.p.x - 1}, prevState });
			cout << "(Left)Pushed (" << curState.p.y << ", " << curState.p.x -1 << ") to the searched queue" << endl;

		}

		

	}

	//figure out where the next unvisited tile is
	// traversable(map, tracker.curRow, tracker.curCol, curDir); //USE THIS TO SEE IF YOU CAN MOVE IN SAME DIR

	return {};
}