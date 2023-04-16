#include "Header.hpp"

void executeMoves(vector<Direction> dirs, Maze& maze) {
    
    vector<Instruction> instructions = convertToInstruction(dirs, maze.tracker.direction);
    
    
    for (int i = 0; i < instructions.size(); i++) {
        string temp = "";
        switch (instructions[i]) {
        case Forward:
            temp = "forward";
            maze.robot.straight(1);

            maze.tracker.x += dirToPos(maze.tracker.direction).x;
            maze.tracker.y += dirToPos(maze.tracker.direction).y;

            maze.map[maze.tracker.y][maze.tracker.x].visited = true;
            break;
        case TurnLeft:
            temp = "turn left";
            // maze.robot.turn(90, Right);
            maze.robot.turn(90.0);
            maze.tracker.direction = dirTurn(TurnLeft, maze.tracker.direction);
            break;
        case TurnRight:
            temp = "turn Right";
            // maze.robot.turn(90, Right);
            maze.robot.turn(-90.0);
            maze.tracker.direction = dirTurn(TurnRight, maze.tracker.direction);
            break;
        case Turn180:
            temp = "turn 180";
            // maze.robot.turn(180, Right);
            maze.robot.turn(180.0);
            maze.tracker.direction = dirTurn(Turn180, maze.tracker.direction);
            break;
        }

        if (maze.robot.getColor() == Black) {
            cout << "BLACK" << endl;
            Pos p = dirToPos(maze.tracker.direction);
            maze.map[maze.tracker.y + p.y][maze.tracker.x + p.x].blackHole = true;
            break;
        }

        // cout << temp << endl;
        cout << "(" << maze.tracker.y << ", " << maze.tracker.x << ")" << endl;
        // cout << "Direction: ";

        cout << maze.map[maze.tracker.y][maze.tracker.x] << endl;

        printDir(maze.tracker.direction);
//        maze.robot.delay(1000);
    }
}

int main()
{
    Maze maze;

    while (maze.robot.getTimeStep() != -1) {
        //maze.robot.turn(-90);
        //cout << "Facing: " << radToDeg(maze.robot.getYaw()) << endl;
        //auto coords = maze.robot.getCoords();
        //cout << coords.x << " " << coords.y << " " << coords.z << endl;

        ////maze.robot.straight(1);
        ////maze.robot.straight(-1);

        //maze.robot.delay(1000);   

        vector<Direction> dir = maze.BFS();
        if (dir.size() != 0)
        {
           executeMoves(dir, maze);
        }
       

    
    
    }
    return 0;
}