#include "Header.hpp"

void executeMoves(vector<Direction> dirs, Maze& maze) {
    
    vector<Instruction> instructions = convertToInstruction(dirs, maze.tracker.direction);
    
    
    for (int i = 0; i < instructions.size(); i++) {
        string temp = "";
        switch (instructions[i]) {
        case Forward:
            temp = "forward";
            
            //makes sure tracker is not updated when black hole is seen
            if (maze.robot.straight(1, maze) == Normal)
            {
                maze.tracker.x += dirToPos(maze.tracker.direction).x;
                maze.tracker.y += dirToPos(maze.tracker.direction).y;
            }

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
    signs_and_victims val, leftval;
    while (maze.robot.getTimeStep() != -1) {
        //maze.robot.turn(-90);
        //cout << "Facing: " << radToDeg(maze.robot.getYaw()) << endl;
        //auto coords = maze.robot.getCoords();
        //cout << coords.x << " " << coords.y << " " << coords.z << endl;

        ////maze.robot.straight(1);
        ////maze.robot.straight(-1);

        //maze.robot.delay(1000);   
        val = maze.robot.getSign(Right);
        leftval = maze.robot.getSign(Left);
        printf("r: %s \n", maze.robot.printSign(val));
        printf("l: %s \n", maze.robot.printSign(leftval));
        if (maze.robot.printSign(val) == "H")
        {
            maze.robot.transmission('H');
            printf("transmission successful,H \n");
        }
        if (maze.robot.printSign(val) == "S")
        {
            maze.robot.transmission('S');
            printf("transmission successful, S \n");
        }
        if (maze.robot.printSign(val) == "U")
        {
            maze.robot.transmission('U');
            printf("transmission successful,U \n");
        }
        if (maze.robot.printSign(leftval) == "H")
        {
            maze.robot.transmission('H');
            printf("transmission successful, H \n");
        }
        if (maze.robot.printSign(leftval) == "S")
        {
            maze.robot.transmission('S');
            printf("transmission successful, S \n");
        }
        if (maze.robot.printSign(leftval) == "U")
        {
            maze.robot.transmission('U');
            printf("transmission successful, U \n");
        }
        vector<Direction> dir = maze.BFS();
        if (dir.size() != 0)
        {
          executeMoves(dir, maze);
        }
        else
        {
            maze.robot.exit_maze();
        }
        //maze.robot.lidarFuncs();
        //cout << "Left: " << maze.robot.printSign(maze.robot.getSign(Left)) << endl;
        //cout << "Right: " << maze.robot.printSign(maze.robot.getSign(Right)) << endl;
    }
    return 0;
}