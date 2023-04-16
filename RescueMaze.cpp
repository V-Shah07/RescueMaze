#include "Header.hpp"

int
main(int argc, char** argv)
{
  Maze maze;

  while (maze.robot.getTimeStep() != -1) {

      cout << "Entering BFS" << endl;
    vector<Instruction> instructs =
      convertToInstruction(maze.BFS(), maze.tracker.direction);
    cout << "exiting BFS" << endl;
    for (int i = 0; i < instructs.size(); i++) {
        
      string temp = "";
      switch (instructs[i]) {
        case Forward:
          temp = "forward";
          maze.robot.straight(6.0, 1);
          
          maze.tracker.x += dirToPos(maze.tracker.direction).x;
          maze.tracker.y += dirToPos(maze.tracker.direction).y;

          maze.map[maze.tracker.y][maze.tracker.x].visited = true;
          break;
        case TurnLeft:
          temp = "turn left";
          // maze.robot.turn(90, Right);
          maze.robot.turn(-90.0, 4.0);
          maze.tracker.direction = dirTurn(TurnLeft, maze.tracker.direction);
          break;
        case TurnRight:
          temp = "turn Right";
          // maze.robot.turn(90, Right);
          maze.robot.turn(90.0, 4.0);
          maze.tracker.direction = dirTurn(TurnRight, maze.tracker.direction);
          break;
        case Turn180:
          temp = "turn 180";
          // maze.robot.turn(180, Right);
          maze.robot.turn(180.0, 4.0);
          maze.tracker.direction = dirTurn(Turn180, maze.tracker.direction);
          break;
      }

      if (maze.robot.getColor() == Black)
      {
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
        maze.robot.delay(1000);
    }
    
  }

    return 0;
}