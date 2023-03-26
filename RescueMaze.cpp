#include "Header.hpp"

int
main(int argc, char** argv)
{
  Maze maze;

  while (maze.robot.getTimeStep() != -1) {

    vector<Instruction> instructs =
      convertToInstruction(maze.BFS(), maze.tracker.direction);

    for (int i = 0; i < instructs.size(); i++) {
      string temp = "";
      switch (instructs[i]) {
        case Forward:
          temp = "forward";
          maze.robot.straight(6.0, 1);
          switch (maze.tracker.direction) {
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