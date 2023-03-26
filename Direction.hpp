#pragma once

#include <vector>
using namespace std;

// direction
//  +1 to go left,
//  -1 to go right
enum Direction { Up, Left, Down, Right };

enum Instruction { Forward, TurnLeft, TurnRight, Turn180 };

vector<Instruction> convertToInstruction(vector<Direction> directions, Direction heading);

// returns what you need to do to turn from a to b
Instruction dirDiff(Direction from, Direction to);

// return the direction after the specified turn
Direction dirTurn(Instruction instruction, Direction dir);

Direction relToAbs(Direction curDir, Direction target);