#include "Direction.hpp"

#include <iostream>
Instruction dirDiff(Direction from, Direction to) {
    // difference between from and to
    int diff = (from - to);
    if (diff < 0) {
        diff += 4;
    }

    // Up:    0
    // Left:  1
    // Down:  2
    // Right: 3

    // (left turn) Right - Up = 3 
    // (left turn) Left - Down = -1 + 4 = 3
    // etc...
    switch (diff) {
    case 0: return Forward;
    case 1: return TurnRight;
    case 2: return Turn180;
    case 3: return TurnLeft;
    }
}

Direction dirTurn(Instruction instruction, Direction dir) {
    int d = dir;

    switch (instruction) {
    case Forward: break;
    case TurnLeft: d += 1; break;
    case TurnRight: d -= 1; break;
    case Turn180: d -= 2; break;
    }

    // keep d within the range 0..=3
    if (d > 3) {
        d -= 4;
    }
    if (d < 0) {
        d += 4;
    }

    return (Direction)d;
}

void printDir(Direction dir)
{
    switch (dir)
    {
    case Up:
        cout << "UP";
        break;
    case Down:
        cout << "Down";
        break;
    case Left:
        cout << "Left";
        break;
    case Right: 
        cout << "Right";
        break;
    }
    
    cout << endl;
}
void printInstructions(Instruction instruct)
{
    switch (instruct)
    {
    case Forward:
        cout << "Forward";
        break;
    case TurnLeft:
        cout << "Turn Left";
        break;
    case TurnRight:
        cout << "Turn Right";
        break;
    case Turn180:
        cout << "Turn 180";
        break;
    default:
        cout << "Invalid Instruction";
        break;
    }
    cout << endl;
}

vector<Instruction> convertToInstruction(vector<Direction> directions, Direction heading) {
    vector<Instruction> out;

    //printDir(directions[0]);
    Instruction instruction = dirDiff(heading, directions[0]);
    out.push_back(instruction);
    if (instruction != Forward)
    {
        out.push_back(Forward);
    }
    //printInstructions(instruction);

    // get the difference between every direction after that
    for (int i = 1; i < directions.size(); i++) {

        //printDir(directions[i]);
        Instruction instruction = dirDiff(directions[i - 1], directions[i]);
        out.push_back(instruction);
        if (instruction != Forward)
        {
            out.push_back(Forward);
        }
        //printInstructions(instruction);
    }

    return out;
}

Direction relToAbs(Direction curDir, Direction target) {
    switch (dirDiff(curDir, target)) {
    case Forward: return Up;
    case TurnLeft: return Left;
    case TurnRight: return Right;
    case Turn180: return Down;
    }
}

Pos dirToPos(Direction dir)
{
    Pos retPos = { 0, 0 };
    switch (dir) {
    case Left:
        retPos.x--;
        break;
    case Right:
        retPos.x++;
        break;
    case Up:
        retPos.y--;
        break;
    case Down:
        retPos.y++;
        break;
    }
    return retPos;
}