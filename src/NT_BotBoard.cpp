#include <cmath>
#include "nxt_ttt/NT_BotBoard.h"
#include "nxt_ttt/NT_Helper.h"

using namespace nxt_ttt;

BotBoard::BotBoard() { ; }
BotBoard::~BotBoard() { ; }

double BotBoard::getPlatRotation(int x, int y)
{
    double dAngle = 0.0;

    if      (x == 0 && y == 0)
        dAngle = M_PI / 4.0;
    else if (x == 0 && y == 1)
        dAngle = M_PI / 2.0;
    else if (x == 0 && y == 2)
        dAngle = 3.0 * (M_PI / 4.0);
    else if (x == 1 && y == 0)
        dAngle = 0;
    else if (x == 1 && y == 1)
        dAngle = 0;
    else if (x == 1 && y == 2)
        dAngle = M_PI;
    else if (x == 2 && y == 0)
        dAngle = -(M_PI / 4.0);
    else if (x == 2 && y == 1)
        dAngle = -(M_PI / 2.0);
    else if (x == 2 && y == 2)
        dAngle = -(3.0 * (M_PI / 4.0));

    return dAngle;
}

double BotBoard::getColorSlideRotation(int x, int y)
{
    if (x == 1 && y == 1)
        return 1.2;
    else return 0.5;
}

double BotBoard::getDropSlideRotation(int x, int y)
{
    if (x == 1 && y == 1)
        return 2;
    else return 1.5;
}
