#ifndef NT_ROBOT_H
#define NT_ROBOT_H

#include "nxt_ttt/NT_AbstractRobot.h"
#include "nxt_ttt/NT_TTT.h"
#include "nxt_ttt/NT_Helper.h"

namespace nxt_ttt
{

class Robot : public AbstractRobot
{

public:
    Robot() : AbstractRobot() {}

    virtual void getColor(int x, int y);
    virtual void dropBall(int x, int y);

private:

};

}

#endif // NT_ROBOT_H
