#ifndef NT_ROBOT_H
#define NT_ROBOT_H

#include "nxt_ttt/NT_AbstractRobot.h"
#include "nxt_ttt/NT_TTT.h"
#include "nxt_ttt/NT_Helper.h"

#include <ros/ros.h>

namespace nxt_ttt
{

class Motor
{
public:
    Motor();
    ~Motor();

private:
    ros::Publisher  m_Publisher;

    std::string     m_sName;

    float           m_fPosDesi;
    float           m_fPos;

    float           m_fEffDesi;
    float           m_fEff;
};

class Robot : public AbstractRobot
{
private:
    enum RobotAction {
        RA_GETCOLOR,
        RA_DROPBALL,
        RA_GETPLAYEREVENT
    };

    enum RobotState{
        RS_TURNINGPLATMOTOR     = 1,
        RS_TURNINGDROPMOTOR     = 2,
        RS_TURNINGCANONMOTOR    = 4,
        RS_WAITINGCOLOR         = 8,
        RS_WAITINGPLAYEREVENT   = 16,
    };

public:
    Robot() : AbstractRobot() { ; }
    virtual ~Robot();

    virtual void getColor(int x, int y);
    virtual void dropBall(int x, int y);
    virtual void waitPlayerPlay();

private:



};

}

#endif // NT_ROBOT_H
