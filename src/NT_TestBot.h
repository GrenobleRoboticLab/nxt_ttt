#ifndef NT_TESTBOT_H
#define NT_TESTBOT_H

#include "nxt_ttt/NT_AbstractRobot.h"
#include "nxt_ttt/NT_Motor.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace nxt_ttt {

const std::string plat("plat_motor");
const std::string drop("drop_motor");
const std::string slide("slide_motor");

class TestBot : public AbstractRobot
{
public:
    TestBot();
    virtual ~TestBot() { ; }

    virtual void    getColor(int x, int y);
    virtual void    dropBall(int x, int y);
    virtual void    waitPlayerPlay();
    virtual void    actionPerformed(const std::string & sMotorName);
    void            ask();

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_MSub;

    Motor           m_SlideMotor;
    Motor           m_DropMotor;
    Motor           m_PlatMotor;

    bool            m_bNeedDisplay;

    std::string     m_sCurrentMotor;
    void            motorCb(const sensor_msgs::JointState::ConstPtr & msg);
};

}

#endif // NT_TESTBOT_H
