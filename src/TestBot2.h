#ifndef TESTBOT2_H
#define TESTBOT2_H

#include "nxt_ttt/NT_AbstractRobot.h"
#include "nxt_ttt/NT_RobotAction.h"
#include "nxt_msgs/Contact.h"

namespace nxt_ttt {

const std::string plat("plat_motor");
const std::string drop("drop_motor");
const std::string slide("slide_motor");

class TestBot2 : public AbstractRobot
{
public:
    TestBot2();
    virtual ~TestBot2() { releaseAction(); }

    virtual void    getColor(int x, int y)  { ; }
    virtual void    dropBall(int x, int y)  { ; }
    virtual void    waitPlayerPlay()        { ; }
    virtual void    actionPerformed(const std::string & sMotorName);
    void            ask();

private:
    ros::NodeHandle m_nh;
    ros::Publisher  m_Pub;
    MotorAction*    m_pAction;

    ros::Subscriber m_Sub;

    bool            m_bNeedReleaseAction;

    void            releaseAction();
    void            contactCb(const nxt_msgs::Contact::ConstPtr & msg);
}; // class TestBot2

} // namespace nxt_ttt

#endif // TESTBOT2_H
