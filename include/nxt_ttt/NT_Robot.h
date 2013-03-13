#ifndef NT_ROBOT_H
#define NT_ROBOT_H

#include <vector>

#include "nxt_ttt/NT_AbstractRobot.h"
#include "nxt_ttt/NT_TTT.h"
#include "nxt_ttt/NT_Helper.h"
#include "nxt_ttt/NT_BotBoard.h"
#include "nxt_ttt/NT_Motor.h"

#include <ros/ros.h>
#include <nxt_msgs/Color.h>
#include <nxt_msgs/Range.h>
#include <nxt_msgs/Contact.h>
#include <sensor_msgs/JointState.h>

#define BASE_EFF_INC 0.11f

namespace nxt_ttt
{

class Robot : public AbstractRobot
{
private:
    enum RobotAction {
        RA_NONE,
        RA_GETCOLOR,
        RA_DROPBALL,
        RA_GETPLAYEREVENT
    };

    enum RobotState{
        RS_NONE                 = 0,
        RS_TURNINGPLATMOTOR     = 1,
        RS_TURNINGDROPMOTOR     = 2,
        RS_TURNINGSLIDEMOTOR    = 4,
        RS_TURNINGMASK          = 7,
        RS_WAITINGCOLOR         = 8,
        RS_WAITINGPLAYEREVENT   = 16
    };

public:
    Robot();
    virtual ~Robot();

    virtual void            getColor(int x, int y);
    virtual void            dropBall(int x, int y);
    virtual void            waitPlayerPlay();

    void                    actionPerformed(const std::string & sMotorName);

private:
    ros::NodeHandle         m_NodeHandle;

    ros::Subscriber         m_MotorSubscriber;
    ros::Subscriber         m_UltraSubscriber;
    ros::Subscriber         m_ColorSubscriber;
    ros::Subscriber         m_ContactSubscriber;

    RobotAction             m_CurrentAction;
    unsigned long           m_State;

    Motor                   m_PlatMotor;
    Motor                   m_DropMotor;
    Motor                   m_SlideMotor;

    BotBoard                m_BotBoard;

    int                     m_nDesiX;
    int                     m_nDesiY;

    double                  m_dPlatOr;

    std::vector<Color>      m_vColor;

    void                    stopAll();
    void                    rotatePlatMotor(double dRad);
    void                    rotateSlideMotor(double dRad);
    void                    rotateDropMotor(double dRad);

    void                    motorCb(const sensor_msgs::JointState::ConstPtr & msg);
    void                    ultraCb(const nxt_msgs::Range::ConstPtr & msg);
    void                    colorCb(const nxt_msgs::Color::ConstPtr & msg);
    void                    contactCb(const nxt_msgs::Contact::ConstPtr & msg);

}; // class Robot

} // namespace nxt_ttt

#endif // NT_ROBOT_H
