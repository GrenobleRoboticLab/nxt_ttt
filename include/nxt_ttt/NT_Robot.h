#ifndef NT_ROBOT_H
#define NT_ROBOT_H

#include "nxt_ttt/NT_AbstractRobot.h"
#include "nxt_ttt/NT_TTT.h"
#include "nxt_ttt/NT_Helper.h"

#include <ros/ros.h>
#include <nxt_msgs/JointCommand.h>
#include <nxt_msgs/Color.h>
#include <nxt_msgs/Range.h>
#include <nxt_msgs/Contact.h>
#include <sensor_msgs/JointState.h>

#define BASE_EFF_INC 0.02f

namespace nxt_ttt
{

class Robot;

class BotBoard {
public:
    BotBoard();
    ~BotBoard();

    void                    updatePlatPos(double dPlatMotorPos);

    double                  getPlatRotation(int x, int y);
    double                  getSlideRotation(int x, int y);

private:
    double                  m_dPlatAngle;

    double                  m_dFirstPlatMotorPos;
    double                  m_dLastPlatMotorPos;
};

class MotorState {
public:
    MotorState(const std::string & sName = "", double dEffort = 0.0, double dPosition = 0.0, double dVelocity = 0.0);
    ~MotorState();

    const MotorState&       operator=(const MotorState & state);
    bool                    operator==(const MotorState & state);
    bool                    operator!=(const MotorState & state);

    void                    initFrom(const MotorState & state);
    bool                    compare(const MotorState & state);

    void                    setEffort(double dEffort)           { m_dEffort = dEffort;      }
    void                    setPosition(double dPosition)       { m_dPosition = dPosition;  }
    void                    setVelocity(double dVelocity)       { m_dVelocity = dVelocity;  }
    void                    setName(const std::string & sName)  { m_sName = sName;          }


    double                  getEffort()                   const { return m_dEffort;         }
    double                  getPosition()                 const { return m_dPosition;       }
    double                  getVelocity()                 const { return m_dVelocity;       }
    const std::string&      getName()                     const { return m_sName;           }

private:
    std::string             m_sName;

    double                  m_dPosition;
    double                  m_dEffort;
    double                  m_dVelocity;
}; // class MotorState

class Motor
{
private:
    Motor(const Motor&);
    const Motor& operator=(const Motor&);
public:
    Motor(ros::NodeHandle & nh, const std::string & sName, Robot * pParent = NULL);
    virtual ~Motor();

    bool                    update(const MotorState & state);
    bool                    rotate(double dRadAngle);
    void                    stop();

    void                    setName(const std::string & sName) { m_sName = sName;   }
    std::string             getName()                    const { return m_sName;    }

private:
    ros::Publisher          m_Publisher;
    nxt_msgs::JointCommand  m_Command;

    std::string             m_sName;

    float                   m_fPosDesi;
    float                   m_fPos;

    float                   m_fEffDesi;
    float                   m_fEff;

    float                   m_fVelocity;

    Robot*                  m_pParent;
    bool                    m_bHasGoal;

    void                    checkGoal();
    void                    publish();
}; // class Motor

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

    int                     m_nDesiX;
    int                     m_nDesiY;

    double                  m_dPlatOr;

    void                    stopAll();

    void                    motorCb(const sensor_msgs::JointState::ConstPtr & msg);
    void                    ultraCb(const nxt_msgs::Range::ConstPtr & msg);
    void                    colorCb(const nxt_msgs::Color::ConstPtr & msg);
    void                    contactCb(const nxt_msgs::Contact::ConstPtr & msg);

}; // class Robot

} // namespace nxt_ttt

#endif // NT_ROBOT_H
