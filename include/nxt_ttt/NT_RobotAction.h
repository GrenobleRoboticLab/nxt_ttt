#ifndef NT_ROBOTACTION_H
#define NT_ROBOTACTION_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nxt_msgs/JointCommand.h>
#include "NT_AbstractRobot.h"

namespace nxt_ttt {

class MotorAction {
public:
    MotorAction(ros::NodeHandle & nh, const std::string & sName);
    ~MotorAction();

    void                    start(double dDesiRotation, double dBaseEffort, ros::Publisher * pPublisher, AbstractRobot * pRobot);
    void                    startDesi(double dDesiPos, double dBaseEffort, ros::Publisher * pPublisher, AbstractRobot * pRobot);
    double                  getLastPos() { return m_dLastPos; }

private:
    ros::Subscriber         m_Sub;
    ros::Publisher*         m_pPub;

    double                  m_dDesiRotation;

    double                  m_dFirstPos;
    double                  m_dLastPos;
    double                  m_dDesiPos;

    double                  m_dBaseEffort;
    double                  m_dDesiEffort;
    double                  m_dSendedEffort;

    bool                    m_bIsInit;
    bool                    m_bStarted;
    bool                    m_bEnded;
    bool                    m_bNeedCompute;

    std::string             m_sName;

    AbstractRobot*          m_pRobot;

    nxt_msgs::JointCommand  m_CurrentCommand;

    void                    motorCallback(const sensor_msgs::JointState::ConstPtr & msg);
    void                    computeDesiPos();
    void                    sendCommand();

}; // class MotorAction

} // namespace nxt_ttt

#endif // NT_ROBOTACTION_H
