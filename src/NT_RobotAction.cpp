#include "nxt_ttt/NT_RobotAction.h"

using namespace nxt_ttt;


MotorAction::MotorAction(ros::NodeHandle &nh, const std::string &sName)
{
    m_Sub   = nh.subscribe("joint_state", 1, &MotorAction::motorCallback, this);
    m_sName = sName;

    m_pRobot    = NULL;
    m_pPub      = NULL;

    m_bIsInit   = false;
    m_bStarted  = false;
    m_bEnded    = false;

    m_dBaseEffort   = 0.0;
    m_dDesiEffort   = 0.0;
    m_dSendedEffort = 0.0;
}

MotorAction::~MotorAction()
{
    m_dDesiEffort = 0.0;
    sendCommand();
}

void MotorAction::start(double dDesiRotation, double dBaseEffort, ros::Publisher* pPub, AbstractRobot *pRobot)
{
    m_bNeedCompute  = true;
    m_dDesiRotation = dDesiRotation;
    m_dBaseEffort   = (dBaseEffort > 0) ? dBaseEffort : -dBaseEffort;
    m_pRobot        = pRobot;
    m_pPub          = pPub;

    m_bStarted      = true;
}

void MotorAction::startDesi(double dDesiPos, double dBaseEffort, ros::Publisher* pPub, AbstractRobot *pRobot)
{
    m_bNeedCompute  = false;
    m_dDesiPos      = dDesiPos;
    m_dBaseEffort   = (dBaseEffort > 0) ? dBaseEffort : -dBaseEffort;
    m_pRobot        = pRobot;
    m_pPub          = pPub;

    m_bStarted      = true;
}

void MotorAction::motorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if (msg->name.back() == m_sName)
    {
        if (!m_bIsInit)
        {
            if (msg->velocity.back() == 0.0)
            {
                m_dFirstPos = msg->position.back();
                m_bIsInit   = true;
                computeDesiPos();
            }
        }
        else if (m_bEnded)
        {
            if (msg->velocity.back() == 0)
            {
                m_dLastPos = msg->position.back();
                m_pRobot->actionPerformed(m_sName);
            }
        }
        else
        {
            if ((msg->position.back() < m_dDesiPos + 0.1) && (msg->position.back() > m_dDesiPos - 0.1))
            {
                m_dDesiEffort = 0.0;
                sendCommand();
                m_bEnded = true;
            }
            else if (msg->velocity.back() > 2.5 && msg->effort.back() < m_dDesiEffort + 0.1 && msg->effort.back() > m_dDesiEffort - 0.05)
            {
                m_dDesiEffort += (m_dDesiEffort > 0) ? -0.001 : 0.001;
                sendCommand();
            }
            else if (msg->velocity.back() < 1.0 && msg->effort.back() < m_dDesiEffort + 0.1 && msg->effort.back() > m_dDesiEffort - 0.05)
            {
                m_dDesiEffort += (m_dDesiEffort < 0) ? -0.001 : 0.001;
                sendCommand();
            }
            else sendCommand();
        }
    }
}

void MotorAction::computeDesiPos()
{
    if (m_bNeedCompute)
    {
        m_dDesiPos = m_dFirstPos + m_dDesiRotation;
    }

    if ((m_dFirstPos - m_dDesiPos) > 0)
        m_dDesiEffort = -m_dBaseEffort;
    else
        m_dDesiEffort = m_dBaseEffort;
}

void MotorAction::sendCommand()
{
        m_CurrentCommand.name = m_sName;
        m_CurrentCommand.effort = m_dDesiEffort;
        if (m_pPub)
            m_pPub->publish(m_CurrentCommand);
        m_dSendedEffort = m_dDesiEffort;
}
