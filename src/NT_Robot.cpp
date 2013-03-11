#include "nxt_ttt/NT_Robot.h"

using namespace nxt_ttt;

MotorState::MotorState(const std::string &sName, double dEffort, double dPosition, double dVelocity)
{
    m_sName     = sName;
    m_dPosition = dPosition;
    m_dEffort   = dEffort;
    m_dVelocity = dVelocity;
}

MotorState::~MotorState() { ; }

const MotorState& MotorState::operator =(const MotorState & state)
{
    initFrom(state);
    return *this;
}

bool MotorState::operator==(const MotorState & state) { return compare(state); }
bool MotorState::operator!=(const MotorState & state) { return !compare(state); }

void MotorState::initFrom(const MotorState &state)
{
    m_sName     = state.m_sName;
    m_dPosition = state.m_dPosition;
    m_dEffort   = state.m_dEffort;
    m_dVelocity = state.m_dVelocity;
}

bool MotorState::compare(const MotorState &state)
{
    if ((m_sName     == state.m_sName)
      &&(m_dPosition == state.m_dPosition)
      &&(m_dEffort   == state.m_dEffort)
      &&(m_dVelocity == state.m_dVelocity))
        return true;
    return false;
}

Motor::Motor(ros::NodeHandle &nh, const std::string & sName, Robot *pParent)
{
    LOG("Motor : " + sName + " Constructing\n");
    m_Publisher = nh.advertise<nxt_msgs::JointCommand>("joint_command", 1);
    m_sName     = sName;
    m_pParent   = pParent;
    m_bHasGoal  = false;
    m_fPosDesi  = 0.0;
    m_fPos      = 0.0;
    m_fEffDesi  = 0.0;
    m_fEff      = 0.0;
}

Motor::~Motor()
{
    LOG("INFO : Motor : " + m_sName + "Destructing\n");
    m_Command.effort = 0.0;
    m_Publisher.publish(m_Command);
}

bool Motor::update(const MotorState &state)
{
    bool bRet = false;

    if (m_sName == state.getName())
    {
        m_fPos  = state.getPosition();
        m_fEff  = state.getEffort();
        checkGoal();
        bRet = true;
    }

    return bRet;
}

bool Motor::rotate(double dRadAngle)
{
    bool bRet = false;

    if (!m_bHasGoal)
    {
        LOG("INFO : Motor : " + m_sName + " Setting goal.\n");
        m_bHasGoal  = true;
        m_fPosDesi  = m_fPos + dRadAngle;
        bRet = true;
    }
    else
        LOG("ERROR : Motor : " + m_sName + " Try to set goal but has already a goal.\n");

    return bRet;
}

void Motor::checkGoal()
{
    bool bNeedPub = false;

    if (m_bHasGoal)
    {

    }
    else
    {
        if (m_fEff != 0.0f)
        {
            LOG("ERROR : Motor : " + m_sName + " as no goal set but still runing.\n");
            if (m_fEffDesi != 0.0f)
                m_fEffDesi = 0.0f;
        }
    }

    if(bNeedPub)
        publish();
}

void Motor::publish()
{
    m_Command.name      = m_sName;
    m_Command.effort    = m_fEffDesi;
    m_Publisher.publish(m_Command);
}

Robot::Robot() :
    AbstractRobot(),
    m_PlatMotor(m_NodeHandle, "plat_motor", this),
    m_DropMotor(m_NodeHandle, "drop_motor", this),
    m_SlideMotor(m_NodeHandle, "slide_motor", this)
{
    LOG("INFO : Robot : Constructing\n");
}

Robot::~Robot()
{
    LOG("INFO : Robot : Destructing");
}

void Robot::getColor(int x, int y)
{

}

void Robot::dropBall(int x, int y)
{

}

void Robot::waitPlayerPlay()
{

}

void Robot::actionPerformed(const std::string &sMotorName)
{

}
