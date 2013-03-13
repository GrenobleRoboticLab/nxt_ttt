#include "nxt_ttt/NT_Motor.h"
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
    LOG("INFO : Motor : " + sName + " Constructing\n");
    m_Publisher = nh.advertise<nxt_msgs::JointCommand>("joint_command", 1);
    m_sName     = sName;
    m_pParent   = pParent;
    m_bHasGoal  = false;
    m_fPosDesi  = 0.0f;
    m_fPos      = 0.0f;
    m_fEffDesi  = 0.0f;
    m_fEff      = 0.0f;
    m_fVelocity = 0.0f;
}

Motor::~Motor()
{
    LOG("INFO : Motor : " + m_sName + " Destructing\n");
    m_fEffDesi = 0.0f;
    publish();
}

bool Motor::update(const MotorState &state)
{
    bool bRet = false;

    if (m_sName == state.getName())
    {
        m_fPos      = state.getPosition();
        m_fEff      = state.getEffort();
        m_fVelocity = state.getVelocity();
        bRet        = true;

        checkGoal();
    }

    return bRet;
}

bool Motor::rotate(double dRadAngle)
{
    bool bRet = false;

    if (!m_bHasGoal)
    {
        LOG("INFO : Motor : " + m_sName + " Setting goal.\n");
        m_bHasGoal      = true;
        m_fPosDesi      = m_fPos + dRadAngle;
        m_dLastRotation = dRadAngle;
        bRet            = true;
    }
    else
        LOG("ERROR : Motor : " + m_sName + " Try to set goal but has already a goal.\n");

    return bRet;
}

bool Motor::rollback()
{
    return rotate(-m_dLastRotation);
}

void Motor::stop()
{
    LOG("INFO : Motor : " + m_sName + "Ask for stopping.\n");
    m_fEffDesi = 0.0f;
    publish();
}

void Motor::checkGoal()
{
    bool bNeedPub = false;

    if (m_bHasGoal)
    {
        if (m_fVelocity == 0.0 && (m_fEff == m_fEffDesi))
        {
            if (m_fEffDesi == 0.0)
                m_fEffDesi = 0.47f;

            LOG("INFO : Motor : " + m_sName + " Increase effort.\n");
            if (m_fPosDesi > m_fPos)
                m_fEffDesi += BASE_EFF_INC;
            else
                m_fEffDesi -= BASE_EFF_INC;

            bNeedPub    = true;
        }
        else // if (abs(m_fVelocity) > 0)
        {
            if ((m_fEffDesi >= 0)
             && (m_fPos < (m_fPosDesi + .2f))
             && (m_fPos > (m_fPosDesi - .2)))
            {
                LOG("INFO : Motor : "+ m_sName + " Goal reach\n");
                m_fEffDesi  = 0.0f;
                m_bHasGoal  = false;
                bNeedPub    = true;
                m_pParent->actionPerformed(m_sName);
            }
        }
    }
    else
    {
        if (m_fEff != 0.0f)
        {
            bNeedPub = true;
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
    LOG("INFO : Motor : " + m_sName + "Publishing\n");
    m_Publisher.publish(m_Command);
}
