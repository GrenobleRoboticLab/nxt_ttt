#include "nxt_ttt/NT_Robot.h"

using namespace nxt_ttt;

const int           translateX = 1;
const int           translateY = 1;

const std::string   PLATNAME("plat_motor");
const std::string   SLIDENAME("slide_motor");
const std::string   DROPNAME("drop_motor");
const std::string   ULTRANAME("ultra_sensor");
const std::string   COLORNAME("color_sensor");
const std::string   CONTACTNAME("contact_sensor");

BotBoard::BotBoard() { ; }
BotBoard::~BotBoard() { ; }

void BotBoard::updatePlatPos(double dPlatMotorPos)
{
    if (m_dFirstPlatMotorPos == 0.0)
        m_dFirstPlatMotorPos = dPlatMotorPos;
    m_dLastPlatMotorPos = dPlatMotorPos;
    m_dPlatAngle = m_dLastPlatMotorPos - m_dFirstPlatMotorPos;
}

double BotBoard::getPlatRotation(int x, int y)
{
    int     nX      = x - translateX,
            nY      = y - translateY;

    double  dAngle  = 2 * atan(nY / (nX + sqrt(pow(nX, 2) + pow(nY, 2))));

    return dAngle - m_dPlatAngle;
}

double BotBoard::getSlideRotation(int x, int y)
{
    int     nX      = x - translateX,
            nY      = y - translateY;

    return /* double dRad = */sqrt(pow(nX, 2) + pow(nY, 2));
}

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

void Motor::stop()
{
    LOG("INFO : Motor : " + m_sName + "Ask for stopping.");
    m_fEffDesi = 0.0f;
    publish();
}

void Motor::checkGoal()
{
    bool bNeedPub = false;

    if (m_bHasGoal)
    {
        std::cout << "Effort : " << m_fEff << " Effort desi = " << m_fEffDesi << std::endl;
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
            std::cout << "Position desi = " << m_fPosDesi << " \t Position = " << m_fPos << std::endl;
            if ((m_fEffDesi > 0)
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

Robot::Robot() :
    AbstractRobot(),
    m_PlatMotor(m_NodeHandle, PLATNAME, this),
    m_DropMotor(m_NodeHandle, DROPNAME, this),
    m_SlideMotor(m_NodeHandle, SLIDENAME, this)
{
    LOG("INFO : Robot : Constructing\n");
    m_CurrentAction     = RA_NONE;

    m_MotorSubscriber   = m_NodeHandle.subscribe("joint_state", 1, &Robot::motorCb, this);
    m_UltraSubscriber   = m_NodeHandle.subscribe(ULTRANAME, 1, &Robot::ultraCb, this);
    m_ColorSubscriber   = m_NodeHandle.subscribe(COLORNAME, 1, &Robot::colorCb, this);
    m_ContactSubscriber = m_NodeHandle.subscribe(CONTACTNAME, 1, &Robot::contactCb, this);
}

Robot::~Robot()
{
    LOG("INFO : Robot : Destructing\n");
    stopAll();
}

void Robot::getColor(int x, int y)
{
    if (m_CurrentAction != RA_NONE)
        LOG("ERROR : Robot : The robot didn't finidh his previous action.");
    stopAll();
    m_CurrentAction = RA_GETCOLOR;

    // rotate platMotor & slideMotor
}

void Robot::dropBall(int x, int y)
{
    if (m_CurrentAction != RA_NONE)
        LOG("ERROR : Robot : The robot didn't finidh his previous action.");
    stopAll();
    m_CurrentAction = RA_DROPBALL;

    // rotate platMotor & slideMotor
}

void Robot::waitPlayerPlay()
{
    if (m_CurrentAction != RA_NONE)
        LOG("ERROR : Robot : The robot didn't finidh his previous action.");
    stopAll();
    m_CurrentAction = RA_GETPLAYEREVENT;
}

void Robot::actionPerformed(const std::string &sMotorName)
{
    if(m_CurrentAction == RA_GETCOLOR)
    {
        // todo
    }
    else if (m_CurrentAction == RA_DROPBALL)
    {
        // todo
    }
}

void Robot::stopAll()
{
    LOG("INFO : Robot : Stopping all motor.");
    m_SlideMotor.stop();
    m_PlatMotor.stop();
    m_DropMotor.stop();
}

void Robot::motorCb(const sensor_msgs::JointState::ConstPtr &msg)
{
    if (m_CurrentAction != RA_GETPLAYEREVENT && m_CurrentAction != RA_NONE)
    {
        if (((m_State & RS_TURNINGPLATMOTOR) == RS_TURNINGPLATMOTOR)
          && (msg->name.back() == PLATNAME))
            m_PlatMotor.update(MotorState(msg->name.back(), msg->effort.back(), msg->position.back(), msg->velocity.back()));

        if (((m_State & RS_TURNINGSLIDEMOTOR) == RS_TURNINGSLIDEMOTOR)
          && (msg->name.back() == SLIDENAME))
            m_SlideMotor.update(MotorState(msg->name.back(), msg->effort.back(), msg->position.back(), msg->velocity.back()));

        if (((m_State & RS_TURNINGDROPMOTOR) == RS_TURNINGDROPMOTOR)
          && (msg->name.back() == DROPNAME))
            m_DropMotor.update(MotorState(msg->name.back(), msg->effort.back(), msg->position.back(), msg->velocity.back()));
    }
}

void Robot::ultraCb(const nxt_msgs::Range::ConstPtr &msg)
{
    if (m_CurrentAction == RA_GETPLAYEREVENT)
    {
        if (msg->range < 0.2)
        {
            if (m_pApplication)
                m_pApplication->cbPlayed(NT_PLAYER);
            m_CurrentAction = RA_NONE;
        }
    }
}

void Robot::colorCb(const nxt_msgs::Color::ConstPtr &msg)
{
    if ((m_State & RS_WAITINGCOLOR) == RS_WAITINGCOLOR)
    {
        m_pTTT->cbColor(m_nDesiX, m_nDesiY, Color(msg->r, msg->g, msg->b));
        m_State = RS_NONE;
    }
}

void Robot::contactCb(const nxt_msgs::Contact::ConstPtr &msg)
{
    // todo
    if (msg->contact)
    {
    }
}
