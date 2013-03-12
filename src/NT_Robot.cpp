#include "nxt_ttt/NT_Robot.h"

using namespace nxt_ttt;

const int           translateX = 1;
const int           translateY = 1;

const double        dropRotation = 0.0;

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

void BotBoard::updateSlidePos(double dSlideMotorPos)
{
    if (m_dFirstSlideMotorPos == 0.0)
        m_dFirstSlideMotorPos = dSlideMotorPos;
    m_dLastSlideMotorPos = dSlideMotorPos;
    m_dSlideAngle = m_dLastSlideMotorPos - m_dFirstSlideMotorPos;
}

double BotBoard::getPlatRotation(int x, int y)
{
    int     nX      = x - translateX,
            nY      = y - translateY;

    double  dAngle  = 2 * atan(nY / (nX + sqrt(pow(nX, 2) + pow(nY, 2))));

    return dAngle - m_dPlatAngle;
}

double BotBoard::getColorSlideRotation(int x, int y)
{
    int     nX      = x - translateX,
            nY      = y - translateY;

    return /* double dRad = */sqrt(pow(nX, 2) + pow(nY, 2)) - m_dSlideAngle;
}

double BotBoard::getDropSlideRotation(int x, int y)
{
    int     nX      = x - translateX,
            nY      = y - translateY;

    return /* double dRad = */sqrt(pow(nX, 2) + pow(nY, 2)) - m_dSlideAngle;
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
    if(m_CurrentAction == RA_NONE)
    {
        LOG("INFO : Robot : Started action : getting color\n");
        stopAll();
        m_CurrentAction = RA_GETCOLOR;
        m_BotBoard.updatePlatPos(m_PlatMotor.getPos());
        m_BotBoard.updateSlidePos(m_SlideMotor.getPos());

        rotatePlatMotor(m_BotBoard.getPlatRotation(x, y));
        rotateSlideMotor(m_BotBoard.getColorSlideRotation(x, y));
    }
    else //if (m_CurrentAction != RA_NONE)
        LOG("ERROR : Robot : The robot didn't finish his previous action.\n");
}

void Robot::dropBall(int x, int y)
{
    if (m_CurrentAction == RA_NONE)
    {
        LOG("INFO : Robot : Started action : dropping ball\n");
        stopAll();
        m_CurrentAction = RA_DROPBALL;
        m_BotBoard.updatePlatPos(m_PlatMotor.getPos());
        m_BotBoard.updateSlidePos(m_SlideMotor.getPos());


        rotatePlatMotor(m_BotBoard.getPlatRotation(x, y));
        rotateSlideMotor(m_BotBoard.getDropSlideRotation(x, y));
    }
    else // if (m_CurrentAction != RA_NONE)
        LOG("ERROR : Robot : The robot didn't finidh his previous action.\n");
}

void Robot::waitPlayerPlay()
{
    if (m_CurrentAction != RA_NONE)
        LOG("ERROR : Robot : The robot didn't finish his previous action.\n");
    stopAll();
    m_CurrentAction = RA_GETPLAYEREVENT;
}

void Robot::actionPerformed(const std::string &sMotorName)
{
    bool bDropped = false;

    if (((m_State & RS_TURNINGPLATMOTOR) == RS_TURNINGPLATMOTOR) && sMotorName == PLATNAME)
    {
        m_BotBoard.updatePlatPos(m_PlatMotor.getPos());
        m_State &= ~RS_TURNINGPLATMOTOR;
    }
    else if (((m_State & RS_TURNINGDROPMOTOR) == RS_TURNINGDROPMOTOR) && sMotorName == DROPNAME)
    {
        m_BotBoard.updatePlatPos(m_PlatMotor.getPos());
        m_State     &= ~RS_TURNINGDROPMOTOR;
        bDropped    = true;
    }
    else if (((m_State & RS_TURNINGSLIDEMOTOR) == RS_TURNINGSLIDEMOTOR) && sMotorName == SLIDENAME)
    {
        m_BotBoard.updatePlatPos(m_PlatMotor.getPos());
        m_State &= ~RS_TURNINGSLIDEMOTOR;
    }

    if(m_CurrentAction == RA_GETCOLOR)
    {
        if ((m_State & RS_TURNINGMASK) == RS_NONE)
        {
            LOG("INFO : Robot : wait color\n");
            m_State = m_State | RS_WAITINGCOLOR;
        }
    }
    else if (m_CurrentAction == RA_DROPBALL)
    {
        if ((m_State & RS_TURNINGMASK) == RS_NONE)
        {
            if (bDropped)
            {
                LOG("INFO : Robot : TTT Played\n");
                m_DropMotor.rollback();

                if (m_pApplication)
                    m_pApplication->cbPlayed(NT_PLAYER);

                m_CurrentAction = RA_NONE;
            }
            else // if(!bDropped)
                rotateDropMotor(dropRotation);
        }
    }
}

void Robot::stopAll()
{
    LOG("INFO : Robot : Stopping all motor.\n");
    m_SlideMotor.stop();
    m_PlatMotor.stop();
    m_DropMotor.stop();
}

void Robot::rotateDropMotor(double dRad)
{
    m_State = m_State | RS_TURNINGDROPMOTOR;
    m_DropMotor.rotate(dRad);
}

void Robot::rotatePlatMotor(double dRad)
{
    m_State = m_State | RS_TURNINGPLATMOTOR;
    m_PlatMotor.rotate(dRad);
}

void Robot::rotateSlideMotor(double dRad)
{
    m_State = m_State | RS_TURNINGSLIDEMOTOR;
    m_SlideMotor.rotate(dRad);
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
            LOG("INFO : Robot : Player Played\n");
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
        m_vColor.push_back(Color(msg->r, msg->g, msg->b));
        if (m_vColor.size() > 5)
        {
            LOG("INFO : Robot : Searching Color.\n");
            Color tempColor = NOCOLOR;
            for (size_t i = 0; i < m_vColor.size(); i ++)
            {
                if (m_vColor[i] != NOCOLOR)
                    tempColor = m_vColor[i];
            }

            m_pTTT->cbColor(m_nDesiX, m_nDesiY, tempColor);
            m_vColor.clear();
            m_State &= ~RS_WAITINGCOLOR;
        }
    }
}

void Robot::contactCb(const nxt_msgs::Contact::ConstPtr &msg)
{
    // todo
    if (msg->contact)
    {
        LOG("INFO : Robot : Board just made 360.\n");
    }
}
