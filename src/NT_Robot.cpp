#include "nxt_ttt/NT_Robot.h"

using namespace nxt_ttt;

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
            m_State |= RS_WAITINGCOLOR;
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
                rotateDropMotor(DROP_ROTATION);
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
    m_State |= RS_TURNINGDROPMOTOR;
    m_DropMotor.rotate(dRad);
}

void Robot::rotatePlatMotor(double dRad)
{
    m_State |= RS_TURNINGPLATMOTOR;
    m_PlatMotor.rotate(dRad);
}

void Robot::rotateSlideMotor(double dRad)
{
    m_State |= RS_TURNINGSLIDEMOTOR;
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
    if (m_CurrentAction == RA_GETCOLOR && (m_State & RS_WAITINGCOLOR) == RS_WAITINGCOLOR)
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
            m_CurrentAction = RA_NONE;
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
