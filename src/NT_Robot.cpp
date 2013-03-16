#include "nxt_ttt/NT_Robot.h"

#include <nxt_msgs/JointCommand.h>

using namespace nxt_ttt;

Robot::Robot() :
    AbstractRobot()
{
    LOG("INFO : Robot : Constructing\n");
    m_CurrentAction     = RA_NONE;
    m_State             = RS_NONE;

    m_pApplication      = NULL;
    m_pTTT              = NULL;
    m_pMotorAction      = NULL;
    m_bNeedRelease      = false;
    m_nDesiX            = 0;
    m_nDesiY            = 0;
    m_vColor.clear();

    m_MotorPub          = m_NodeHandle.advertise<nxt_msgs::JointCommand>("joint_command", 1);
    m_UltraSubscriber   = m_NodeHandle.subscribe(ULTRANAME, 1, &Robot::ultraCb, this);
    m_ColorSubscriber   = m_NodeHandle.subscribe(COLORNAME, 1, &Robot::colorCb, this);
    m_ContactSubscriber = m_NodeHandle.subscribe(CONTACTNAME, 1, &Robot::contactCb, this);
}

Robot::~Robot()
{
    LOG("INFO : Robot : Destructing\n");
}

void Robot::getColor(int x, int y)
{
    if (m_CurrentAction != RA_NONE)
        LOG("WARNING : Robot : GETCOLOR : The robot didn't finish his previous action.\n");

    LOG("INFO : Robot : Started action : getting color\n");
    m_CurrentAction = RA_GETCOLOR;
    m_State         = RS_TURNINGPLATMOTOR;
    m_nDesiX        = x;
    m_nDesiY        = y;

    m_pMotorAction  = new MotorAction(m_NodeHandle, PLATNAME);

    if (m_pMotorAction)
        m_pMotorAction->startDesi(m_BotBoard.getPlatRotation(x, y), 0.4, &m_MotorPub, this);
}

void Robot::dropBall(int x, int y)
{
    if (m_CurrentAction != RA_NONE)
        LOG("WARNING : Robot : DROPBALL : The robot didn't finish his previous action.\n");
    LOG("INFO : Robot : Started action : dropping ball\n");
    m_CurrentAction = RA_DROPBALL;
    m_State         = RS_TURNINGPLATMOTOR;
    m_nDesiX        = x;
    m_nDesiY        = y;

    m_pMotorAction  = new MotorAction(m_NodeHandle, PLATNAME);

    if (m_pMotorAction)
        m_pMotorAction->startDesi(m_BotBoard.getPlatRotation(x, y), 0.4, &m_MotorPub, this);
}

void Robot::waitPlayerPlay()
{
    if (m_CurrentAction != RA_NONE)
        LOG("WARNING : Robot : WAITPLAYER : The robot didn't finish his previous action.\n");
    m_CurrentAction = RA_GETPLAYEREVENT;
}

void Robot::actionPerformed(const std::string &sMotorName)
{
    m_bNeedRelease = true;
}

void Robot::ultraCb(const nxt_msgs::Range::ConstPtr &msg)
{
    if (m_CurrentAction == RA_GETPLAYEREVENT)
    {
        if (msg->range < 0.1)
        {
            LOG("INFO : Robot : Player Played\n");
            m_CurrentAction = RA_NONE;
            m_State         = RS_NONE;
            if (m_pApplication)
                m_pApplication->cbPlayed(NT_PLAYER);
        }
    }
}

void Robot::colorCb(const nxt_msgs::Color::ConstPtr &msg)
{
    if (m_CurrentAction == RA_GETCOLOR && m_State == RS_WAITINGCOLOR)
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

            m_CurrentAction = RA_NONE;
            m_State         = RS_NONE;
            m_pTTT->cbColor(m_nDesiX, m_nDesiY, tempColor);
            m_vColor.clear();
        }
    }
}

void Robot::contactCb(const nxt_msgs::Contact::ConstPtr &msg)
{
    if (m_bNeedRelease)
    {
        releaseMotorAction();
        if (m_CurrentAction == RA_GETCOLOR)
        {
            if (m_State == RS_TURNINGPLATMOTOR)
            {
                m_pMotorAction = new MotorAction(m_NodeHandle, SLIDENAME);

                if (m_pMotorAction)
                    m_pMotorAction->startDesi(m_BotBoard.getColorSlideRotation(m_nDesiX, m_nDesiY), 0.5, &m_MotorPub, this);
                m_State = RS_TURNINGSLIDEMOTOR;
            }
            else if (m_State == RS_TURNINGSLIDEMOTOR)
            {
                m_State = RS_WAITINGCOLOR;
            }
        }
        else if (m_CurrentAction == RA_DROPBALL)
        {
            if (m_State == RS_TURNINGPLATMOTOR)
            {
                m_pMotorAction = new MotorAction(m_NodeHandle, SLIDENAME);

                if (m_pMotorAction)
                    m_pMotorAction->startDesi(m_BotBoard.getDropSlideRotation(m_nDesiX, m_nDesiY), 0.5, &m_MotorPub, this);
                m_State = RS_TURNINGSLIDEMOTOR;
            }
            else if (m_State == RS_TURNINGSLIDEMOTOR)
            {
                m_pMotorAction = new MotorAction(m_NodeHandle, DROPNAME);

                if (m_pMotorAction)
                    m_pMotorAction->start(2 * M_PI, 0.5, &m_MotorPub, this);
                m_State = RS_TURNINGDROPMOTOR;
            }
            else if (m_State == RS_TURNINGDROPMOTOR)
            {
                m_State         = RS_NONE;
                m_CurrentAction = RA_NONE;
                if (m_pTTT)
                    m_pTTT->cbDropped(m_nDesiX, m_nDesiY);
            }
        }
    }
}

void Robot::releaseMotorAction()
{
    if (m_bNeedRelease)
        m_bNeedRelease = false;
    if (m_pMotorAction)
        delete m_pMotorAction;
    m_pMotorAction = NULL;
}
