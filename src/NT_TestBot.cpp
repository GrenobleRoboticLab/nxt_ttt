#include "NT_TestBot.h"

using namespace nxt_ttt;

TestBot::TestBot() :
    AbstractRobot()
{
    m_PlatMotor = new Motor(m_nh, plat, this);
    m_DropMotor = new Motor(m_nh, drop, this);
    m_SlideMotor = new Motor(m_nh, slide, this);
    m_bNeedDisplay  = false;
    m_MSub          = m_nh.subscribe("joint_state", 1, &TestBot::motorCb, this);
}

void TestBot::motorCb(const sensor_msgs::JointState::ConstPtr & msg)
{
std::cout << m_sCurrentMotor << std::endl;

    if (msg->name.size() > 1)
        std::cout << "Plus d'un JointState dans le message." << std::endl;

    if (msg->name.back() == m_sCurrentMotor)
    {
        std::cout << "Message inc" << std::endl;
        if (m_bNeedDisplay)
        {
            if (msg->velocity.back() == 0.0)
            {
                std::cout << "Dernière position du mouvement : " << msg->position.back() << std::endl;
                m_bNeedDisplay = false;
                ask();
            }
        }
        else if (m_DropMotor->getName() == m_sCurrentMotor) {
            std::cout << "Message inc DROP" << std::endl;
            m_DropMotor->update(MotorState(msg->name.back(), msg->effort.back(), msg->position.back(), msg->velocity.back()));
        }
        else if (m_SlideMotor->getName() == m_sCurrentMotor) {
            std::cout << "Message inc SLIDE" << std::endl;
            m_SlideMotor->update(MotorState(msg->name.back(), msg->effort.back(), msg->position.back(), msg->velocity.back()));
        }
        else if (m_PlatMotor->getName() == m_sCurrentMotor) {
            std::cout << "Message inc PLAT" << std::endl;
            m_PlatMotor->update(MotorState(msg->name.back(), msg->effort.back(), msg->position.back(), msg->velocity.back()));
        }
        std::cout << "Message inc" << std::endl;
    }
}

void TestBot::ask()
{
    int     nRobotChoose    = 0;
    double  dPosDesi        = 0.0;
    Motor*  pMotorChoose    = NULL;

    std::cout << "Tester quel moteur : " << std::endl << "[0] : plat_motor [1] : drop_motor [2] : slide_motor" << std::endl;
    std::cin >> nRobotChoose;

    if (nRobotChoose == 0)
    {
        pMotorChoose = m_PlatMotor;
        std::cout << "Vous avez choisi le moteur : plat_motor" << std::endl;
        m_sCurrentMotor = plat;
    }
    else if (nRobotChoose == 1)
    {
        pMotorChoose = m_DropMotor;
        std::cout << "Vous avez choisi le moteur : drop_motor" << std::endl;
        m_sCurrentMotor = drop;
    }
    else if (nRobotChoose == 2)
    {
        pMotorChoose = m_SlideMotor;
        std::cout << "Vous avez choisi le moteur : slide_motor" << std::endl;
        m_sCurrentMotor = slide;
    }

    std::cout << "Quel angle ?" << std::endl;
    std::cin >> dPosDesi;
    std::cout << "Angle : " << dPosDesi << std::endl;

    if (pMotorChoose)
    {
        std::cout << "En attente..." << std::cout;
        pMotorChoose->rotate(dPosDesi);
    }
    else ask();
    //std::flush(std::cout);
}

void TestBot::actionPerformed(const std::string &sMotorName)
{
    if (sMotorName == m_sCurrentMotor)
    {
        m_bNeedDisplay = true;
    }
}

void TestBot::dropBall(int x, int y)    { std::cout << "Méthode non implémentée." << std::endl; }
void TestBot::getColor(int x, int y)    { std::cout << "Méthode non implémentée." << std::endl; }
void TestBot::waitPlayerPlay()          { std::cout << "Méthode non implémentée." << std::endl; }
