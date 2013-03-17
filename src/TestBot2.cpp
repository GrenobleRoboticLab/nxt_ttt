#include "TestBot2.h"

using namespace nxt_ttt;

TestBot2::TestBot2() : AbstractRobot()
{
    m_Sub       = m_nh.subscribe("contact_sensor", 1, &TestBot2::contactCb, this);
    m_Pub       = m_nh.advertise<nxt_msgs::JointCommand>("joint_command", 1);
    m_pAction   = NULL;
}

void TestBot2::actionPerformed(const std::string &sMotorName)
{
    m_bNeedReleaseAction = true;
}

void TestBot2::ask()
{
    int     nRobotChoose    = 0;
    double  dPosDesi        = 0.0;

    std::cout << "Tester quel moteur : " << std::endl << "[0] : plat_motor [1] : drop_motor [2] : slide_motor" << std::endl;
    std::cin >> nRobotChoose;

    if (nRobotChoose == 0)
    {
        std::cout << "Vous avez choisi le moteur : plat_motor" << std::endl;
        m_pAction = new MotorAction(m_nh, plat);
    }
    else if (nRobotChoose == 1)
    {
        std::cout << "Vous avez choisi le moteur : drop_motor" << std::endl;
        m_pAction = new MotorAction(m_nh, drop);
    }
    else if (nRobotChoose == 2)
    {
        std::cout << "Vous avez choisi le moteur : slide_motor" << std::endl;
        m_pAction = new MotorAction(m_nh, slide);
    }

    std::cout << "Quel angle ?" << std::endl;
    std::cin >> dPosDesi;
    std::cout << "Angle : " << dPosDesi << std::endl;

    if (m_pAction)
    {
        m_bNeedReleaseAction    = false;
        std::cout << "En attente..." << std::endl;
        m_pAction->start(dPosDesi, 0.5, &m_Pub, this);
    }
    else ask();
}

void TestBot2::releaseAction()
{
    if (m_pAction)
        delete m_pAction;
    m_pAction = NULL;
    m_bNeedReleaseAction = false;
}

void TestBot2::contactCb(const nxt_msgs::Contact::ConstPtr &msg)
{
    if (m_bNeedReleaseAction)
        releaseAction();
    else if (!m_pAction)
        ask();
}
