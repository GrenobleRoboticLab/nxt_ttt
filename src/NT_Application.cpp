#include "nxt_ttt/NT_Application.h"

using namespace nxt_ttt;

Application::Application() {
    LOG("INFO : Application : Constructing\n");
    m_pTTT      = NULL;
    m_pRobot    = NULL;
}

Application::~Application()
{
    LOG("INFO : Application : Destructing\n");
}

void Application::start() {
    LOG("INFO : Application : Calling ros::spin\n");
    m_pTTT->play();
    ros::spin();
}

void Application::stop() {
    LOG("INFO : Application : Calling ros::shutdown\n");
    ros::shutdown();
}
void Application::setTTT(AbstractTTT *pTTT) {
    LOG("INFO : Application : Setting ttt pointer\n");
    m_pTTT = pTTT;
    m_pTTT->setApplication(this);
}

void Application::setRobot(AbstractRobot *pRobot)
{
    LOG("INFO : Application : Setting robot pointer\n");
    m_pRobot = pRobot;
    m_pRobot->setApplication(this);
}

void Application::cbPlayed(NT_USER user)
{
    if (user == NT_PLAYER)
    {
        if (m_pTTT)
        {
            m_pTTT->play();
            std::cout << "Waiting..." << std::endl;
        }
        else
        {
            std::cout << "No brain found... Quitting" << std::endl;
            stop();
        }
    }
    else // if(user == NT_TTT
    {
        if (m_pRobot)
            m_pRobot->waitPlayerPlay();
        std::cout << "Your turn !" << std::endl;
    }
}

void Application::cbEnd(NT_USER winner)
{
    if (winner == NT_TTT)
        std::cout << "You lose !" << std::endl;
    else std::cout << "You win !" << std::endl;
}
