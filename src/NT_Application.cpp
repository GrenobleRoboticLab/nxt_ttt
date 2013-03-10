#include "nxt_ttt/NT_Application.h"

using namespace nxt_ttt;

Application::Application() {
    m_pTTT = NULL;
}

Application::~Application() {}

void Application::start() {
    ros::spin();
}

void Application::stop() {
    ros::shutdown();
}
void Application::setTTT(AbstractTTT *pTTT) {
    m_pTTT = pTTT;
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
