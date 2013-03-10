#include "nxt_ttt/NT_AbstractTTT.h"

using namespace nxt_ttt;

AbstractTTT::AbstractTTT() {
    m_pApplication  = NULL;
    m_pRobot        = NULL;
}

AbstractTTT::~AbstractTTT() {
}

void AbstractTTT::setApplication(Application *pApp) {
    m_pApplication = pApp;
}

void AbstractTTT::setRobot(AbstractRobot *pRobot)
{
    m_pRobot = pRobot;
}
