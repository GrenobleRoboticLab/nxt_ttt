#include "nxt_ttt/NT_AbstractRobot.h"

using namespace nxt_ttt;

AbstractRobot::AbstractRobot() {
    LOG("INFO : AbstractRobot : Constructing\n");
    m_pApplication  = NULL;
    m_pTTT          = NULL;
}

AbstractRobot::~AbstractRobot() {
    LOG("INFO : AbstractRobot : Destructing\n");
}

void AbstractRobot::setApplication(Application *pApp) {
    m_pApplication = pApp;
}

void AbstractRobot::setTTT(AbstractTTT *pTTT) {
    m_pTTT = pTTT;
}
