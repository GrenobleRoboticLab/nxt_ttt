#include "nxt_ttt/NT_AbstractTTT.h"

using namespace nxt_ttt;

AbstractTTT::AbstractTTT() {
    m_pApplication  = NULL;
    boardEmpty = true;
    firstTime = true;
}

AbstractTTT::~AbstractTTT() {
}

void AbstractTTT::setApplication(Application *pApp) {
    m_pApplication = pApp;
}
