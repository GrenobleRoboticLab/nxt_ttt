#include "nxt_ttt/NT_AbstractTTT.h"

using namespace nxt_ttt;

AbstractTTT::AbstractTTT() {
    m_pApplication = NULL;
}

AbstractTTT::~AbstractTTT() {
}

void AbstractTTT::setApplication(Application *pApp) {
    m_pApplication = pApp;
}

void AbstractTTT::cbColor(int x, int y, int color) {
    m_nBoard[x][y] = color;
}

void AbstractTTT::cbDropped(int x, int y) {
}
