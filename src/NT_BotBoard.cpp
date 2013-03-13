#include <cmath>
#include "nxt_ttt/NT_BotBoard.h"
#include "nxt_ttt/NT_Helper.h"

using namespace nxt_ttt;

BotBoard::BotBoard() { ; }
BotBoard::~BotBoard() { ; }

void BotBoard::updatePlatPos(double dPlatMotorPos)
{
    if (m_dFirstPlatMotorPos == 0.0)
        m_dFirstPlatMotorPos = dPlatMotorPos;
    m_dLastPlatMotorPos = dPlatMotorPos;
    m_dPlatAngle = m_dLastPlatMotorPos - m_dFirstPlatMotorPos;
}

void BotBoard::updateSlidePos(double dSlideMotorPos)
{
    if (m_dFirstSlideMotorPos == 0.0)
        m_dFirstSlideMotorPos = dSlideMotorPos;
    m_dLastSlideMotorPos = dSlideMotorPos;
    m_dSlideAngle = m_dLastSlideMotorPos - m_dFirstSlideMotorPos;
}

double BotBoard::getPlatRotation(int x, int y)
{
    int     nX      = x - TRANSLATE_X,
            nY      = y - TRANSLATE_Y;
    double  dFac    = (nX + sqrt(pow(nX, 2) + pow(nY, 2)));

    double  dAngle  = 0.0;

    if (dFac != 0)
        dAngle = 2 * atan(nY / dFac);

    return dAngle - m_dPlatAngle;
}

double BotBoard::getColorSlideRotation(int x, int y)
{
    int     nX      = x - TRANSLATE_X,
            nY      = y - TRANSLATE_Y;

    return 0.2 * (sqrt(pow(nX, 2) + pow(nY, 2)) - m_dSlideAngle);
}

double BotBoard::getDropSlideRotation(int x, int y)
{
    int     nX      = x - TRANSLATE_X,
            nY      = y - TRANSLATE_Y;

    return (0.2 * (sqrt(pow(nX, 2) + pow(nY, 2)) - m_dSlideAngle)) + 1.57;
}
