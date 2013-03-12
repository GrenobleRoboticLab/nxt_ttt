#include "nxt_ttt/NT_TTT.h"

using namespace nxt_ttt;

TTT::TTT() : AbstractTTT()
{
    LOG("INFO : TTT : Constructing\n");
    m_bBoardEmpty = true;
    m_bFirstTime = true;
}

TTT::~TTT()
{
    LOG("INFO : TTT : Destructing\n");
}

void TTT::play()
{
    scan();
/*
    for (int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            m_pRobot->getColor(i, j);
            if(board[i][j]!=0)
            {
                boardEmpty == false;
            }
        }
    }

    if(boardEmpty)
    {
        AbstractRobot::dropBall(1,1);
    }
    else
    {
        if(firstTime)
        {
            if(board[1][1].isNotColor())
            {
                AbstractRobot::dropBall(1,1);
                firstTime == false;
            }
            else
            {
                AbstractRobot::dropBall(0,2);
                firstTime == false;
            }
        }
        else
        {
            if(!forceDrop())
            {
                bestDrop();
            }
        }
    }
    */
}

void TTT::cbColor(int x, int y, Color color)
{
    if (m_wCurrentState == TS_SCAN)
    {
        if (color.isColor())
            m_bBoardEmpty = false;

        m_Board[x][y] = color;
        continueScan();
    }
}

void TTT::cbDropped(int x, int y)
{
    if (m_wCurrentState == TS_DROP)
    {
        m_wCurrentState = TS_NONE;
        m_pApplication->cbPlayed(NT_TTT);
    }
}

TTT::CheckResult TTT::checkRow(int j)
{
    CheckResult ret;
    int playerColorCount    = 0,
        tttColorCount       = 0;

    for (int i = 0; i < 3; i++)
    {
        if (m_Board[i][j] == NOCOLOR)
        {
            ret.lastX = i;
            ret.lastY = j;
        }
        else if (m_Board[i][j] == PLAYERCOLOR)
            playerColorCount++;
        else
            tttColorCount++;
    }

    if ((tttColorCount + playerColorCount) == 3)
        ret.flag = CF_FREE;
    else if (tttColorCount == 2)
        ret.flag = CF_CHOICE;
    else if (playerColorCount == 2)
        ret.flag = CF_WARNING;
    else if (tttColorCount == 3)
        ret.flag = CF_WIN;
    else if (playerColorCount == 3)
        ret.flag = CF_LOSE;

    return ret;
}

TTT::CheckResult TTT::checkCol(int i)
{
    CheckResult ret;
    int playerColorCount    = 0,
        tttColorCount       = 0;

    for (int j = 0; j < 3; j++)
    {
        if (m_Board[i][j] == NOCOLOR)
        {
            ret.lastX = i;
            ret.lastY = j;
        }
        else if (m_Board[i][j] == PLAYERCOLOR)
            playerColorCount++;
        else
            tttColorCount++;
    }

    if ((tttColorCount + playerColorCount) == 3)
        ret.flag = CF_FREE;
    else if (tttColorCount == 2)
        ret.flag = CF_CHOICE;
    else if (playerColorCount == 2)
        ret.flag = CF_WARNING;
    else if (tttColorCount == 3)
        ret.flag = CF_WIN;
    else if (playerColorCount == 3)
        ret.flag = CF_LOSE;

    return ret;
}

TTT::CheckResult TTT::checkDiag(int i)
{
    CheckResult ret;
    int playerColorCount    = 0,
        tttColorCount       = 0;

    if(m_Board[1][1] == NOCOLOR)
    {
        ret.lastX = 1;
        ret.lastY = 1;
    }
    else if(m_Board[1][1] == PLAYERCOLOR)
        playerColorCount++;
    else
        tttColorCount++;

    if(i == 0)
    {
        if(m_Board[0][0] == NOCOLOR)
        {
            ret.lastX = 0;
            ret.lastY = 0;
        }
        else if(m_Board[0][0] == PLAYERCOLOR)
            playerColorCount++;
        else
            tttColorCount++;

        if(m_Board[2][2] == NOCOLOR)
        {
            ret.lastX = 2;
            ret.lastY = 2;
        }
        else if(m_Board[2][2] == PLAYERCOLOR)
            playerColorCount++;
        else
            tttColorCount++;
    }
    else if(i == 2)
    {
        if(m_Board[2][0] == NOCOLOR)
        {
            ret.lastX = 2;
            ret.lastY = 0;
        }
        else if(m_Board[2][0] == PLAYERCOLOR)
            playerColorCount++;
        else
            tttColorCount++;

        if(m_Board[0][2] == NOCOLOR)
        {
            ret.lastX = 0;
            ret.lastY = 2;
        }
        else if(m_Board[0][2] == PLAYERCOLOR)
            playerColorCount++;
        else
            tttColorCount++;
    }

    if ((tttColorCount + playerColorCount) == 3)
        ret.flag = CF_FREE;
    else if (tttColorCount == 2)
        ret.flag = CF_CHOICE;
    else if (playerColorCount == 2)
        ret.flag = CF_WARNING;
    else if (tttColorCount == 3)
        ret.flag = CF_WIN;
    else if (playerColorCount == 3)
        ret.flag = CF_LOSE;

    return ret;
}

void TTT::scan()
{
    m_wCurrentState = TS_SCAN;
    m_nCurrentX     = 0;
    m_nCurrentY     = 0;
    m_pRobot->getColor(m_nCurrentX, m_nCurrentY);
}

void TTT::continueScan()
{
    if (m_wCurrentState == TS_SCAN)
    {
        m_nCurrentY++;
        if (m_nCurrentY < 3)
            m_pRobot->getColor(m_nCurrentX, m_nCurrentY);
        else {
            m_nCurrentX ++;
            m_nCurrentY = 0;
            if (m_nCurrentX < 3)
                m_pRobot->getColor(m_nCurrentX, m_nCurrentY);
            else
                treat();
        }
    }
}

void TTT::treat()
{
    m_wCurrentState = TS_DROP;

    if(m_bBoardEmpty)
    {
        m_pRobot->dropBall(1,1);
        return;
    }
    else
    {
        CheckResult result;

        for(int i=0; i<3; i++)
        {
            result = checkCol(i);
            if (applyResult(result))
                return;

            result = checkRow(i);
            if (applyResult(result))
                return;
        }

        result = checkDiag(0);
        if (applyResult(result))
            return;

        result = checkDiag(2);
        if (applyResult(result))
            return;
    }
    bestDrop();
}

bool TTT::applyResult(CheckResult check)
{
    bool bRet = false;

    if(check.flag != CF_FREE)
    {
        bRet = true;
        if (check.flag == CF_WIN)
            m_pApplication->cbEnd(NT_TTT);
        else if (check.flag == CF_LOSE)
            m_pApplication->cbEnd(NT_PLAYER);
        else
        {
            m_wCurrentState = TS_NONE;
            m_pRobot->dropBall(check.lastX, check.lastY);
        }
    }

    return bRet;
}

void TTT::bestDrop()
{
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(m_Board[i][j] == PLAYERCOLOR)
            {
                int bestX, bestY, oppX, oppY;

                if(i==0 && j==0) { oppX = 2; oppY = 2; }
                else if(i==1 && j==0) { oppX = 1; oppY = 2; }
                else if(i==2 && j==0) { oppX = 0; oppY = 2; }
                else if(i==0 && j==1) { oppX = 2; oppY = 1; }
                else if(i==2 && j==1) { oppX = 0; oppY = 1; }
                else if(i==0 && j==2) { oppX = 2; oppY = 0; }
                else if(i==1 && j==2) { oppX = 1; oppY = 0; }
                else if(i==2 && j==2) { oppX = 0; oppY = 0; }

                do
                {
                    bestX = rand() % 3;
                    bestY = rand() % 3;
                }
                while( ( bestX==1 && bestY==1) || (bestX==i && bestY==j) || (bestX==oppX && bestY==oppY) );

                m_pRobot->dropBall(bestX, bestY);
                return;
            }
        }
    }
}
