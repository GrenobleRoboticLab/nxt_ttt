#include "nxt_ttt/NT_TTT.h"

using namespace nxt_ttt;

void TTT::play()
{

    for (int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            AbstractRobot::getColor(i, j);play;
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
            if(board[1][1]==0)
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
}

void TTT::cbColor(int x, int y, Color *color)
{
    board[x][y] == color;
}

bool TTT::forceDrop()
{
    if(board[0][0]==board[1][0] && board[2][0]==0)
    {
        AbstractRobot::dropBall(2,0);
        return true;
    }
    else if(board[1][0]==board[2][0] && board[0][0]==0)
    {
        AbstractRobot::dropBall(0,0);
        return true;
    }
    else if(board[0][1]==board[1][0] && board[2][1]==0)
    {
        AbstractRobot::dropBall(2,1);
        return true;
    }
    else if(board[1][1]==board[2][1] && board[0][1]==0)
    {
        AbstractRobot::dropBall(0,1);
        return true;
    }
    else if(board[0][2]==board[1][2] && board[2][2]==0)
    {
        AbstractRobot::dropBall(2,2);
        return true;
    }
    else if(board[1][2]==board[2][2] && board[0][2]==0)
    {
        AbstractRobot::dropBall(0,2);
        return true;
    }
    else if(board[0][0]==board[0][1] && board[0][2]==0)
    {
        AbstractRobot::dropBall(0,2);
        return true;
    }
    else if(board[0][2]==board[0][1] && board[0][0]==0)
    {
        AbstractRobot::dropBall();
        return true;
    }
    else if(board[1][0]==board[1][1] && board[1][2]==0)
    {
        AbstractRobot::dropBall(1,2);
        return true;
    }
    else if(board[1][2]==board[1][1] && board[1][0]==0)
    {
        AbstractRobot::dropBall(1,0);
        return true;
    }
    else if(board[2][2]==board[2][1] && board[2][0]==0)
    {
        AbstractRobot::dropBall(2,0);
        return true;
    }
    else if(board[2][0]==board[2][1] && board[2][2]==0)
    {
        AbstractRobot::dropBall(2,2);
        return true;
    }
    else if(board[0][0]==board[1][1] && board[2][2]==0)
    {
        AbstractRobot::dropBall(2,2);
        return true;
    }
    else if(board[2][2]==board[1][1] && board[0][0]==0)
    {
        AbstractRobot::dropBall(0,0);
        return true;
    }
    else if(board[0][2]==board[1][1] && board[2][0]==0)
    {
        AbstractRobot::dropBall(2,0);
        return true;
    }
    else if(board[2][0]==board[1][1] && board[0][2]==0)
    {
        AbstractRobot::dropBall(0,2);
        return true;
    }
    else if(board[2][2]==board[2][0] && board[2][1]==0)
    {
        AbstractRobot::dropBall(2,1);
        return true;
    }
    else if(board[0][2]==board[2][2] && board[1][2]==0)
    {
        AbstractRobot::dropBall(1,2);
        return true;
    }
    else if(board[0][0]==board[0][2] && board[0][1]==0)
    {
        AbstractRobot::dropBall(0,1);
        return true;
    }
    else if(board[0][0]==board[2][0] && board[1][0]==0)
    {
        AbstractRobot::dropBall(1,0);
        return true;
    }
    else
    {
        return false;
    }

}

void TTT::bestDrop()
{
    for(int i=0; i<3; i++)
    {
        for(j=0; j<3; j++)
        {
            if(board[i][j]==2)
            {
                int bestX, bestY, oppX, oppY;

                if(i==0 && j==0) { oppX = 2; oppY = 2; }
                if(i==1 && j==0) { oppX = 1; oppY = 2; }
                if(i==2 && j==0) { oppX = 0; oppY = 2; }
                if(i==0 && j==1) { oppX = 2; oppY = 1; }
                if(i==2 && j==1) { oppX = 0; oppY = 1; }
                if(i==0 && j==2) { oppX = 2; oppY = 0; }
                if(i==1 && j==2) { oppX = 1; oppY = 0; }
                if(i==2 && j==2) { oppX = 0; oppY = 0; }

                do
                {
                    bestX = rand() % 3;
                    bestY = rand() % 3;
                }
                while( ( bestX==1 && bestY==1) || (bestX==i && bestY==j) || (bestX==oppX && bestY==oppY) );

                AbstractRobot::dropBall(bestX, bestY);
            }
        }
    }
}
