#include "nxt_ttt/NT_TTT.h"
#include <map>

using namespace nxt_ttt;

int exploredLines[8];
std::vector<int> ambiguousLinesId;

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
}

void TTT::cbColor(int x, int y, Color color)
{
    LOG("INFO : TTT : Receive Color.\n");
    std::cout << "\t\t\t\t\t : " << color.getRed() << " : " << color.getGreen() << " : " << color.getBlue() << std::endl;
    if (m_wCurrentState == TS_SCAN)
    {
        if (color.isColor())
            m_bBoardEmpty = false;

        m_Board[x][y] = color;
        continueScan();
    }
}

void TTT::exploreLines(){
    int weight;
    for(int i = 0; i < 8; i++){
        std::cout << "line :" << i << std::endl;
        weight = 0;
        for(int j = 0; j < 3; j++){
            if(m_Board[LINES[i][j].x][LINES[i][j].y] == PLAYERCOLOR)
                weight += 1;
            else if(m_Board[LINES[i][j].x][LINES[i][j].y] == BOTCOLOR)
                weight += 4;
            else
                weight += 2;
        }
       //problème avec la ligne BOT, PLAYER, PLAYER = NONE, NONE, NONE
        if(weight == 6 && (m_Board[LINES[i][0].x][LINES[i][0].y] == PLAYERCOLOR || m_Board[LINES[i][0].x][LINES[i][0].y] == BOTCOLOR))
            weight = 0;
        exploredLines[i] = weight;
    }
}

//we calculate weight from each line configuration the case is in
int TTT::weightCase(Point point){
    double weight = 0;
    for(int i = 0; i < 8; i++){
        for(int j = 0; j < 3; j++){
            if (LINES[i][j].x == point.x && LINES[i][j].y == point.y){
                if(exploredLines[i] == 7) weight += 1;
                else if(exploredLines[i] == 6) weight += 2;
                else if(exploredLines[i] == 5) weight += 3;
                else if(exploredLines[i] == 8) weight += 4;
            }
        }
    }
    return weight;
}

Point TTT::bestCase(){
    Point bestCase;
    int   bestScore = 0;
    int   tempScore = 0;
    int size = ambiguousLinesId.size();
    for(int i = 0; i < size; i++){
        for (int j = 0; j < 3; j++){
            if(m_Board[LINES[ambiguousLinesId[i]][j].x][LINES[ambiguousLinesId[i]][j].y] == NOCOLOR){
                tempScore = weightCase(LINES[ambiguousLinesId[i]][j]);
                if (tempScore > bestScore){
                    bestScore = tempScore;
                    bestCase = LINES[ambiguousLinesId[i]][j];
                }
            }
        }
    }
    return bestCase;
}

void TTT::treat()
{
    printBoard();
    exploreLines();
    for(int b = 0; b < 8; b++){
        std::cout << "Ligne " << b << ": " << exploredLines[b] << std::endl;
    }
    ambiguousLinesId.clear();

    StateFlag bestState = SF_NONE;

    for(int i = 0; i < 8; i++){
        switch (exploredLines[i]){

        // 3 bot
        case 12 :
        {
            std::cout << "3 bot" << std::endl;
            m_pApplication->cbEnd(NT_TTT);
            return;
        }
        // 3 player
        case 3 :
        {
            std::cout << "3 player" << std::endl;
            m_pApplication->cbEnd(NT_PLAYER);
            return;
        }
        // 2 bot 1 empty
        case 10 :
        {
            std::cout << "2 bot 1 empty" << std::endl;
            Point line[3];
            for(int k = 0; k < 2; k++){
                line[k] = LINES[i][k];
            }
            for(int j = 0; j < 2; j++){
                if (m_Board[line[j].x][line[j].y] == NOCOLOR){
                    std::cout << "bestCase :" << "(" << line[j].x << "," << line[j].y << ")" << std::endl;
                    m_pRobot->dropBall(line[j].x, line[j].y);
                    m_pApplication->cbEnd(NT_TTT);
                    return;
                }
            }
        }
        // 2 player 1 empty
        case 4 :
        {
            std::cout << "2 player 1 empty" << std::endl;
            Point line[3];
            for(int k = 0; k < 2; k++){
                line[k] = LINES[i][k];
            }
            for(int j = 0; j < 2; j++){
                if (m_Board[line[j].x][line[j].y] == NOCOLOR){
                    std::cout << "bestCase :" << "(" << line[j].x << "," << line[j].y << ")" << std::endl;
                    m_pRobot->dropBall(line[j].x, line[j].y);
                    return;
                }
            }
        }
        // 1 bot 2 empty
        case 8 :
        {
            std::cout << "1 bot 2 empty" << std::endl;
            if(bestState != SF_BOTONE){
                bestState = SF_BOTONE;
                ambiguousLinesId.clear();
            }
            ambiguousLinesId.push_back(i);
            std::cout << "ambiguousLine :" << i << "bestState : " << bestState << std::endl;
            break;
        }
        // 3 empty
        case 6 :
        {
            std::cout << "3 empty" << std::endl;
            if(bestState != SF_BOTONE){
                if(bestState != SF_EMPTY){
                    ambiguousLinesId.clear();
                    bestState = SF_EMPTY;
                }
                ambiguousLinesId.push_back(i);
            }
            std::cout << "ambiguousLine :" << i << "bestState : " << bestState << std::endl;
            break;
        }
        // 1 player 2 empty
        case 5 :
        {
            std::cout << "1 player 2 empty" << std::endl;
            if(bestState != SF_BOTONE && bestState != SF_EMPTY){
                if(bestState != SF_PLAYERONE){
                    bestState = SF_PLAYERONE;
                    ambiguousLinesId.clear();
                }
                ambiguousLinesId.push_back(i);
                std::cout << "ambiguousLine :" << i << "bestState : " << bestState << std::endl;
            }
            break;
        }
        // 1 each
        case 7 :
            std::cout << "1 each" << std::endl;
        {
            if( bestState == SF_NONE)
                bestState = SF_ONEEACH;
            if( bestState == SF_ONEEACH)
                ambiguousLinesId.push_back(i);
            std::cout << "ambiguousLine :" << i << "bestState : " << bestState << std::endl;
        }
        }
    }
    Point bestPoint = bestCase();
    std::cout << "bestCase :" << "(" << bestPoint.x << "," << bestPoint.y << ")" << std::endl;
    m_pRobot->dropBall(bestPoint.x, bestPoint.y);
    return;
}

void TTT::cbDropped(int x, int y)
{
    m_wCurrentState = TS_NONE;
    m_pApplication->cbPlayed(NT_TTT);
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

void TTT::printBoard()
{
    std::cout << std::endl << "BOARD SCANNED" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (m_Board[i][j] == PLAYERCOLOR)
                std::cout << "X";
            else if (m_Board[i][j] == BOTCOLOR)
                std::cout << "O";
            else
                std::cout << "?";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << std::endl  << std::endl;
}
