#include <stdio.h>
#include "nxt_ttt/NT_Helper.h"

using namespace nxt_ttt;

FILE* pFile;

void LOG(const std::string & sLog)
{
    if (pFile)
        fputs(sLog.c_str(), pFile);
}

void STARTLOG(const char * pFilename)
{
    pFile = fopen(pFilename, "w");
    LOG("Start logging\n");
}

void STOPLOG()
{
    LOG("Stop logging\n");
    if (pFile)
        fclose(pFile);
}

Color::Color(float fRed, float fGreen, float fBlue)
{
    m_fRed      = fRed;
    m_fGreen    = fGreen;
    m_fBlue     = fBlue;
}

Color::~Color() {}

bool Color::compare(const Color &color)
{
    bool bRet = false;

    if ((m_fRed     == color.m_fRed)
      &&(m_fGreen   == color.m_fGreen)
      &&(m_fBlue    == color.m_fBlue))
           bRet = true;

    return bRet;
}

void Color::initFrom(const Color &color)
{
    m_fRed      = color.m_fRed;
    m_fGreen    = color.m_fGreen;
    m_fBlue     = color.m_fBlue;
}

bool Color::isColor()       { return !isNotColor();     }
bool Color::isNotColor()    { return compare(NOCOLOR);  }
