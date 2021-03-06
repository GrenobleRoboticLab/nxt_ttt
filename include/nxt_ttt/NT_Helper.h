#ifndef NT_HELPER_H
#define NT_HELPER_H

#include <iostream>

void LOG(const std::string & sLog);
void STARTLOG(const char * pFilename);
void STOPLOG();

namespace nxt_ttt {

class Color {
public:
    Color(float fRed = 0.0f, float fGreen = 0.0f, float fBlue = 0.0f);
    Color(const Color & color) { initFrom(color); }
    ~Color();

    bool    operator==(const Color & color) { return compare(color); }
    bool    operator!=(const Color & color) { return !compare(color); }
    const Color& operator=(const Color & color) { initFrom(color); return *this;}

    bool    compare(const Color & color);
    void    initFrom(const Color & color);

    float   getRed()                { return m_fRed;        }
    float   getGreen()              { return m_fGreen;      }
    float   getBlue()               { return m_fBlue;       }

    void    setRed(float fValue)    { m_fRed = fValue;      }
    void    setGreen(float fValue)  { m_fGreen = fValue;    }
    void    setBlue(float fValue)   { m_fBlue = fValue;     }

    bool    isColor();
    bool    isNotColor();

private:
    float   m_fRed;
    float   m_fGreen;
    float   m_fBlue;

};

const int           TRANSLATE_X     = 1;
const int           TRANSLATE_Y     = 1;

const Color         BOTCOLOR(1.0f, 1.0f, 0.0f);
const Color         PLAYERCOLOR(0.0f, 1.0f, 0.0f);
const Color         NOCOLOR(0.0f, 0.0f, 0.0f);
const std::string   PLATNAME("plat_motor");
const std::string   SLIDENAME("slide_motor");
const std::string   DROPNAME("drop_motor");
const std::string   ULTRANAME("ultra_sensor");
const std::string   COLORNAME("color_sensor");
const std::string   CONTACTNAME("contact_sensor");

}

#endif // NT_HELPER_H
