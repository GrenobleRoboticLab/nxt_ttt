#ifndef NT_HELPER_H
#define NT_HELPER_H

namespace nxt_ttt {

class Color {

public:
    Color();
    ~Color();

    float   getRed() { return m_fRed; }
    float   getGreen() { return m_fGreen; }
    float   getBlue() { return m_fBlue; }

    void    setRed(float fValue) { m_fRed = fValue; }
    void    setGreen(float fValue) { m_fGreen = fValue; }
    void    setBlue(float fValue) { m_fBlue = fValue; }

private:
    float   m_fRed;
    float   m_fGreen;
    float   m_fBlue;

};

}

#endif // NT_HELPER_H
