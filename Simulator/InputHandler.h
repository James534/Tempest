#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H
#include "irrlicht.h"

class InputHandler : public irr::IEventReceiver
{
public:
    //TODO, MAKE ALL THIS INTO THE .CPP FILE
    InputHandler(bool);

    virtual bool OnEvent(const irr::SEvent& event){
        if (event.EventType == irr::EET_KEY_INPUT_EVENT)
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
        return false;
    }
    virtual bool IsKeyDown(irr::EKEY_CODE keyCode) const
    {
        return KeyIsDown[keyCode];
    }

    void update(irr::f32);

    bool usingKeyboard();
    void setAcc(irr::core::vector3df);
    irr::core::vector3df getAcc();
    void setRot(irr::core::vector3df);
    irr::core::vector3df getRot();
private:
    bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];

    bool useKey;
    irr::core::vector3df acc;
    irr::core::vector3df rot;
};

#endif // INPUTHANDLER_H
