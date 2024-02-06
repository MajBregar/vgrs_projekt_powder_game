#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>
#include <touchgfx/Callback.hpp>
#include <touchgfx/mixins/Draggable.hpp>

#include "main.h"


class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    void DrawBoxClickHandler(const touchgfx::Container&, const touchgfx::ClickEvent &);
    void ButtonClickHandler(const touchgfx::Container&, const touchgfx::ClickEvent &);
    virtual void handleDragEvent(const DragEvent &event);

    virtual void sendInput(uint16_t x, uint16_t y, uint8_t size, uint16_t w, uint16_t h);
    virtual void interpPaint(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint8_t size, uint16_t w, uint16_t h);
    virtual void showSelection(touchgfx::Box* b);
    virtual void updateFrame();
    virtual void updateBrushSize(char bs);
    virtual void updatePausedUnpaused(gamestate gs);


protected:
    touchgfx::Callback<Screen1View, const touchgfx::Container&, const touchgfx::ClickEvent&> DrawBoxCallback;
    touchgfx::Callback<Screen1View, const touchgfx::Container&, const touchgfx::ClickEvent&> ButtonsCallback;
    uint8_t brushSize = 1;
};

#endif // SCREEN1VIEW_HPP
