/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef SCREEN1VIEWBASE_HPP
#define SCREEN1VIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/containers/CacheableContainer.hpp>
#include <touchgfx/widgets/BoxWithBorder.hpp>
#include <touchgfx/containers/Container.hpp>
#include <touchgfx/mixins/Draggable.hpp>
#include <touchgfx/mixins/ClickListener.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>

class Screen1ViewBase : public touchgfx::View<Screen1Presenter>
{
public:
    Screen1ViewBase();
    virtual ~Screen1ViewBase();
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::Image bg;
    touchgfx::CacheableContainer drawingContainer;
    touchgfx::BoxWithBorder boxWithBorder1;
    touchgfx::Draggable< touchgfx::ClickListener< touchgfx::Container > > brushCanvas;
    touchgfx::Box box1;
    touchgfx::Container BlockButtons;
    touchgfx::Box background_box;
    touchgfx::ClickListener< touchgfx::Container > selectButton_powder;
    touchgfx::Box selection_tick_powder;
    touchgfx::Box red_box_powder;
    touchgfx::TextArea blockText_1_2;
    touchgfx::TextArea blockText_1;
    touchgfx::ClickListener< touchgfx::Container > selectButton_seed;
    touchgfx::Box selection_tick_seed;
    touchgfx::Box red_box_seed;
    touchgfx::TextArea blockText_1_1_2;
    touchgfx::TextArea blockText_1_1;
    touchgfx::ClickListener< touchgfx::Container > selectButton_water;
    touchgfx::Box selection_tick_water;
    touchgfx::Box red_box_water;
    touchgfx::TextArea blockText_1_1_1_2;
    touchgfx::TextArea blockText_1_1_1;
    touchgfx::ClickListener< touchgfx::Container > selectButton_erase;
    touchgfx::Box selection_tick_erase;
    touchgfx::Box red_box_erase;
    touchgfx::TextArea blockText_1_1_1_1_1_1_1;
    touchgfx::TextArea blockText_1_1_1_1_2_1;
    touchgfx::ClickListener< touchgfx::Container > selectButton_acid;
    touchgfx::Box selection_tick_acid;
    touchgfx::Box red_box_acid;
    touchgfx::TextArea blockText_1_1_1_1_1_1;
    touchgfx::TextArea blockText_1_1_1_1_2;
    touchgfx::ClickListener< touchgfx::Container > selectButton_block;
    touchgfx::Box selection_tick_block;
    touchgfx::Box red_box_block;
    touchgfx::TextArea blockText_1_1_1_1_1;
    touchgfx::TextArea blockText_1_1_1_1;
    touchgfx::ClickListener< touchgfx::Container > settingsButton_pup;
    touchgfx::Box red_box_pup;
    touchgfx::TextArea text_startStop_dropshadow;
    touchgfx::TextArea text_startStop;
    touchgfx::TextArea text_start;
    touchgfx::TextArea text_stop;
    touchgfx::ClickListener< touchgfx::Container > settingsButton_penS;
    touchgfx::Box red_box_pens;
    touchgfx::TextArea text_startStop_dropshadow_1;
    touchgfx::TextArea text_penS;
    touchgfx::TextAreaWithOneWildcard brushSize_dropshadow;
    touchgfx::TextAreaWithOneWildcard brushSize_text;

    /*
     * Wildcard Buffers
     */
    static const uint16_t BRUSHSIZE_DROPSHADOW_SIZE = 3;
    touchgfx::Unicode::UnicodeChar brushSize_dropshadowBuffer[BRUSHSIZE_DROPSHADOW_SIZE];
    static const uint16_t BRUSHSIZE_TEXT_SIZE = 3;
    touchgfx::Unicode::UnicodeChar brushSize_textBuffer[BRUSHSIZE_TEXT_SIZE];

private:

};

#endif // SCREEN1VIEWBASE_HPP
