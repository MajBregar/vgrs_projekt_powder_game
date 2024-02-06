#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>


Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}

void Screen1Presenter::screenDotInput(uint16_t x, uint16_t y){
	model->screenDotInput(x, y);
}


void Screen1Presenter::updateFrame(){
	view.updateFrame();
}

void Screen1Presenter::changeBlockType(block_TypeDef block){
	model->changeBlockType(block);
}


void Screen1Presenter::pressedPauseUnpause(){
	model->pressedPauseUnpause();
}

void Screen1Presenter::changedBrushSize(){
	model->changedBrushSize();
}

void Screen1Presenter::changeGameState(gamestate gs){
	model->changeGameState(gs);
}

void Screen1Presenter::updateBrushSize(char bs){
	view.updateBrushSize(bs);
}

void Screen1Presenter::updatePausedUnpaused(gamestate gs){
	view.updatePausedUnpaused(gs);
}
