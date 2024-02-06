#include <gui/screen1_screen/Screen1View.hpp>
#include <touchgfx/Color.hpp>
#include <touchgfx/widgets/PixelDataWidget.hpp>
#include <touchgfx/Bitmap.hpp>
#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/hal/HAL.hpp>

using namespace touchgfx;

extern "C"{
	extern canvas_TypeDef canvas;
}

void Screen1View::updateFrame(){
	drawingContainer.invalidateContent();
}

void Screen1View::updateBrushSize(char bs){
	brushSize = bs - '0';
	Unicode::snprintf(brushSize_textBuffer, 3, "%c", bs);
	Unicode::snprintf(brushSize_dropshadowBuffer, 3, "%c", bs);
}

void Screen1View::updatePausedUnpaused(gamestate gs){
	if (gs == GAME_PAUSED){
		text_start.setVisible(false);
		text_start.invalidate();
		text_stop.setVisible(true);
		text_stop.invalidate();
	} else if (gs == GAME_RUNNING){
		text_start.setVisible(true);
		text_start.invalidate();
		text_stop.setVisible(false);
		text_stop.invalidate();
	}
}

void Screen1View::sendInput(uint16_t x, uint16_t y, uint8_t size, uint16_t w, uint16_t h){
	x = x - size/2;
	y = y - size/2;
	if (x < 0 || x >= w || y < 0 || y >= h) return;
	presenter->screenDotInput(x, y);
}

uint16_t interpFunc(uint16_t x, uint16_t sy, uint16_t sx, uint16_t k){
	return sy + (x - sx) * k;
}

void Screen1View::interpPaint(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint8_t size, uint16_t w, uint16_t h){
	if (ex == sx){
		char dir = sy < ey ? 1 : -1;
		uint16_t cur_y = sy;
		while (cur_y != ey){
			sendInput(sx, cur_y, size, w, h);
			cur_y += dir;
		}
		return;
	}

	if (sx > ex){
		uint16_t tmpx = ex;
		uint16_t tmpy = ey;
		ex = sx;
		ey = sy;
		sx = tmpx;
		sy = tmpy;
	}
	uint16_t k = (ey - sy) / (ex - sx);

	for (int x = sx; x != ex; x += 1){
		sendInput(x, interpFunc(x, sy, sx, k), size, w, h);
	}

}


void Screen1View::handleDragEvent(const touchgfx::DragEvent& event)
{
    View::handleDragEvent(event);
    if (event.getDeltaX() > 20 || event.getDeltaY() > 20) return;
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    uint16_t ox = event.getOldX() - drawingContainer.getX();
    uint16_t oy = event.getOldY() - drawingContainer.getY();
    uint16_t nx = event.getNewX() - drawingContainer.getX();
    uint16_t ny = event.getNewY() - drawingContainer.getY();
	//paintDot(x, y, 6, (uint16_t*) arr);
    interpPaint(ox, oy, nx, ny, brushSize, canvas.width, canvas.height);
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

}
void Screen1View::showSelection(touchgfx::Box* b){
	selection_tick_powder.setVisible(false);
	selection_tick_powder.invalidate();
	selection_tick_water.setVisible(false);
	selection_tick_water.invalidate();
	selection_tick_seed.setVisible(false);
	selection_tick_seed.invalidate();
	selection_tick_block.setVisible(false);
	selection_tick_block.invalidate();
	selection_tick_acid.setVisible(false);
	selection_tick_acid.invalidate();
	selection_tick_erase.setVisible(false);
	selection_tick_erase.invalidate();
	b->setVisible(true);
	b->invalidate();
}


void Screen1View::ButtonClickHandler(const touchgfx::Container &cont, const touchgfx::ClickEvent &event){
    if (&cont == &this->selectButton_powder){
    	if (event.getType() == ClickEvent::PRESSED){
    		red_box_powder.setVisible(true);
    		presenter->changeBlockType(BT_POWDER);
    		showSelection(&selection_tick_powder);
    	} else {
    		red_box_powder.setVisible(false);
    	}
    	red_box_powder.invalidate();
    } else if (&cont == &this->selectButton_block){
    	if (event.getType() == ClickEvent::PRESSED){
    		red_box_block.setVisible(true);
    		presenter->changeBlockType(BT_WALL);
    		showSelection(&selection_tick_block);
		} else {
			red_box_block.setVisible(false);
		}
    	red_box_block.invalidate();
    } else if (&cont == &this->selectButton_water){
    	if (event.getType() == ClickEvent::PRESSED){
			red_box_water.setVisible(true);
			presenter->changeBlockType(BT_WATER);
			showSelection(&selection_tick_water);
		} else {
			red_box_water.setVisible(false);
		}
    	red_box_water.invalidate();
    } else if (&cont == &this->selectButton_seed){
    	if (event.getType() == ClickEvent::PRESSED){
			red_box_seed.setVisible(true);
			presenter->changeBlockType(BT_SEED);
			showSelection(&selection_tick_seed);
		} else {
			red_box_seed.setVisible(false);
		}
    	red_box_seed.invalidate();
    } else if (&cont == &this->selectButton_acid){
    	if (event.getType() == ClickEvent::PRESSED){
			red_box_acid.setVisible(true);
			presenter->changeBlockType(BT_ACID);
			showSelection(&selection_tick_acid);
		} else {
			red_box_acid.setVisible(false);
		}
    	red_box_acid.invalidate();
    } else if (&cont == &this->selectButton_erase){
    	if (event.getType() == ClickEvent::PRESSED){
			red_box_erase.setVisible(true);
			presenter->changeBlockType(BT_VACCUM);
			showSelection(&selection_tick_erase);
		} else {
			red_box_erase.setVisible(false);
		}
    	red_box_erase.invalidate();
    } else if (&cont == &this->settingsButton_pup){
    	if (event.getType() == ClickEvent::PRESSED){
    		red_box_pup.setVisible(true);
    		presenter->pressedPauseUnpause();
		} else {
			red_box_pup.setVisible(false);
		}
    	red_box_pup.invalidate();
    } else if (&cont == &this->settingsButton_penS){
    	if (event.getType() == ClickEvent::PRESSED){
    		red_box_pens.setVisible(true);
    		presenter->changedBrushSize();
		} else {
			red_box_pens.setVisible(false);
		}
    	red_box_pens.invalidate();
    }
}

void Screen1View::DrawBoxClickHandler(const touchgfx::Container &cont, const touchgfx::ClickEvent &event){
    if (&cont == &this->brushCanvas){
        if (event.getType() == ClickEvent::PRESSED){
            //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
            uint16_t screenx = brushCanvas.getX() + event.getX();
            uint16_t screeny = brushCanvas.getY() + event.getY();
            brushCanvas.moveTo(screenx - brushCanvas.getWidth()/2, screeny - brushCanvas.getHeight()/2);


            uint16_t pixelx = screenx - drawingContainer.getX();
            uint16_t pixely = screeny - drawingContainer.getY();

            sendInput(pixelx, pixely, brushSize, canvas.width, canvas.height);
        } else if (event.getType() == ClickEvent::RELEASED){
            //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
        }
    }
}

Screen1View::Screen1View():
DrawBoxCallback(this, &Screen1View::DrawBoxClickHandler),
ButtonsCallback(this, &Screen1View::ButtonClickHandler)
{
}



void Screen1View::setupScreen()
{
    drawingContainer.enableCachedMode(true);
    canvas.width = drawingContainer.getWidth();
    canvas.height = drawingContainer.getHeight();

    BitmapId bmpId = Bitmap::dynamicBitmapCreate(canvas.width, canvas.height, Bitmap::RGB565);
    if (bmpId != BITMAP_INVALID){
    	drawingContainer.setCacheBitmap(bmpId);
    	canvas.canvasBitmap = (uint16_t*) Bitmap::dynamicBitmapGetAddress(bmpId);
        for (int i = 0; i < canvas.width * canvas.height; i++) canvas.canvasBitmap[i] = BT_VACCUM;
        drawingContainer.invalidateContent();
    }

    brushCanvas.setClickAction(DrawBoxCallback);
    selectButton_powder.setClickAction(ButtonsCallback);
    selectButton_block.setClickAction(ButtonsCallback);
    selectButton_water.setClickAction(ButtonsCallback);
    selectButton_seed.setClickAction(ButtonsCallback);
    selectButton_acid.setClickAction(ButtonsCallback);
    selectButton_erase.setClickAction(ButtonsCallback);
    settingsButton_pup.setClickAction(ButtonsCallback);
    settingsButton_penS.setClickAction(ButtonsCallback);

    presenter->changeGameState(GAME_BUILDING);
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}
