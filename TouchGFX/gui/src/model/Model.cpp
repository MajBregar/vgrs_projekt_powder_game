#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <cmsis_os.h>
extern "C"{
	extern osMessageQueueId_t pixelInputQueueHandle;
	extern osMessageQueueId_t frameUpdateQueueHandle;
	extern osMessageQueueId_t blockSelectQueueHandle;
	extern osMessageQueueId_t gameSettingsQueueHandle;
}


Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	uint8_t updateframe = 0;
	osStatus_t queueStatus = osMessageQueueGet(frameUpdateQueueHandle, &updateframe, 0U, 0U);
	if (queueStatus == osOK && updateframe == 1){
		modelListener->updateFrame();
		if (GS == GAME_BUILDING){
			GS = GAME_RUNNING;
			updateGameSettings();
		}

	}

}

void Model::screenDotInput(uint16_t x, uint16_t y){
	uint32_t pixels = 0 | x;
	pixels = (pixels << 16) | y;
	osMessageQueuePut(pixelInputQueueHandle, &pixels, 0U, 0U);
}

void Model::changeBlockType(block_TypeDef block){
	osMessageQueuePut(blockSelectQueueHandle, &block, 0U, 0U);
}

void Model::updateGameSettings(){
	uint16_t settings = 0;
	settings = settings | GS;
	uint16_t bs = (0 | brushSize) << 2;
	settings = settings | bs;
	osMessageQueuePut(gameSettingsQueueHandle, &settings, 0U, 0U);
}


void Model::pressedPauseUnpause(){
	if (GS == GAME_PAUSED){
		GS = GAME_RUNNING;
		updateGameSettings();
		modelListener->updatePausedUnpaused(GAME_RUNNING);
	} else if (GS == GAME_RUNNING){
		GS = GAME_PAUSED;
		updateGameSettings();
		modelListener->updatePausedUnpaused(GAME_PAUSED);
	}
}

void Model::changedBrushSize(){
	brushSize *= 2;
	if (brushSize > 8){
		brushSize = 1;
	}
	updateGameSettings();
	modelListener->updateBrushSize('0'+brushSize);
}

void Model::changeGameState(gamestate gs){
	GS = gs;
	updateGameSettings();
}


