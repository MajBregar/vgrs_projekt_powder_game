#ifndef MODEL_HPP
#define MODEL_HPP

#include <stdint.h>
#include "main.h"


class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
    virtual void screenDotInput(uint16_t x, uint16_t y);
    virtual void changeBlockType(block_TypeDef block);
    virtual void changedBrushSize();
    virtual void pressedPauseUnpause();
    virtual void updateGameSettings();
    virtual void changeGameState(gamestate gs);

protected:
    ModelListener* modelListener;
    gamestate GS = GAME_RESET;
    uint8_t brushSize = 1;

};

#endif // MODEL_HPP
