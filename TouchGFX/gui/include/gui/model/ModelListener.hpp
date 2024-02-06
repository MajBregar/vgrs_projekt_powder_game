#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>
#include "main.h"
class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }
    virtual void updateFrame();
    virtual void updateBrushSize(char bs);
    virtual void updatePausedUnpaused(gamestate gs);
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
