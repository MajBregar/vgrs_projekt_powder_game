#include <gui/common/FrontendApplication.hpp>
#include <touchgfx/Bitmap.hpp>

extern "C"{
	extern volatile uint8_t* sdramcachestart;
	extern uint32_t sdramcachesize;

}

//uint8_t cache[2 * 480 * 272];
FrontendApplication::FrontendApplication(Model& m, FrontendHeap& heap)
    : FrontendApplicationBase(m, heap)
{
    Bitmap::setCache((uint16_t*)sdramcachestart, sdramcachesize, 1);
}
