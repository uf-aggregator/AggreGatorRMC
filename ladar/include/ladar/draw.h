#ifndef _SDL_DRAW_
#define _SDL_DRAW_
#include "ladar/SDL/SDL.h" 

class DrawSDL {
	protected:
		int quit;
		SDL_Surface *screen;
		SDL_Event event;
		Uint32 yellow, white;
	public:
		DrawSDL();
		~DrawSDL(){
			delete [] screen;
		}
		int init();
		int drawCore(std::vector<std::pair<int, int> > coordinates);
		void putpixel(SDL_Surface *surface, int x, int y, Uint32 pixel);
		void draw(std::vector<std::pair<float, float> > coordinates);
};

#endif
