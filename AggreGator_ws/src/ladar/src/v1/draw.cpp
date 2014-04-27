#include <iostream>
#include <cstdlib>
#include <utility>
#include <vector>
#include <cmath>
#include <sstream>
#include "ladar/SDL/SDL.h"
#include "ladar/draw.h"
using namespace std;

void DrawSDL::putpixel(SDL_Surface *surface, int x, int y, Uint32 pixel)
{
  int bpp = surface->format->BytesPerPixel;
 // Here p is the address to the pixel we want to set
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

 switch(bpp)
 {
  case 1:
   *p = pixel;
   break;
  case 2:
   *(Uint16 *)p = pixel;
   break;
  case 3:
   if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
   {
    p[0] = (pixel >> 16) & 0xff;
    p[1] = (pixel >> 8) & 0xff;
    p[2] = pixel & 0xff;
   } else {
    p[0] = pixel & 0xff;
    p[1] = (pixel >> 8) & 0xff;
    p[2] = (pixel >> 16) & 0xff;
   }
   break;
  case 4:
   *(Uint32 *)p = pixel;
   break;
 }
}//end putpixel

int DrawSDL::init(){
  //initialize the SDL object
  if((SDL_Init(SDL_INIT_VIDEO|SDL_INIT_AUDIO)==-1))
  {
    printf("Could not initialize SDL: %s.\n", SDL_GetError());
    return -1;
  }

   screen = SDL_SetVideoMode(800, 450, 32, SDL_SWSURFACE | SDL_FULLSCREEN);
   
  if ( screen == NULL )
  {
    fprintf(stderr, "Couldn't set video mode: %s\n", SDL_GetError());
    return -2;
  }

  //set color values
  yellow = SDL_MapRGB(screen->format, 0xff, 0xff, 0x00);  
  white = SDL_MapRGB(screen->format, 0xff, 0xff, 0xff);

  return 1;
}

DrawSDL::DrawSDL(){
  init();
  quit= 0;
}

int DrawSDL::drawCore(vector<pair<int, int> > coordinates){
     // Make the dot at the center of the screen
     while( !quit ) {
        // Poll for events
          while( SDL_PollEvent( &event ) ){
            switch( event.type ){
                case SDL_KEYUP:
                    if(event.key.keysym.sym == SDLK_ESCAPE)
                    quit = 1;
                    break;
                    if(event.key.keysym.sym == SDLK_F1)
                    SDL_WM_ToggleFullScreen(screen); // Only on X11
                    break;
                case SDL_QUIT:
                    quit = 1;
                    break;
                default: break;
            }//end switch
        }//end while

        // Lock the screen for direct access to the pixels
        if ( SDL_MUSTLOCK(screen) ){
           if ( SDL_LockSurface(screen) < 0 ){
              fprintf(stderr, "Can't lock screen: %s\n", SDL_GetError());
              return -3;
           }//endif
        }//endif

        // Plot the argument
        for(int i = 0; i < coordinates.size(); i++){
            putpixel(screen, coordinates.at(i).first, coordinates.at(i).second, yellow);
            SDL_UpdateRect(screen, coordinates.at(i).first, coordinates.at(i).second, 1, 1);
        }
        // Unlock Surface if necessary
        if ( SDL_MUSTLOCK(screen) ) SDL_UnlockSurface(screen);
    }//end while

     //clean up
     SDL_Quit();
 
}//end draw(<int, int>)

void DrawSDL::draw(vector<pair<float, float> > coordinates){
    int height = screen->h/2;
    int width = screen->w/2;
    int x, y;
    vector<pair<int, int> > converted(coordinates.size());
    converted.clear();

    //translate all coordinates relative to middle of screen
    for(int i = 0; i < coordinates.size(); i++){
      x = width + coordinates.at(i).first * 100;
      y = height + coordinates.at(i).second * 100;
      pair<int, int> coord(x, y);
      converted.push_back(coord);
    }

    drawCore(converted);
}
