#ifndef GAMEPAD_H
#define GAMEPAD_h
#include <SDL3/SDL.h>

int SDL_init();
bool my_SDL_event(void *userdata, SDL_Event *event);
int SDL_iterate();
void SDL_quit();
void handle_SDL_event(SDL_Event *event);
#endif