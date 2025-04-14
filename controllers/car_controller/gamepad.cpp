#define SDL_MAIN_USE_CALLBACKS 0  /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <iostream>

#include "gamepad.h"

/* We will use this renderer to draw into this window every frame. */
static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Joystick *joystick = NULL;
SDL_Gamepad *gamepad = NULL;     
static SDL_Color colors[64];
using namespace std;
int SDL_init() 
{
    int i = 0;
    char* error_code = NULL;
    SDL_SetAppMetadata("Test gamepad", "1.0", "jjy");

    if (!SDL_Init(SDL_INIT_GAMEPAD))
    {
        error_code = (char *)SDL_GetError();
        SDL_Log("Could not initialize SDL: %s", error_code);
        std::cout << "Could not initialize SDL: " << error_code << std::endl;
        return SDL_APP_FAILURE;
    }
    cout << "Initialize SDL successfully\n";
    cout << "SDL continue\n";
    return SDL_APP_CONTINUE;  /* carry on with the program! */
}


bool my_SDL_event(void *userdata, SDL_Event *event)
{
    if (event->type == SDL_EVENT_QUIT)
    {
        return SDL_APP_SUCCESS;
    }
    else if (event->type == SDL_EVENT_GAMEPAD_ADDED)
    {
        if (gamepad == NULL)
        {
            gamepad = SDL_OpenGamepad(event->gdevice.which);
            if (!gamepad)
            {
                SDL_Log("Failed to open joystick ID %u: %s", (unsigned int) event->gdevice.which, SDL_GetError());
                cout << "Failed to open joystick ID " << (unsigned int) event->gdevice.which << " : " << SDL_GetError() << endl;
            }
            else
            {
                cout << "Successful to open gamepad ID" << (unsigned int) event->gdevice.which << endl;
                cout << "Gamepad path: " << SDL_GetGamepadPath(gamepad) << endl;
                cout << "Gamepad name: " << SDL_GetGamepadName(gamepad) << endl;
                cout << "Gamepad type: " << SDL_GetGamepadType(gamepad) << endl;
                cout << "Gamepad mapping: " <<  SDL_GetGamepadMapping(gamepad) << endl;
                cout << "Gamepad product: " << SDL_GetGamepadProduct(gamepad) << endl;
            }
        }
    }
    else if (event->type == SDL_EVENT_GAMEPAD_REMOVED)
    {
        if (gamepad && (SDL_GetGamepadID(gamepad) == event->gdevice.which))
        {
            SDL_CloseGamepad(gamepad);
            gamepad = NULL;
            cout << "Gamepad is removed and close gamepad." << endl;
        }
    }
    return SDL_APP_CONTINUE;  /* carry on with the program! */
}

int SDL_iterate()
{

    return SDL_APP_CONTINUE;  /* carry on with the program! */
}

/* This function runs once at shutdown. */
void SDL_quit()
{
    if (gamepad)
    {
        SDL_CloseGamepad(gamepad);
    }
    /* SDL will clean up the window/renderer for us. */
}

static bool enable_axis = false;
void handle_SDL_event(SDL_Event *event)
{
    if (event->type == SDL_EVENT_QUIT)
    {
        // return SDL_APP_SUCCESS;
        cout << "quit" << endl;
    }
    else if (event->type == SDL_EVENT_GAMEPAD_ADDED)
    {
        if (gamepad == NULL)
        {
            gamepad = SDL_OpenGamepad(event->gdevice.which);
            if (!gamepad)
            {
                SDL_Log("Failed to open gamepad ID %u: %s", (unsigned int) event->gdevice.which, SDL_GetError());
                cout << "Failed to open gamepad--ID: " << (unsigned int) event->gdevice.which << " : " << SDL_GetError() << endl;
            }
            else
            {
                cout << "Successful to open gamepad ID" << (unsigned int) event->gdevice.which << endl;
                cout << "Gamepad path: " << SDL_GetGamepadPath(gamepad) << endl;
                cout << "Gamepad name: " << SDL_GetGamepadName(gamepad) << endl;
                cout << "Gamepad type: " << SDL_GetGamepadType(gamepad) << endl;
                cout << "Gamepad mapping: " <<  SDL_GetGamepadMapping(gamepad) << endl;
                cout << "Gamepad product: " << SDL_GetGamepadProduct(gamepad) << endl;
            }
        }
    }
    else if (event->type == SDL_EVENT_GAMEPAD_BUTTON_DOWN)
    {
        bool button = SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER);
        if (button)
        {
            button == false;
            cout << "Left shoulder button is pressed" << endl;
            // enable_axis = not enable_axis;
            enable_axis = !enable_axis;
            if (enable_axis == true)
            {
                cout << "Enable axis" << endl;
            }
            else
            {
                cout << "Disable axis" << endl;
            }
        }
    }
    else if (enable_axis && (event->type == SDL_EVENT_GAMEPAD_AXIS_MOTION))
    // else if (false)
    {
        extern double speed;
        extern double steering_angle;
        extern double throttle;
        int left_x = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX);
        int left_y = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTY);
        int right_x = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTX);
        int right_y = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTY);
        // cout << "left-x: " << left_x << endl;
        // cout << "left-y: " << left_y << endl;
        // cout << "right-x: " << right_x << endl;
        // cout << "right-y: " << right_y << endl;
        double temp = - right_y / 32767.0;
        speed = temp * 0.2;
        // throttle = - right_y / 32767.0 * 0.7;
        steering_angle = left_x / 32767.0 * 0.7;
    }
    else if (event->type == SDL_EVENT_GAMEPAD_REMOVED)
    {
        if (gamepad && (SDL_GetGamepadID(gamepad) == event->gdevice.which))
        {
            SDL_CloseGamepad(gamepad);
            gamepad = NULL;
            cout << "Gamepad is removed and close gamepad." << endl;
        }
    }
}