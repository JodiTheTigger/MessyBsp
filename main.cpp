/*
    MessyBsp. BSP collision and loading example code.
    Copyright (C) 2014 Richard Maxwell <jodi.the.tigger@gmail.com>
    This file is part of MessyBsp
    MessyBsp is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU Affero General Public License for more details.
    You should have received a copy of the GNU Affero General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include "Bsp.hpp"
#include "BspBrushToMesh.hpp"
#include "TraceTest.hpp"
#include "VectorMaths3.hpp"
#include "Matrix4x4Maths.hpp"
#include "GLDebug.hpp"
#include "third-party/getopt/getopt.h"

// SDL + OpenGL
#include <SDL.h>

// Need ifdef for different platforms.
#include <GL/glew.h>

#include <vector>
#include <memory>

#include <cstdlib>
#include <cstdio>
#include <cmath>

static const float Pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679f;
static const float DegToRad = (2.0f * Pi / 360.0f);

struct Globals
{
    // 50Hz for now.
    unsigned tickClientPeriodInMicroseconds = 20 * 1000;
    unsigned ticksPerSecond = 1000000 / tickClientPeriodInMicroseconds;

    // Need to convert all this stuff to SI units.
    // For now, lets assume 1 game unit = 1m
    float viewAngleSpeedPerTick = 2 * Pi / ticksPerSecond * 2.0f;
    float moveDeltaPerTick = 200.0f / ticksPerSecond;
    float viewAnglePerMouseMoveUnit = 0.01f;

    // For testing lighting, rotates once every 5 seconds
    float modelRotationsPerTick = ((2 * Pi) / 5.0f) / ticksPerSecond;

    // got the deadzone value from SDL somewhere.
    int joystickDeadZone = 6000;

    // Deduced this one emperically.
    float joystickToMouseMultiplier = 0.001f;
};

const Globals globals = {};

enum ActionMap
{
    Forward = 0,
    Backward,
    StrafeLeft,
    StrafeRight,
    Up,
    Down,
    Quit,

    Count
};

struct PlayerActions
{
    bool actions[ActionMap::Count];

    int mouseX;
    int mouseY;
};

struct ControllerIdToPointer
{
    int id;
    SDL_GameController* pad;
    SDL_Joystick* padAsJoystick;
    int instance;
};

static const uint8_t keymap[ActionMap::Count] =
{
    SDL_SCANCODE_W,
    SDL_SCANCODE_S,
    SDL_SCANCODE_A,
    SDL_SCANCODE_D,
    SDL_SCANCODE_SPACE,
    SDL_SCANCODE_Q,
    SDL_SCANCODE_ESCAPE,
};

static const SDL_GameControllerButton padmap[ActionMap::Count] =
{
    SDL_CONTROLLER_BUTTON_DPAD_UP,
    SDL_CONTROLLER_BUTTON_DPAD_DOWN,
    SDL_CONTROLLER_BUTTON_DPAD_LEFT,
    SDL_CONTROLLER_BUTTON_DPAD_RIGHT,
    SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
    SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
    SDL_CONTROLLER_BUTTON_START,
};

// Globals
Matrix4x4 g_projection;

void DoGraphics(const Bsp::CollisionBsp& bsp);


void PrintHelp()
{
    printf("\n");
    printf("MessyBsp - By Richard Maxwell\n\n");

    printf("  Renders or does a collsion detection benchmark on a quake3 bsp.\n\n");

    printf("  MessyBsp [-b] [-h] [-f <path to quake3 bsp>]\n\n");

    printf("  -b:  Benchmark 100,000 random collision tests\n");
    printf("       Prints the cost in Microseconds. Otherwise\n");
    printf("       Renders all the solid brushes using opengl.\n\n");

    printf("  -f:  Quake3 bsp file to use. Defaults to 'final.bsp'.\n\n");

    printf("  -h:  This help text.\n");
    printf("\n");
}

int main(int argc, char *argv[])
{
    bool benchmark = false;
    char fileName[1024];

    strcpy(fileName, "final.bsp");

    // Parse options
    while (auto ch = getopt(argc, argv, "hbf:"))
    {
        if (ch < 0)
        {
            break;
        }

        if (ch == 'h')
        {
            PrintHelp();

            return 0;
        }

        if (ch == 'f')
        {
            if (optarg)
            {
                strncpy(fileName, optarg, sizeof(fileName) - 1);
            }
        }

        if (ch == 'b')
        {
            benchmark = true;
        }
    }

    {
        auto fileHandleExists = fopen(fileName, "r");
        if (!fileHandleExists)
        {
            PrintHelp();
            printf("---------------------------\n");
            printf("File '%s' cannot be opened.\n", fileName);
            printf("---------------------------\n");
            return -1;
        }

        fclose(fileHandleExists);
    }

    Bsp::CollisionBsp bsp;

    Bsp::GetCollisionBsp(fileName, bsp);

    if (benchmark)
    {
        auto result = TimeBspCollision(bsp, 100000);

        printf("Trace Took %ld microseconds\n", result.count());

        return 0;
    }

    DoGraphics(bsp);

    return 0;
}

void AddController(
        int id,
        std::vector<ControllerIdToPointer>& controllers)
{
    if (!SDL_IsGameController(id))
    {
        return;
    }

    for (const auto& pad : controllers)
    {
        if (pad.id == id)
        {
            return;
        }
    }

    ControllerIdToPointer result;

    result.id  = id;
    result.pad = SDL_GameControllerOpen(id);

    if (result.pad)
    {
        result.padAsJoystick = SDL_GameControllerGetJoystick(result.pad);

        if (result.padAsJoystick)
        {
            result.instance = SDL_JoystickInstanceID(result.padAsJoystick);
            controllers.push_back(result);
            return;
        }
    }

    if (result.pad != nullptr)
    {
        SDL_GameControllerClose(result.pad);
    }
}

void RemoveController(
        int instance,
        std::vector<ControllerIdToPointer>& controllers)
{
    for (const auto& pad : controllers)
    {
        if (pad.instance == instance)
        {
            SDL_GameControllerClose(pad.pad);
            return;
        }
    }
}

void OpenAllControllers(std::vector<ControllerIdToPointer>& controllers)
{
    auto count = SDL_NumJoysticks();

    for (int i = 0; i < count; ++i)
    {
        AddController(i, controllers);
    }
}

PlayerActions GetActions(std::vector<ControllerIdToPointer>& controllers)
{
    PlayerActions result = {{},0,0};

    // Keyboard
    const auto* keys = SDL_GetKeyboardState(nullptr);

    for (unsigned i = 0; i < ActionMap::Count; ++i)
    {
        if (keys[keymap[i]])
        {
            result.actions[i] = true;
        }
    }

    // Mouse
    SDL_GetRelativeMouseState(&result.mouseX, &result.mouseY);

    // Gamepad
    for (const auto& controller : controllers)
    {
        // buttons
        for (unsigned i = 0; i < ActionMap::Count; ++i)
        {
            if (SDL_GameControllerGetButton(controller.pad, padmap[i]))
            {
                result.actions[i] = true;
            }
        }

        // Gamepad axis
        if (!result.mouseX && !result.mouseY)
        {
            auto x = SDL_GameControllerGetAxis(
                        controller.pad,
                        SDL_CONTROLLER_AXIS_RIGHTX);

            auto y = SDL_GameControllerGetAxis(
                        controller.pad,
                        SDL_CONTROLLER_AXIS_RIGHTY);

            if (x > globals.joystickDeadZone)
            {
                result.mouseX = x - globals.joystickDeadZone;
            }

            if (x < -globals.joystickDeadZone)
            {
                result.mouseX = x + globals.joystickDeadZone;
            }

            if (y > globals.joystickDeadZone)
            {
                result.mouseY = y - globals.joystickDeadZone;
            }

            if (y < -globals.joystickDeadZone)
            {
                result.mouseY = y + globals.joystickDeadZone;
            }

            result.mouseX = static_cast<int>(result.mouseX * globals.joystickToMouseMultiplier);
            result.mouseY = static_cast<int>(result.mouseY * globals.joystickToMouseMultiplier);
        }
    }

    return result;
}

// This class should be full of platform voodoo if done properly.
// good thing I'm not doing it properly.
uint64_t Microseconds()
{
    return static_cast<uint64_t>(SDL_GetTicks()) * 1000l;
}

std::vector<float> MakeTriangles(const Bsp::CollisionBsp& bsp)
{
    static bool useBsp = true;
    static unsigned maxCount = 10009;

    if (useBsp)
    {
        return Bsp::BrushMeshesAsTriangleListWithNormals(
            bsp,
            maxCount);
    }
    else
    {      
        return std::vector<float>
        {
            // front (x,y,z,nx,ny,nz)
            0.0f,   0.0f,   0.0f,        0.0f,   0.0f,   1.0f,
            10.0f,  0.0f,   0.0f,        0.0f,   0.0f,   1.0f,
            0.0f,   10.0f,  0.0f,        0.0f,   0.0f,   1.0f,

            10.0f,  0.0f,   0.0f,        0.0f,   0.0f,   1.0f,
            10.0f,  10.0f,  0.0f,        0.0f,   0.0f,   1.0f,
            0.0f,   10.0f,  0.0f,        0.0f,   0.0f,   1.0f,

            // left
            0.0f,   0.0f,   0.0f,       -1.0f,  0.0f,   0.0f,
            0.0f,   10.0f,  0.0f,       -1.0f,  0.0f,   0.0f,
            0.0f,   0.0f,  -10.0f,      -1.0f,  0.0f,   0.0f,

            0.0f,  0.0f,   -10.0f,      -1.0f,  0.0f,   0.0f,
            0.0f,  10.0f,  0.0f,        -1.0f,  0.0f,   0.0f,
            0.0f,  10.0f,  -10.0f,      -1.0f,  0.0f,   0.0f,

            // right
            10.0f,  0.0f,   0.0f,        1.0f,  0.0f,   0.0f,
            10.0f,  0.0f,   -10.0f,      1.0f,  0.0f,   0.0f,
            10.0f,  10.0f,  0.0f,        1.0f,  0.0f,   0.0f,

            10.0f,  0.0f,   -10.0f,      1.0f,  0.0f,   0.0f,
            10.0f,  10.0f,  -10.0f,      1.0f,  0.0f,   0.0f,
            10.0f,  10.0f,  0.0f,        1.0f,  0.0f,   0.0f,


            // back
            0.0f,   0.0f,   -10.0f,      0.0f,  0.0f,  -1.0f,
            0.0f,   10.0f,  -10.0f,      0.0f,  0.0f,  -1.0f,
            10.0f,  0.0f,   -10.0f,      0.0f,  0.0f,  -1.0f,

            10.0f,  0.0f,   -10.0f,      0.0f,  0.0f,  -1.0f,
            0.0f,   10.0f,  -10.0f,      0.0f,  0.0f,  -1.0f,
            10.0f,  10.0f,  -10.0f,      0.0f,  0.0f,  -1.0f,

            // top
            0.0f,   10.0f,   0.0f,       0.0f,  1.0f,   0.0f,
            10.0f,   10.0f,  -10.0f,     0.0f,  1.0f,   0.0f,
            0.0f,  10.0f,   -10.0f,      0.0f,  1.0f,   0.0f,

            0.0f,  10.0f,   0.0f,        0.0f,  1.0f,   0.0f,
            10.0f,   10.0f,  0.0f,       0.0f,  1.0f,   0.0f,
            10.0f,  10.0f,  -10.0f,      0.0f,  1.0f,   0.0f,

            // bottom
            0.0f,   0.0f,   0.0f,        0.0f, -1.0f,   0.0f,
            0.0f,  0.0f,   -10.0f,       0.0f, -1.0f,   0.0f,
            10.0f,   0.0f,  -10.0f,      0.0f, -1.0f,   0.0f,

            0.0f,  0.0f,   0.0f,         0.0f, -1.0f,   0.0f,
            10.0f,  0.0f,  -10.0f,       0.0f, -1.0f,   0.0f,
            10.0f,   0.0f,  0.0f,        0.0f, -1.0f,   0.0f,
        };
    }
}

void DoGraphics(const Bsp::CollisionBsp& bsp)
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER);

    // Window mode MUST include SDL_WINDOW_OPENGL for use with OpenGL.
    // (btw, when will C++ get scoped_exit?)
    SDL_Window *window = SDL_CreateWindow(
        "SDL2/OpenGL Demo",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        640,
        480,
        SDL_WINDOW_OPENGL|
        SDL_WINDOW_RESIZABLE|
        SDL_WINDOW_INPUT_GRABBED);

    // Create an OpenGL context associated with the window.

    SDL_GLContext glcontext = SDL_GL_CreateContext(window);

    if (!glcontext)
    {
        printf("Create context failed: %s\n", SDL_GetError());

        return;
    }

    SDL_GL_MakeCurrent(window, glcontext);

    // capture the mouse so no-one else can use it. Means
    // when the app starts the mouse doesn't leave the window
    // http://gamedev.stackexchange.com/questions/33519/trap-mouse-in-sdl
    // NOTE: for a real game, have an option to turn this on or off, so the
    // user can escape the window if running in a window.
    SDL_SetRelativeMouseMode(SDL_TRUE);

    // Now I can init glew.
    {
        glewExperimental = GL_TRUE;

        GLenum err = glewInit();
        if (GLEW_OK != err)
        {
            /* Problem: glewInit failed, something is seriously wrong. */
            fprintf(stderr, "Error: %s\n", glewGetErrorString(err));

        }

        fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    }    

    // RAM: debug for the debug extension
    bool noDebug = true;
    if (noDebug && glewIsExtensionSupported("GL_KHR_debug"))
    {
        noDebug = false;
        glDebugMessageCallback((GLDEBUGPROC) DebugCallback, nullptr);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        GLuint unusedIds = 0;
        glDebugMessageControl(GL_DONT_CARE,
            GL_DONT_CARE,
            GL_DONT_CARE,
            0,
            &unusedIds,
            true);

        printf("GL_KHR_debug\n");
    }

    if (noDebug && glewIsExtensionSupported("GL_ARB_debug_output"))
    {
        noDebug = false;
        // RAM: TODO
        printf("GL_ARB_debug_output\n");
    }

    if (noDebug && glewIsExtensionSupported("GL_AMD_debug_output"))
    {
        noDebug = false;
        // RAM: TODO
        printf("GL_AMD_debug_output\n");
    }

    if (!noDebug)
    {
        glEnable(GL_DEBUG_OUTPUT);
    }

    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Loading vertex data (1 == number of buffers)
    // http://en.wikipedia.org/wiki/Vertex_Buffer_Object
    GLuint triangleVboHandle;
    glGenBuffers(1, &triangleVboHandle);
    glBindBuffer(GL_ARRAY_BUFFER, triangleVboHandle);

    auto triangles = MakeTriangles(bsp);
    glBufferData(
        GL_ARRAY_BUFFER,
        triangles.size() * sizeof(float),
        triangles.data(),
        GL_STATIC_DRAW);

    // try https://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/loading.php
    // http://classes.soe.ucsc.edu/cmps162/Spring12/s12/labnotes/shaders.html
    // FFS I cannot find a simple tutorial!
    // try https://www.khronos.org/webgl/wiki/Tutorial
    // normals are transformed differently, ugh.
    // http://www.songho.ca/opengl/gl_normaltransform.html
    // Using demo code from:
    // http://www.lighthouse3d.com/cg-topics/code-samples/opengl-3-3-glsl-1-5-sample/

    // Even though I pass in a vec3 for the verticies, opengl will
    // expand them to a vec4 filling in the missing values with the
    // template <0,0,0,1>
    // http://stackoverflow.com/questions/8551935/opengl-es-2-0-specifiying-position-attribute-vec3-or-vec4
    static const GLchar* vs[] =
    {
        "//#version 330 core                  \n"
        "uniform mat4 projMatrix;           \n"
        "uniform mat4 modelViewMatrix;      \n"
        "uniform mat4 normalMatrix;         \n"
        "uniform vec3 lightDir;             \n"
        "                                   \n"
        "attribute vec4 vPosition;          \n"
        "attribute vec3 vNormal;            \n"
        "varying vec4 eyeNormal;            \n"
        "varying vec4 eyePosition;          \n"
        "                                   \n"
        "void main()                        \n"
        "{                                  \n"
        "    eyePosition = modelViewMatrix * vPosition;\n"
        "    eyeNormal = normalize(normalMatrix * vec4(vNormal, 0.0));\n"
        "    gl_Position = projMatrix * eyePosition;\n"
        "}"
    };

    static const GLchar* ps[] =
    {
        "//#version 330 core                  \n"
        "varying vec4 eyePosition;               \n"
        "varying vec4 eyeNormal;                 \n"
        "uniform vec4 lightPos;                  \n"
        "void main()                            \n"
        "{                                      \n"
        "    vec4 c = vec4(0.1, 0.1, 1.0, 1.0); \n"
        "    vec3 s = vec3(normalize(lightPos - eyePosition));\n"
        "    vec3 eyeN = vec3(eyeNormal);\n"
        "    float mydot = dot(eyeN, s);\n"
        "    float diff = max(mydot, 0.0);\n"
        "    gl_FragColor = c * diff;       \n"
        "}"
    };

    auto vsO = glCreateShader(GL_VERTEX_SHADER);
    auto psO = glCreateShader(GL_FRAGMENT_SHADER);
    auto pO = glCreateProgram();
    glShaderSource(vsO, 1, vs, nullptr);
    glShaderSource(psO, 1, ps, nullptr);
    glCompileShader(vsO);
    glCompileShader(psO);
    PrintShaderLog(vsO, Log::Shader);
    PrintShaderLog(psO, Log::Shader);

    glAttachShader(pO, vsO);
    glAttachShader(pO, psO);
    glLinkProgram(pO);

    GLint gotLinked;
    glGetProgramiv(pO, GL_LINK_STATUS, &gotLinked);
    if (gotLinked != GL_TRUE)
    {
        PrintShaderLog(pO, Log::Program);
    }

    // Get the attribute addresses so we can setup the state
    // for the vertex buffer correctly.
    // RAM: TODO: Swap to glBindAttribLocation()
    // See http://stackoverflow.com/questions/4635913/explicit-vs-automatic-attribute-location-binding-for-opengl-shaders
    auto lvPosition  = glGetAttribLocation(pO, "vPosition");
    auto lvNormal    = glGetAttribLocation(pO, "vNormal");

    // Get the ids for the uniforms as well
    auto lmodelViewMatrix = glGetUniformLocation(pO, "modelViewMatrix");
    auto lprojMatrix = glGetUniformLocation(pO, "projMatrix");
    auto lnormalMatrix = glGetUniformLocation(pO, "normalMatrix");
    auto llightDir = glGetUniformLocation(pO, "lightdir");
    auto llightPos = glGetUniformLocation(pO, "lightPos");

    // Right, enable the normal and position attribes in the vertex buffer
    // and set what offset they are using.
    // The last item is meant to be a pointer to the data
    // but it's actually an offset. yay.
    glEnableVertexAttribArray(lvPosition);
    glVertexAttribPointer(
                lvPosition,
                3,
                GL_FLOAT,
                GL_FALSE,
                3*2*sizeof(float),
                reinterpret_cast<const void*>(0));

    if (lvNormal >= 0)
    {
        glEnableVertexAttribArray(lvNormal);
        glVertexAttribPointer(
                    lvNormal,
                    3,
                    GL_FLOAT,
                    GL_FALSE,
                    3*2*sizeof(float),
                    reinterpret_cast<const void*>(3*sizeof(float)));
    }



    // On my linux, my wireless xbox360 controller has an unrecognised GUID
    // so add it explicitly. I copied the linux x360 wireless controller
    // but had to use the hat isntead of the buttons for the dpad to work.
    // Also I had to adjust the right joystick axis to 2 and 3
    // https://hg.libsdl.org/SDL/file/b577c4753421/include/SDL_gamecontroller.h#l95
    // https://hg.libsdl.org/SDL/file/tip/src/joystick/SDL_gamecontrollerdb.h#l58
    auto mapAddResult = SDL_GameControllerAddMapping(
                "0000000058626f782047616d65706100,X360 Wireless Controller,a:b0,b:b1,back:b6,dpup:h0.1,dpleft:h0.8,dpdown:h0.4,dpright:h0.2,guide:b8,leftshoulder:b4,leftstick:b9,lefttrigger:a2,leftx:a0,lefty:a1,rightshoulder:b5,rightstick:b10,righttrigger:a5,rightx:a2,righty:a3,start:b7,x:b2,y:b3,");

    std::vector<ControllerIdToPointer> controllers;
    if (mapAddResult >= 0)
    {
        // Add any existing controllers.
        OpenAllControllers(controllers);
    }


    // MAIN SDL LOOP
    bool running = true;
    bool visible = true;
    bool resized = true;
    bool rotatingModel = false;
    Vec3 cameraPosition = {0,0,30};
    Radians yaw = {-90.0f * DegToRad};
    Radians pitch = {0.0f};
    Radians modelRotation = {0.0f};
    auto then = Microseconds();

    while (running)
    {
        SDL_Event e;

        if (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT)
            {
                running = false;
            }

            if (e.type == SDL_KEYUP)
            {
                if (e.key.keysym.sym == SDLK_ESCAPE)
                {
                    running = false;
                }
            }

            if (e.type == SDL_CONTROLLERDEVICEADDED)
            {
                AddController(e.cdevice.which, controllers);
            }

            if (e.type == SDL_CONTROLLERDEVICEREMOVED)
            {
                RemoveController(e.cdevice.which, controllers);
            }

            if (e.type == SDL_WINDOWEVENT)
            {
                switch (e.window.event)
                {
                    case SDL_WINDOWEVENT_SHOWN:
                    case SDL_WINDOWEVENT_EXPOSED:
                    case SDL_WINDOWEVENT_MAXIMIZED:
                    case SDL_WINDOWEVENT_RESTORED:
                    {
                        visible = true;
                        resized = true;
                        break;
                    }

                    case SDL_WINDOWEVENT_HIDDEN:
                    case SDL_WINDOWEVENT_MINIMIZED:
                    {
                        visible = false;
                        break;
                    }

                    case SDL_WINDOWEVENT_RESIZED:
                    {
                        resized = true;
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }
        }

        // Poll for user input.
        auto now = Microseconds();
        if (now < then)
        {
            then = now;
        }

        auto delta = now - then;
        if (delta > globals.tickClientPeriodInMicroseconds)
        {
            PlayerActions actions = GetActions(controllers);

            if (actions.actions[Quit])
            {
                running = false;
            }

            // deal with where we are looking.
            float mouseDelta =
                globals.viewAnglePerMouseMoveUnit *
                globals.viewAngleSpeedPerTick;

            yaw.data     += -actions.mouseX * mouseDelta;
            pitch.data   += -actions.mouseY * mouseDelta;

            // clamp to +- 90 degrees up and down
            // +- Pi for hrizontal
            if (yaw.data < -(2.0f*Pi)) yaw.data += Pi * 2.0f;
            if (yaw.data >  (2.0f*Pi)) yaw.data -= Pi * 2.0f;

            if (pitch.data < -((Pi / 2.0f) - 0.01f)) pitch.data = -((Pi / 2.0f) - 0.01f);
            if (pitch.data >  ((Pi / 2.0f) - 0.01f)) pitch.data =  (Pi / 2.0f) - 0.01f;

            // Ok, forward movement is tied only to the yaw angle, then
            // treat up as up, and left as normal to forward.
            Vec3 forward =
            {
                std::sin(yaw.data),

                // Set this to 0 to prevent "flying".
                -std::sin(pitch.data),
                std::cos(yaw.data),
            };
            forward = Normalise(forward);

            // Fun fact, "forward" is along the +ve z axis.
            // for openGL that's out of the monitor, ie backwards.
            forward = - forward;

            Vec3 left =
            {
                forward.data[2],
                0,
                -forward.data[0],
            };

            Vec3 up =
            {
                0,
                1.0f,
                0,
            };

            Vec3 movement = {0.0f};

            if (actions.actions[Forward])
            {
                movement += forward;
            }
            if (actions.actions[Backward])
            {
                movement -= forward;
            }

            if (actions.actions[StrafeLeft])
            {
                movement += left;
            }
            if (actions.actions[StrafeRight])
            {
                movement -= left;
            }

            if (actions.actions[Up])
            {
                movement += up;
            }
            if (actions.actions[Down])
            {
                movement -= up;
            }

            if (SquareF(movement) > 0.0f)
            {
                cameraPosition +=
                    Normalise(movement) * globals.moveDeltaPerTick;
            }

            then = now;

            // light rotation
            if (rotatingModel)
            {
                modelRotation.data += globals.modelRotationsPerTick;
            }
        }

        // now you can make GL calls.
        if (visible)
        {
            if (resized)
            {
                int width;
                int height;

                SDL_GetWindowSize(window, &width, &height);

                glViewport(0, 0, width, height);

                float ratio = 1.0f * width / height;

                g_projection = ProjectionMatrix(Radians{90 * DegToRad}, ratio, 0.1f, 10000.0f);

                resized = false;
            }

            glClearColor(0.1f,0.2f,0.1f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            auto model = Ry(modelRotation) * Rx({modelRotation.data*2.0f});

            // Get Matricies
            auto view = LookAtRH(cameraPosition, yaw, pitch) * model;

            // Note: I should be able to avoid an inverse
            // calcuation of the view matrix due to the fact
            // that the rotation part is orthonormal and therefore
            // M^T = M^(-1). But for now, I'll just use inverse.
            auto normalXform = Transpose(Inverse(view));

            auto lightPosition = Vec4
            {
                    20.0f,
                    100.0f,
                    20.0f,
                    0.0f,
            };

            auto lightDir = Normalise(-lightPosition);

            // zero out the w row? Yes!
            normalXform.data[3] = {0.0f, 0.0f, 0.0f, 0.0f};

            // Matrix stuff was hard due to confusing documentation on the net.
            // So I figured it out from first principles and wrote my own guide.
            // https://gist.github.com/JodiTheTigger/477c776b30da21a6a123
            auto viewMatrix = Transpose(view);
            auto projMatrix = Transpose(g_projection);
            auto normalMatrix = Transpose(normalXform);

            glUseProgram(pO);
            glUniformMatrix4fv(
                lmodelViewMatrix,
                1,
                false,
                &viewMatrix.data[0].data[0]);

            glUniformMatrix4fv(
                lprojMatrix,
                1,
                false,
                &projMatrix.data[0].data[0]);

            glUniformMatrix4fv(
                lnormalMatrix,
                1,
                false,
                &normalMatrix.data[0].data[0]);

            glUniform3fv(
                llightDir,
                1,
                &lightDir.data[0]);

            glUniform4fv(
                llightPos,
                1,
                &lightPosition.data[0]);

            glDrawArrays(GL_TRIANGLES, 0, triangles.size() / 3);

            SDL_GL_SwapWindow(window);
        }
    }


    // Once finished with OpenGL functions, the SDL_GLContext can be deleted.
    SDL_GL_DeleteContext(glcontext);

    // Close and destroy the window
    SDL_DestroyWindow(window);

    // Clean up
    SDL_Quit();
}
