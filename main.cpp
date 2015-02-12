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
#include "TraceTest.hpp"
#include "VectorMaths3.hpp"
#include "Matrix4x4Maths.hpp"
#include "GLDebug.hpp"
#include "PlaneMaths.hpp"
#include "third-party/getopt/getopt.h"
#include "third-party/ConvexHull/hull.h"

// SDL + OpenGL
#include <SDL2/SDL.h>

// Need ifdef for different platforms.
#include <GL/glew.h>

#include <vector>
#include <memory>

#include <cstdlib>
#include <cstdio>
#include <cmath>

static const float Pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
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
};

const Globals globals;

enum ActionMap
{
    Forward = 0,
    Backward,
    StrafeLeft,
    StrafeRight,
    Up,
    Down,

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

// WTF? Global? What are you? 12 or something? Remove this!
std::vector<ControllerIdToPointer> g_controllers;

static const uint8_t keymap[ActionMap::Count] =
{
    SDL_SCANCODE_W,
    SDL_SCANCODE_S,
    SDL_SCANCODE_A,
    SDL_SCANCODE_D,
    SDL_SCANCODE_SPACE,
    SDL_SCANCODE_Q
};

static const SDL_GameControllerButton padmap[ActionMap::Count] =
{
    SDL_CONTROLLER_BUTTON_DPAD_UP,
    SDL_CONTROLLER_BUTTON_DPAD_DOWN,
    SDL_CONTROLLER_BUTTON_DPAD_LEFT,
    SDL_CONTROLLER_BUTTON_DPAD_RIGHT,
    SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
    SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
};

// Globals
Matrix4x4 g_projection;

void DoGraphics(const Bsp::CollisionBsp& bsp);

int main(int argc, char** argv)
{
    bool benchmark = false;

    // Parse options
    while (auto ch = getopt(argc, argv, "hb"))
    {
        if (ch < 0)
        {
            break;
        }

        if (ch == 'h')
        {
            printf("MessyBsp - By Richard Maxwell\n\n");

            printf("  Renders 'final.bsp' or does a collsion detection bnechmark.\n\n");

            printf("  MessyBsp [--benchmark] [--help]\n\n");

            printf("  -b:  Benchmark 1,000,000 random collision tests\n");
            printf("       against 'final.bsp'. Prints the cost in Microseconds.\n\n");

            printf("  -h:  This help text.\n");
            printf("\n");

            return 0;
        }

        if (ch == 'b')
        {
            benchmark = true;
            break;
        }
    }

    Bsp::CollisionBsp bsp;

    Bsp::GetCollisionBsp("final.bsp", bsp);

    if (benchmark)
    {
        auto result = TimeBspCollision(bsp, 1000);//1000000);

        printf("Trace Took %ld microseconds\n", result.count());

        return 0;
    }

    // RAM: sdl testing.
    DoGraphics(bsp);

    return 0;
}

void AddController(int id)
{
    if (!SDL_IsGameController(id))
    {
        return;
    }

    for (const auto& pad : g_controllers)
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
            g_controllers.push_back(result);
            return;
        }
    }

    if (result.pad != nullptr)
    {
        SDL_GameControllerClose(result.pad);
    }
}

void RemoveController(int instance)
{
    for (const auto& pad : g_controllers)
    {
        if (pad.instance == instance)
        {
            SDL_GameControllerClose(pad.pad);
            return;
        }
    }
}

void OpenAllControllers()
{
    auto count = SDL_NumJoysticks();

    for (int i = 0; i < count; ++i)
    {
        AddController(i);
    }
}

PlayerActions GetActions()
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
    for (const auto& controller : g_controllers)
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
            result.mouseX = SDL_GameControllerGetAxis(
                        controller.pad,
                        SDL_CONTROLLER_AXIS_LEFTX);

            result.mouseY = SDL_GameControllerGetAxis(
                        controller.pad,
                        SDL_CONTROLLER_AXIS_LEFTY);
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

std::vector<Vec3> MeshFromBrush(const Bsp::CollisionBsp& bsp, Bsp::Brush brush)
{
    std::vector<Vec3> mesh;
    std::vector<Plane> planes;

    // Q3 stores plane distance as the distance from the origin along the normal
    // But my maths assume it's D from Ax + By + Cz + D = 0, so I need to invert
    // the distance.
    auto convertD = [] (Plane p)
    {
        return Plane
        {
            p.normal,
            -p.distance
        };
    };

    for (int i = 0; i < brush.sideCount; ++i)
    {
        const auto brushSide = bsp.brushSides[brush.firstBrushSideIndex + i];
        int planeIndex = brushSide.planeIndex;
        planes.push_back(convertD(bsp.planes[planeIndex]));
    }

    const auto verts = VerticiesFromIntersectingPlanes(planes);

    if (!verts.empty())
    {
        const auto& firstVert = verts.data()[0];

        HullDesc hullInfo;

        hullInfo.mFlags        = QF_TRIANGLES;
        hullInfo.mVcount       = verts.size();
        hullInfo.mVertexStride = sizeof(Vec3);
        hullInfo.mVertices     = reinterpret_cast<const float*>(firstVert.data);

        HullResult result;
        HullLibrary library;

        auto createResult = library.CreateConvexHull(hullInfo, result);

        if (createResult == QE_OK)
        {
            for (unsigned face = 0 ; face < result.mNumFaces; ++face)
            {
                auto index = result.mIndices[face * 3 + 0] * 3;

                Vec3 a =
                {
                    result.mOutputVertices[index + 0],
                    result.mOutputVertices[index + 1],
                    result.mOutputVertices[index + 2],
                };

                index = result.mIndices[face * 3 + 1] * 3;

                Vec3 b =
                {
                    result.mOutputVertices[index + 0],
                    result.mOutputVertices[index + 1],
                    result.mOutputVertices[index + 2],
                };

                index = result.mIndices[face * 3 + 2] * 3;

                Vec3 c =
                {
                    result.mOutputVertices[index + 0],
                    result.mOutputVertices[index + 1],
                    result.mOutputVertices[index + 2],
                };

                auto normal = Normalise(Cross(b-a, c-a));

                mesh.push_back(a);
                mesh.push_back(normal);

                mesh.push_back(b);
                mesh.push_back(normal);

                mesh.push_back(c);
                mesh.push_back(normal);
            }
        }
    }

    return mesh;
}

std::vector<float> MakeTriangles(const Bsp::CollisionBsp& bsp)
{
    std::vector<float> result;

    // RAM: oooh, use the bsp!
    unsigned count = 0;
    for (const auto& brushAABB : bsp.brushes)
    {
        auto oneBrush = MeshFromBrush(bsp, brushAABB.brush);

        for (auto v : oneBrush)
        {
            result.push_back(v.data[0]);
            result.push_back(v.data[1]);
            result.push_back(v.data[2]);
        }

        // only do one for now
        if (++count > 0) break;
    }

    return result;


//    // Make a cube.
//    return std::vector<float>
//    {
//        // front (x,y,z,nx,ny,nz)
//        0.0f,   0.0f,   0.0f,        0.0f,   0.0f,   1.0f,
//        10.0f,  0.0f,   0.0f,        0.0f,   0.0f,   1.0f,
//        0.0f,   10.0f,  0.0f,        0.0f,   0.0f,   1.0f,

//        10.0f,  0.0f,   0.0f,        0.0f,   0.0f,   1.0f,
//        10.0f,  10.0f,  0.0f,        0.0f,   0.0f,   1.0f,
//        0.0f,   10.0f,  0.0f,        0.0f,   0.0f,   1.0f,

//        // left
//        0.0f,   0.0f,   0.0f,       -1.0f,  0.0f,   0.0f,
//        0.0f,   0.0f,  -10.0f,      -1.0f,  0.0f,   0.0f,
//        0.0f,   10.0f,  0.0f,       -1.0f,  0.0f,   0.0f,

//        0.0f,  0.0f,   -10.0f,      -1.0f,  0.0f,   0.0f,
//        0.0f,  10.0f,  -10.0f,      -1.0f,  0.0f,   0.0f,
//        0.0f,  10.0f,  0.0f,        -1.0f,  0.0f,   0.0f,

//        // right
//        10.0f,  0.0f,   0.0f,        1.0f,  0.0f,   0.0f,
//        10.0f,  10.0f,  0.0f,        1.0f,  0.0f,   0.0f,
//        10.0f,  0.0f,   -10.0f,      1.0f,  0.0f,   0.0f,

//        10.0f,  0.0f,   -10.0f,      1.0f,  0.0f,   0.0f,
//        10.0f,  10.0f,  0.0f,        1.0f,  0.0f,   0.0f,
//        10.0f,  10.0f,  -10.0f,      1.0f,  0.0f,   0.0f,


//        // back
//        0.0f,   0.0f,   -10.0f,      0.0f,  0.0f,  -1.0f,
//        0.0f,   10.0f,  -10.0f,      0.0f,  0.0f,  -1.0f,
//        10.0f,  0.0f,   -10.0f,      0.0f,  0.0f,  -1.0f,

//        10.0f,  0.0f,   -10.0f,      0.0f,  0.0f,  -1.0f,
//        0.0f,   10.0f,  -10.0f,      0.0f,  0.0f,  -1.0f,
//        10.0f,  10.0f,  -10.0f,      0.0f,  0.0f,  -1.0f,

//        // top
//        0.0f,   10.0f,   0.0f,       0.0f,  1.0f,   0.0f,
//        10.0f,   10.0f,  -10.0f,     0.0f,  1.0f,   0.0f,
//        0.0f,  10.0f,   -10.0f,      0.0f,  1.0f,   0.0f,

//        0.0f,  10.0f,   0.0f,        0.0f,  1.0f,   0.0f,
//        10.0f,   10.0f,  0.0f,       0.0f,  1.0f,   0.0f,
//        10.0f,  10.0f,  -10.0f,      0.0f,  1.0f,   0.0f,

//        // bottom
//        0.0f,   0.0f,   0.0f,        0.0f, -1.0f,   0.0f,
//        0.0f,  0.0f,   -10.0f,       0.0f, -1.0f,   0.0f,
//        10.0f,   0.0f,  -10.0f,      0.0f, -1.0f,   0.0f,

//        0.0f,  0.0f,   0.0f,         0.0f, -1.0f,   0.0f,
//        10.0f,  0.0f,  -10.0f,       0.0f, -1.0f,   0.0f,
//        10.0f,   0.0f,  0.0f,        0.0f, -1.0f,   0.0f,
//    };
}

void DoGraphics(const Bsp::CollisionBsp& bsp)
{
    SDL_Init(SDL_INIT_VIDEO);

    // Window mode MUST include SDL_WINDOW_OPENGL for use with OpenGL.
    // FFS. I wan't scoped_exit!
    SDL_Window *window = SDL_CreateWindow(
        "SDL2/OpenGL Demo", 0, 0, 640, 480,
        SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);

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

    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);

    // TODO: Load vertex data, gen normal data
    // TODO: player controller
    // TODO: use opengl debug function binding
    // TODO: breakpoint at debug error

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

    // TODO: vertex, fragment, program, bind, load
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


    // MAIN SDL LOOP
    bool running = true;
    bool visible = true;
    bool resized = true;
    Vec3 cameraPosition = {0,0,30};
    Radians yaw = {0.0f};
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
                AddController(e.cdevice.which);
            }

            if (e.type == SDL_CONTROLLERDEVICEREMOVED)
            {
                RemoveController(e.cdevice.which);
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
            PlayerActions actions = GetActions();

            // deal with where we are looking.
            float mouseDelta =
                globals.viewAnglePerMouseMoveUnit *
                globals.viewAngleSpeedPerTick;

            yaw.data     += actions.mouseX * mouseDelta;
            pitch.data   += actions.mouseY * mouseDelta;

            // clamp to +- 90 degrees up and down
            // +- Pi for hrizontal
            if (yaw.data < -Pi) yaw.data += Pi * 2;
            if (yaw.data >  Pi) yaw.data -= Pi * 2;

            if (pitch.data < -((Pi / 2.0f) - 0.01)) pitch.data = -Pi / 2.0f;
            if (pitch.data >  ((Pi / 2.0f) - 0.01)) pitch.data =  Pi / 2.0f;

            // Calculate the new Z axis, or another way, the direction
            // we are looking as a vector.
            Vec3 forward =
            {
                std::sin(yaw.data),
                -std::sin(pitch.data)*std::cos(yaw.data),
                std::cos(pitch.data)*std::cos(yaw.data)
            };
            forward = Normalise(forward);

            Vec3 left =
            {
                forward.data[2],
                forward.data[1],
                forward.data[0],
            };

            Vec3 up =
            {
                forward.data[0],
                forward.data[2],
                forward.data[1],
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
            //modelRotation.data += globals.modelRotationsPerTick;
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

            glClearColor(0.1,0.2,0.1,1);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            auto model = Ry(modelRotation) * Rx({modelRotation.data*2.0f});

            // Get Matricies
            auto view = LookAtRH(cameraPosition, yaw, pitch) * model;

            // Assuming world matrix is identity
            // projection * view * model
            // vertex = PVM * in_vertex;
            // Who would have thought that this little line would have
            // caused me so much fucking pain.
            //auto projViewWorld = g_projection * view;

            // Note: I should be able to avoid an inverse
            // calcuation of the view matrix due to the fact
            // that the rotation part is orthonormal and therefore
            // M^T = M^(-1). But for now, I'll just use inverse.
            auto normalXform = Transpose(Inverse(view));

            // lets put the light at a height of 100, circling with a radius
            // of 20, say.
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

            // Stupid OpenGL docs make matrix stuff confusing
            // http://stackoverflow.com/questions/17717600/confusion-between-c-and-opengl-matrix-order-row-major-vs-column-major
            //
            // Sadly, the use of column-major format in the spec and blue book
            // has resulted in endless confusion in the OpenGL programming
            // community. Column-major notation suggests that matrices are not
            // laid out in memory as a programmer would expect.

            // My matricies are stored in crow major format, so I need to
            // Transpose them to be in OpenGL and DirectX's Column major format.
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
