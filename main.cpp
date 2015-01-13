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
#include "third-party/getopt/getopt.h"

// SDL + OpenGL
#include <SDL2/SDL.h>

// Need ifdef for different platforms.
#include <GL/glew.h>

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
    float moveDeltaPerTick = 2.0f / ticksPerSecond;
    float viewAnglePerMouseMoveUnit = 0.01f;

    // For testing lighting, rotates once every 5 seconds
    float lightRotationsPerTick = ((2 * Pi) / 5.0f) / ticksPerSecond;
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

static const uint8_t keymap[ActionMap::Count] =
{
    SDL_SCANCODE_W,
    SDL_SCANCODE_S,
    SDL_SCANCODE_A,
    SDL_SCANCODE_D,
    SDL_SCANCODE_SPACE,
    SDL_SCANCODE_Q
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

PlayerActions GetActions()
{
    PlayerActions result;

    // Keyboard
    const auto* keys = SDL_GetKeyboardState(nullptr);

    for (unsigned i = 0; i < ActionMap::Count; ++i)
    {
        if (keys[keymap[i]])
        {
            result.actions[i] = true;
        }
        else
        {
            result.actions[i] = false;
        }
    }

    // Mouse
    SDL_GetRelativeMouseState(&result.mouseX, &result.mouseY);

    return result;
}

// This class should be full of platform voodoo if done properly.
// good thing I'm not doing it properly.
uint64_t Microseconds()
{
    return static_cast<uint64_t>(SDL_GetTicks()) * 1000l;
}

std::vector<float> MakeTriangles()
{
    // Make a cube.
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
        0.0f,   0.0f,  -10.0f,      -1.0f,  0.0f,   0.0f,
        0.0f,   10.0f,  0.0f,       -1.0f,  0.0f,   0.0f,

        0.0f,  0.0f,   -10.0f,      -1.0f,  0.0f,   0.0f,
        0.0f,  10.0f,  -10.0f,      -1.0f,  0.0f,   0.0f,
        0.0f,  10.0f,  0.0f,        -1.0f,  0.0f,   0.0f,

        // right
        10.0f,  0.0f,   0.0f,        1.0f,  0.0f,   0.0f,
        10.0f,  10.0f,  0.0f,        1.0f,  0.0f,   0.0f,
        10.0f,  0.0f,   -10.0f,      1.0f,  0.0f,   0.0f,

        10.0f,  0.0f,   -10.0f,      1.0f,  0.0f,   0.0f,
        10.0f,  10.0f,  0.0f,        1.0f,  0.0f,   0.0f,
        10.0f,  10.0f,  -10.0f,      1.0f,  0.0f,   0.0f,


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

enum class Log
{
    Shader,
    Program,
};

void PrintShaderLog(GLuint shader, Log type )
{
    int length = 0;

    glGetShaderiv(
        shader,
        GL_INFO_LOG_LENGTH,
        &length);

    if (length > 0)
    {
        int charsWritten  = 0;
        auto buffer = std::make_unique<char[]>(length);

        if (type == Log::Shader)
        {
            glGetShaderInfoLog(
                shader,
                length,
                &charsWritten,
                buffer.get());
        }
        else
        {
           glGetProgramInfoLog(
                shader,
                length,
                &charsWritten,
                buffer.get());
        }

        printf("%s\n",buffer.get());
    }
}

void CheckGlError(const char *file, int line)
{
    GLenum err (glGetError());

    while (err!=GL_NO_ERROR)
    {
        const char* error;

        switch (err)
        {
            case GL_INVALID_OPERATION:  error="INVALID_OPERATION";  break;
            case GL_INVALID_ENUM:       error="INVALID_ENUM";       break;
            case GL_INVALID_VALUE:      error="INVALID_VALUE";      break;
            case GL_OUT_OF_MEMORY:      error="OUT_OF_MEMORY";      break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
        }

        printf("GL %s - %s:%d\n", error, file, line);
        err=glGetError();
    }
}

#define GLCHECK() CheckGlError(__FILE__,__LINE__)
//#define GLCHECK()

void PrintMatrix(const Matrix4x4& matrix)
{
    for (auto i = 0; i < 4; ++i)
    {
        printf(
            "%3.3f %3.3f %3.3f %3.3f\n",
            matrix.data[i].data[0],
            matrix.data[i].data[1],
            matrix.data[i].data[2],
            matrix.data[i].data[3]);
    }
}

// RAM: Lets try loaind sdl and get a glcontext.
void DoGraphics(const Bsp::CollisionBsp &)
{
    SDL_Init(SDL_INIT_VIDEO);

    // Window mode MUST include SDL_WINDOW_OPENGL for use with OpenGL.
    // FFS. I wan't scoped_exit!
    SDL_Window *window = SDL_CreateWindow(
        "SDL2/OpenGL Demo", 0, 0, 640, 480,
        SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);

    // Create an OpenGL context associated with the window.
    SDL_GLContext glcontext = SDL_GL_CreateContext(window);

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

    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);

    // TODO: Load vertex data, gen normal data
    // TODO: player controller
    // TODO: use opengl debug function binding
    // TODO: breakpoint at debug error

    // Loading vertex data (1 == number of buffers)
    // http://en.wikipedia.org/wiki/Vertex_Buffer_Object
    GLuint triangleVboHandle;
    glGenBuffers(1, &triangleVboHandle);GLCHECK();
    glBindBuffer(GL_ARRAY_BUFFER, triangleVboHandle);GLCHECK();

    auto triangles = MakeTriangles();
    glBufferData(
        GL_ARRAY_BUFFER,
        triangles.size() * sizeof(float),
        triangles.data(),
        GL_STATIC_DRAW);GLCHECK();

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
        "uniform mat4 modelViewProjMatrix;  \n"
        "uniform mat4 normalMatrix;         \n"
        "uniform vec3 lightDir;             \n"
        "                                   \n"
        "attribute vec4 vPosition;          \n"
        "attribute vec4 vNormal;            \n"
        "varying float lightDot;            \n"
        "                                   \n"
        "void main()                        \n"
        "{                                  \n"
        "    gl_Position = modelViewProjMatrix * vPosition;\n"
        "    vec4 transNormal = normalMatrix * vNormal;    \n"
        "    lightDot = max(dot(transNormal.xyz, lightDir), 0.0);\n"
        "}"
    };

    static const GLchar* ps[] =
    {
        "//#version 330 core                  \n"
        "varying float lightDot;                \n"
        "void main()                            \n"
        "{                                      \n"
        "    vec4 c = vec4(0.1, 0.1, 1.0, 1.0); \n"
        "    gl_FragColor = c * lightDot;       \n"
        "}"
    };

    auto vsO = glCreateShader(GL_VERTEX_SHADER);GLCHECK();
    auto psO = glCreateShader(GL_FRAGMENT_SHADER);GLCHECK();
    auto pO = glCreateProgram();GLCHECK();
    glShaderSource(vsO, 1, vs, nullptr);GLCHECK();
    glShaderSource(psO, 1, ps, nullptr);GLCHECK();
    glCompileShader(vsO);GLCHECK();
    glCompileShader(psO);GLCHECK();
    PrintShaderLog(vsO, Log::Shader);
    PrintShaderLog(psO, Log::Shader);

    glAttachShader(pO, vsO);GLCHECK();
    glAttachShader(pO, psO);GLCHECK();
    glLinkProgram(pO);GLCHECK();

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
    auto lvPosition  = glGetAttribLocation(pO, "vPosition");GLCHECK();
    auto lvNormal    = glGetAttribLocation(pO, "vNormal");GLCHECK();

    // Get the ids for the uniforms as well
    auto lmodelViewProjMatrix = glGetUniformLocation(pO, "modelViewProjMatrix");GLCHECK();
    auto lnormalMatrix = glGetUniformLocation(pO, "normalMatrix");GLCHECK();
    auto llightDir = glGetUniformLocation(pO, "lightdir");GLCHECK();

    // Right, enable the normal and position attribes in the vertex buffer
    // and set what offset they are using.
    // The last item is meant to be a pointer to the data
    // but it's actually an offset. yay.
    glEnableVertexAttribArray(lvPosition);GLCHECK();
    glVertexAttribPointer(
                lvPosition,
                3,
                GL_FLOAT,
                GL_FALSE,
                3*2*sizeof(float),
                reinterpret_cast<const void*>(0));GLCHECK();

    if (lvNormal >= 0)
    {
        glEnableVertexAttribArray(lvNormal);GLCHECK();
        glVertexAttribPointer(
                    lvNormal,
                    3,
                    GL_FLOAT,
                    GL_FALSE,
                    3*2*sizeof(float),
                    reinterpret_cast<const void*>(3*sizeof(float)));GLCHECK();
    }


    // MAIN SDL LOOP
    bool running = true;
    bool visible = true;
    bool resized = true;
    Vec3 cameraPosition = {0,0,30};
    Radians yaw = {0.0f};
    Radians pitch = {0.0f};
    Radians lightRotation = {0.0f};
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

            Vec3 movement = {0.0f};

            if (actions.actions[Forward])
            {
                movement.data[2] -= 1.0f;
            }
            if (actions.actions[Backward])
            {
                movement.data[2] += 1.0f;
            }

            if (actions.actions[StrafeLeft])
            {
                movement.data[0] -= 1.0f;
            }
            if (actions.actions[StrafeRight])
            {
                movement.data[0] += 1.0f;
            }

            if (actions.actions[Up])
            {
                movement.data[1] += 1.0f;
            }
            if (actions.actions[Down])
            {
                movement.data[1] -= 1.0f;
            }

            if (SquareF(movement) > 0.0f)
            {
                cameraPosition +=
                    Normalise(movement) * globals.moveDeltaPerTick;
            }

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

            then = now;

            // light rotation
            lightRotation.data += globals.lightRotationsPerTick;
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

                g_projection = ProjectionMatrix(Radians{90 * DegToRad}, ratio, 0.1f, 100.0f);

                resized = false;
            }

            glClearColor(0.1,0.2,0.1,1);GLCHECK();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);GLCHECK();

            // Get Matricies
            auto view = LookAtRH(cameraPosition, yaw, pitch);

            // Assuming world matrix is identity
            // projection * view * model
            // vertex = PVM * in_vertex;
            // Who would have thought that this little line would have
            // caused me so much fucking pain.
            auto projViewWorld = g_projection * view;

            // Note: I should be able to avoid an inverse
            // calcuation of the view matrix due to the fact
            // that the rotation part is orthonormal and therefore
            // M^T = M^(-1). But for now, I'll just use inverse.
            auto normalXform = Transpose(Inverse(view));

            // lets put the light at a height of 100, circling with a radius
            // of 20, say.
            auto lightPosition = Vec4
            {
                    20.0f * std::cos(lightRotation.data),
                    100.0f,
                    20.0f * std::sin(lightRotation.data),
                    0.0f,
            };

            auto lightDir = Normalise(-lightPosition);
            //auto lightDir = Normalise(Vec3{-0.05, -1, -0.3});

            // RAM: lighting debug.
            auto light = normalXform * lightDir;
            light*light;

            // Stupid OpenGL docs make matrix stuff confusing
            // http://stackoverflow.com/questions/17717600/confusion-between-c-and-opengl-matrix-order-row-major-vs-column-major
            //
            // Sadly, the use of column-major format in the spec and blue book
            // has resulted in endless confusion in the OpenGL programming
            // community. Column-major notation suggests that matrices are not
            // laid out in memory as a programmer would expect.

            // My matricies are stored in crow major format, so I need to
            // Transpose them to be in OpenGL and DirectX's Column major format.
            auto openglMatrix = Transpose(projViewWorld);
            auto normalMatrix = Transpose(normalXform);

            glUseProgram(pO);GLCHECK();
            glUniformMatrix4fv(
                lmodelViewProjMatrix,
                1,
                false,
                &openglMatrix.data[0].data[0]);GLCHECK();

            glUniformMatrix4fv(
                lnormalMatrix,
                1,
                false,
                &normalMatrix.data[0].data[0]);GLCHECK();

            glUniform3fv(
                llightDir,
                1,
                &lightDir.data[0]);GLCHECK();

            glDrawArrays(GL_TRIANGLES, 0, triangles.size() / 3);GLCHECK();

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
