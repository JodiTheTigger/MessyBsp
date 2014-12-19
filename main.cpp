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

std::vector<float> MakeTrianglesAndNormals()
{
    return std::vector<float>
    {
        // x,y,z,nx,ny,nz (RH coords, +ve z comes out of monitor)
        0.0f,
        0.0f,
        0.0f,

        0.0f,
        0.0f,
        1.0f,


        5.0f,
        10.0f,
        0.0f,

        0.0f,
        0.0f,
        1.0f,


        10.0f,
        0.0f,
        0.0f,

        0.0f,
        0.0f,
        1.0f,
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

Matrix4x4 ProjectionMatrix(
    Radians fieldOfView,
    float aspect,
    float nearDistance,
    float farDistance)
{
    //
    // General form of the Projection Matrix
    //
    // uh = Cot( fov/2 ) == 1/Tan(fov/2)
    // uw / uh = 1/aspect
    //
    //   uw         0       0       0
    //    0        uh       0       0
    //    0         0      f/(f-n)  1
    //    0         0    -fn/(f-n)  0

    float frustumDepth  = farDistance - nearDistance;
    float oneOverDepth  = 1.0f / frustumDepth;
    float f             = 1.0f / std::tan(0.5f * fieldOfView.data);

    return Matrix4x4
    {{
        {
            f / aspect,
            0.0f,
            0.0f,
            0.0f
        },
        {
            0.0f,
            f,
            0.0f,
            0.0f
        },
        {
            0.0f,
            0.0f,
            farDistance * oneOverDepth,
            1.0f
        },
        {
            0.0f,
            0.0f,
            (-farDistance * nearDistance) * oneOverDepth,
            0.0f
        },
    }};
}

Matrix4x4 inline Translation(Vec3 offset)
{
    return Matrix4x4
    {{
        {1.0f, 0.0f, 0.0f, offset.data[0]},
        {0.0f, 1.0f, 0.0f, offset.data[1]},
        {0.0f, 0.0f, 1.0f, offset.data[2]},
        {0.0f, 0.0f, 0.0f, 1.0f}
    }};
}

Matrix4x4 LookAt(
        Vec3 position,
        Vec3 positionBeenLookedAt,
        Vec3 up = {0.0f, 1.0f, 0.0f})
{
    auto direction  = Normalise(positionBeenLookedAt - position);
    auto right      = Normalise(Cross(direction, up));
    auto newUp      = Normalise(Cross(right, direction));

    auto result = Matrix4x4
    {{
        {right.data[0], newUp.data[0], -direction.data[0], 0.0f},
        {right.data[1], newUp.data[1], -direction.data[1], 0.0f},
        {right.data[2], newUp.data[2], -direction.data[2], 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f},
    }};

    return result * Translation(-position);
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

    // TODO: Load vertex data, gen normal data
    // TODO: player controller
    // TODO: use opengl debug function binding
    // TODO: breakpoint at debug error

    // Loading vertex data (1 == number of buffers)
    // http://en.wikipedia.org/wiki/Vertex_Buffer_Object
    GLuint triangleVboHandle;
    glGenBuffers(1, &triangleVboHandle);GLCHECK();
    glBindBuffer(GL_ARRAY_BUFFER, triangleVboHandle);GLCHECK();

    auto triangles = MakeTrianglesAndNormals();
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
    const GLchar* vs = "\
    uniform mat4 modelViewProjMatrix;\
    uniform mat4 normalMatrix;\
    uniform vec3 lightDir;\
 \
    attribute vec4 vNormal;\
    attribute vec4 vPosition;\
 \
    varying float dot;\
 \
    void main()\
    {\
        gl_Position = modelViewProjMatrix * vPosition;\
        vec4 transNormal = normalMatrix * vNormal;\
        dot = max(dot(transNormal.xyz, lightDir), 0.0);\
    }";

    const GLchar* ps = "\
    varying float dot; \
    void main()\
    {\
        vec4 c = vec4(0.1, 0.1, 1.0, 1.0);\
        \
        gl_FragColor = c * dot;\
    }";

    auto vsO = glCreateShader(GL_VERTEX_SHADER);GLCHECK();
    auto psO = glCreateShader(GL_FRAGMENT_SHADER);GLCHECK();
    auto pO = glCreateProgram();GLCHECK();
    glShaderSource(vsO, 1, &vs, nullptr);GLCHECK();
    glShaderSource(psO, 1, &ps, nullptr);GLCHECK();
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
    auto lvNormal    = glGetAttribLocation(pO, "vNormal");GLCHECK();
    auto lvPosition  = glGetAttribLocation(pO, "vPosition");GLCHECK();

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

                // 1.3 ~= less than 90 degrees in radians.
                g_projection = ProjectionMatrix(Radians{1.3f}, ratio, 0.1f, 10.0f);

                resized = false;
            }

            glClearColor(0,1,0,1);GLCHECK();
            glClear(GL_COLOR_BUFFER_BIT);GLCHECK();

            // Get View Matrix
            // Camera is 5 units behind your back
            // looking at 20 units behind the monitor
            auto view = LookAt(Vec3{0,0,5}, Vec3{0,0,-20});

            // Assuming world matrix is identity
            // projection * view * model
            // vertex = MVP * in_vertex;
            auto projViewWorld = g_projection * view;

            // RAM: TODO: Fix that the normal is full of nans
            auto normalXform = Transpose(Inverse(projViewWorld));
            auto lightDir = Normalise(Vec3{-0.05, -1, -0.3});

            // Stupid OpenGL docs make matrix stuff confusing
            // http://stackoverflow.com/questions/17717600/confusion-between-c-and-opengl-matrix-order-row-major-vs-column-major
            glUseProgram(pO);GLCHECK();
            glUniformMatrix4fv(
                lmodelViewProjMatrix,
                1,
                false,
                &projViewWorld.data[0].data[0]);GLCHECK();

            glUniformMatrix4fv(
                lnormalMatrix,
                1,
                false,
                &normalXform.data[0].data[0]);GLCHECK();

            glUniform3fv(
                llightDir,
                1,
                &lightDir.data[0]);GLCHECK();

            glDrawArrays(GL_TRIANGLES, 0, 3);GLCHECK();

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
