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
#include <cstdlib>

#include <cstdio>

// SDL + OpenGL
#include <SDL2/SDL.h>

// Need ifdef for different platforms.
#include <GL/glew.h>

#include "third-party/getopt/getopt.h"

void DoGraphics();

int main(int argc, char** argv)
{
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
            Bsp::CollisionBsp bsp;

            Bsp::GetCollisionBsp("final.bsp", bsp);

            auto result = TimeBspCollision(bsp, 1000);//1000000);

            printf("Trace Took %ld microseconds\n", result.count());

            return 0;
        }
    }

    // RAM: sdl testing.
    DoGraphics();

    return 0;
};


// RAM: Lets try loaind sdl and get a glcontext.
void DoGraphics()
{
    SDL_Init(SDL_INIT_VIDEO);

    // Window mode MUST include SDL_WINDOW_OPENGL for use with OpenGL.
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

    // TODO: Load vertex data, gen normal data
    // TODO: player controller
    // TODO: use opengl debug function binding
    // TODO: breakpoint at debug error

    // Loading vertex data (1 == number of buffers)
    // http://en.wikipedia.org/wiki/Vertex_Buffer_Object
    GLuint triangleVboHandle;
    glGenBuffers(1, &triangleVboHandle);
    glBindBuffer(GL_ARRAY_BUFFER, triangleVboHandle);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);

    // MAIN SDL LOOP
    bool running = true;
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
        }

        // now you can make GL calls.
        glClearColor(0,1,0,1);
        glClear(GL_COLOR_BUFFER_BIT);

        // glEnableVertexAttribArray
        // glVertexAttribPointer
        // glEnableClientState ??
        // glDrawArrays

        SDL_GL_SwapWindow(window);
    }


    // Once finished with OpenGL functions, the SDL_GLContext can be deleted.
    SDL_GL_DeleteContext(glcontext);

    // Close and destroy the window
    SDL_DestroyWindow(window);

    // Clean up
    SDL_Quit();
}
