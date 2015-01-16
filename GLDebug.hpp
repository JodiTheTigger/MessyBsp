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

#pragma once

// Need ifdef for different platforms.
#include <GL/glew.h>

#include <memory>

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

void GLAPIENTRY DebugCallback(
    GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei,
    const GLchar *message,
    void *)
{
    char const* sourceString = "";

    switch (source)
    {
        case GL_DEBUG_SOURCE_API:
        {
                sourceString = "Api";
                break;
        }

        case GL_DEBUG_SOURCE_APPLICATION:
        {
                sourceString = "Application";
                break;
        }

        case GL_DEBUG_SOURCE_OTHER:
        {
                sourceString = "Other";
                break;
        }

        case GL_DEBUG_SOURCE_SHADER_COMPILER:
        {
                sourceString = "Shader Compiler";
                break;
        }

        case GL_DEBUG_SOURCE_THIRD_PARTY:
        {
                sourceString = "Third Party";
                break;
        }

        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
        {
                sourceString = "Window";
                break;
        }

        default:
        {
            sourceString = "Unknown";
            break;
        }
    }

    char const* typeString = "";

    switch (type)
    {
        case GL_DEBUG_TYPE_ERROR:
        {
                typeString = "Error";
                break;
        }

        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        {
                typeString = "Depreciated";
                break;
        }

        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
        {
                typeString = "Undefined";
                break;
        }

        case GL_DEBUG_TYPE_PORTABILITY:
        {
                typeString = "Portability";
                break;
        }

        case GL_DEBUG_TYPE_PERFORMANCE:
        {
                typeString = "Performance";
                break;
        }

        case GL_DEBUG_TYPE_OTHER:
        {
                typeString = "Other";
                break;
        }

        default:
        {
            typeString = "Unknown";
            break;
        }
    }

    char const* severityString = "";

    switch (severity)
    {
        case GL_DEBUG_SEVERITY_HIGH:
        {
            severityString = "HIGH";
            break;
        }

        case GL_DEBUG_SEVERITY_MEDIUM:
        {
            severityString = "Medium";
            break;
        }

        case GL_DEBUG_SEVERITY_LOW:
        {
            severityString = "Low";
            break;
        }

        default:
        {
            typeString = "Unknown";
            break;
        }
    }

    printf(
        "GL_DEBUG: %s: %s (%s): %d: %s\n",
        sourceString,
        severityString,
        typeString,
        id,
        message);
}
