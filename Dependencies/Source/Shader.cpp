//
// Created by Stefan on 22-Feb-21.
//
#include "Shader.h"


ShaderProgramSource Shader::ParseShader(const char* filepath_v, const char* filepath_f)
{
    ShaderProgramSource source;
    source.fragmentShader = (char*)malloc(sizeof(char) * 9999999);
    source.vertexShader = (char*)malloc(sizeof(char) * 9999999);

    char line[1000];
    memset(line, 0, 1000);
    FILE* f = fopen(filepath_v, "r");
    if(f) {
        memset(source.vertexShader, 0, 9999999);
        while (fgets(line, 1000, f)) {
            line[strlen(line) - 1] = '\0';
            strcat(source.vertexShader, line);
            strcat(source.vertexShader, "\n");
            memset(line, 0, 1000);
        }
        fclose(f);
    }

    f = fopen(filepath_f, "r");
    if(f) {
        memset(source.fragmentShader, 0, 9999999);
        while (fgets(line, 1000, f)) {
            line[strlen(line) - 1] = '\0';
            strcat(source.fragmentShader, line);
            strcat(source.fragmentShader, "\n");
            memset(line, 0, 1000);
        }
        fclose(f);
    }
    source.vertexShader[strlen(source.vertexShader)] = '\0';
    source.fragmentShader[strlen(source.fragmentShader)] = '\0';
    return source;
}

unsigned int Shader::CreateShader(const char* vertexShader, const char* fragmentShader) {
    unsigned int program = glCreateProgram();
    unsigned int vs;
    vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
    unsigned int fs;
    fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    int result;
    glGetProgramiv(program, GL_LINK_STATUS, &result);
    if (result == GL_FALSE)
    {
        int infologLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infologLength);
        GLchar buffer[8888];
        GLsizei charsWritten = 0;
        glGetProgramInfoLog(program, infologLength, &charsWritten, buffer);
        std::cout << buffer << std::endl;
    }

    glValidateProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}

unsigned int Shader::CompileShader(unsigned int type, const char* source) {
    unsigned int shaderID = glCreateShader(type);
    const char *src = source;
    glShaderSource(shaderID, 1, &src, NULL);
    glCompileShader(shaderID);

    int result;
    glGetShaderiv(shaderID, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE) {
        int length;
        glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &length);
        char *message = (char *) alloca(length * sizeof(char));
        glGetShaderInfoLog(shaderID, length, &length, message);
        std::cout << message << std::endl;
    }

    return shaderID;
}