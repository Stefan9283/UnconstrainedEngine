#version 330 core

layout(location = 0) in vec3 positions;
layout(location = 1) in vec3 normals;


uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

out vec3 FragPos;
out vec3 Norm;
void main()
{
    gl_Position = proj * view * model * vec4(positions, 1.0f);
    Norm = mat3(transpose(inverse(model))) * normals;
    FragPos = vec3(model * vec4(positions, 1.0f));
};
