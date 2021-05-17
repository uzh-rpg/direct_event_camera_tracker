#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;
layout (location = 5) in vec3 aVertColor;

out vec2 TexCoords;
out vec3 Normal;
out vec3 VertColor;

uniform mat4 MVP;
uniform mat4 M;

void main()
{
    gl_Position = MVP * vec4(aPos, 1.0);
    TexCoords = aTexCoords;

    Normal = (M * vec4(aNormal, 1.0)).xyz;

    VertColor = aVertColor;
}
