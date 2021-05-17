#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 VertColor;

uniform sampler2D texture_diffuse1;

void main()
{
    // material properties
    vec4 MaterialColor = texture(texture_diffuse1, TexCoords);

    FragColor = vec4(VertColor, 1.0) + MaterialColor;
}
