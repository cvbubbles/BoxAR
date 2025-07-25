#version 330 core
out vec4 FragColor;
 
in vec2 TexCoords;

uniform sampler2D texture_diffuse1;

void main()
{    
    FragColor = texture(texture_diffuse1, vec2(TexCoords.s,1.0f-TexCoords.t));
}

