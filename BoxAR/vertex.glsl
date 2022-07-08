//#version 330 core
//layout (location = 0) in vec3 aPos;
//layout (location = 1) in vec3 aNormal;
//layout (location = 2) in vec2 aTexCoords;

//out vec2 TexCoords;


//uniform mat4 model;
//uniform mat4 view;
//uniform mat4 projection;

//void main()
//{ 
//   TexCoords = aTexCoords;
//    gl_Position = projection * view * model * vec4(aPos, 1.0);
//}


#version 330 core
//layout (location = 0) in vec3 aPos;
//layout (location = 1) in vec3 aNormal;
//layout (location = 2) in vec2 aTexCoords;
//
//// declare an interface block; see 'Advanced GLSL' for what these are.
//out VS_OUT {
//    vec3 FragPos;
//    vec3 Normal;
//    vec2 TexCoords;
//} vs_out;
//
//uniform mat4 projection;
//uniform mat4 view;
//uniform mat4 model;
//
//void main()
//{
//    vs_out.FragPos = aPos;
//    vs_out.Normal = aNormal;
//    vs_out.TexCoords = aTexCoords;
//    gl_Position = projection * view * model * vec4(aPos, 1.0);
//}

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 uNormalMatrix;

out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} vs_out;

void main() {

	vec4 position = view * model * vec4(aPosition, 1.0);
	vs_out.FragPos = position.xyz;

    vec3 normal = normalize(uNormalMatrix * aNormal);
   
	vs_out.TexCoords = aTexCoords;

	vs_out.Normal = normal;
	
	// vertex position
	gl_Position = projection * view * model * vec4(aPosition, 1.0);
}