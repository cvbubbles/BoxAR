#version 330 core
//out vec4 FragColor;
// 
//in vec2 TexCoords;
//
//uniform sampler2D texture_diffuse1;
//
//void main()
//{    
//    FragColor = texture(texture_diffuse1, vec2(TexCoords.s,1.0f-TexCoords.t));
//}

out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
} fs_in;

uniform sampler2D texture_diffuse1;
uniform vec3 lightPos1;
uniform vec3 lightPos2;
uniform vec3 lightPos3;
uniform vec3 viewPos;
uniform bool blinn;

void main()
{           
    vec3 color = texture(texture_diffuse1, vec2(fs_in.TexCoords.s,1.0f-fs_in.TexCoords.t)).rgb;
    vec3 normal = normalize(fs_in.Normal);
    // ambient
    float ambient = 0.0;

    // diffuse
    //vec3 lightDir1 = normalize(lightPos1 - fs_in.FragPos);
    //vec3 lightDir2 = normalize(lightPos2 - fs_in.FragPos);
    //vec3 lightDir3 = normalize(lightPos3 - fs_in.FragPos);
    vec3 lightDir1 = normalize(lightPos1);
    vec3 lightDir2 = normalize(lightPos2);
    vec3 lightDir3 = normalize(lightPos3);
    
    float diff1 = max(dot(lightDir1, normal), 0.0);
    float diff2 = max(dot(lightDir2, normal), 0.0);
    float diff3 = max(dot(lightDir3, normal), 0.0);
    //vec3 diffuse = diff * color;

    // specular
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);
    //vec3 viewDir = normalize(viewPos);
    vec3 reflectDir1 = reflect(-lightDir1, normal);
    vec3 reflectDir2 = reflect(-lightDir2, normal);
    vec3 reflectDir3 = reflect(-lightDir3, normal);

    float specStrenth = 0.2;
    float spec1 = 0;
    float spec2 = 0;
    float spec3 = 0;
    if(blinn)
    {
        vec3 halfwayDir1 = normalize(lightDir1 + viewDir); 
        vec3 halfwayDir2 = normalize(lightDir2 + viewDir); 
        vec3 halfwayDir3 = normalize(lightDir3 + viewDir); 
        spec1 = pow(max(dot(viewDir, reflectDir1), 0.0), 16.0);
        spec2 = pow(max(dot(viewDir, reflectDir2), 0.0), 16.0);
        spec3 = pow(max(dot(viewDir, reflectDir3), 0.0), 16.0);
    }
    else
    {
        //vec3 reflectDir = reflect(-lightDir, normal);
        spec1 = pow(max(dot(viewDir, reflectDir1), 0.0), 8.0);
        spec2 = pow(max(dot(viewDir, reflectDir2), 0.0), 8.0);
        spec3 = pow(max(dot(viewDir, reflectDir3), 0.0), 8.0);
    }

    float diffuse = (diff1+diff2+diff3)/1.0;
    float spec = (spec1+spec2+spec3)/1.0;
    vec3 specular = spec * vec3(0.3); 
    vec3 res = ambient * color + diffuse * color + specular;
    FragColor = vec4(res, 1.0);
}