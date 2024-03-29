#version 330 core

in vec3 vertexPosition_worldspace;
in vec3 vertexNormal_cameraspace;
in vec3 eyeDirection_cameraspace;
in vec3 lightDirection_cameraspace;
in vec3 fragmentColor;

out vec3 color;

uniform vec3 uniformColor;

uniform mat4 Model;
uniform mat4 View;
uniform vec3 lightPosition_worldspace;

void main()
{
    vec3 lightColor = vec3(1.0f, 1.0f, 1.0f);
    float lightPower = 0.8f;
    
    vec3 ambientColor = vec3(0.8,0.8,0.8) * fragmentColor;
    
    /* distance of object to light source */
    float dist = length(lightPosition_worldspace - vertexPosition_worldspace);
    
    vec3 n = normalize(vertexNormal_cameraspace);
    vec3 l = normalize(lightDirection_cameraspace);
    float cosTheta = clamp(dot(n, l), 0, 1);
    
    vec3 e = normalize(eyeDirection_cameraspace);
    vec3 r = reflect(-l,n);
    float cosAlpha = clamp(dot(e, r), 0, 1);
    
    color =
        /* Ambient : simulates indirect lighting */
        ambientColor +
        /* Diffuse : "color" of the object */
        fragmentColor * lightColor * lightPower * cosTheta * (1 / (dist * dist)); //+
        /* Specular : reflective highlight, like a mirror */
//        lightColor * lightColor * lightPower * pow(cosAlpha,5) / (dist * dist);
    
    color = fragmentColor;
}
