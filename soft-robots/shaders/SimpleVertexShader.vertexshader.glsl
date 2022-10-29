#version 330 core

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;

out vec3 vertexPosition_worldspace;
out vec3 vertexNormal_cameraspace;
out vec3 eyeDirection_cameraspace;
out vec3 lightDirection_cameraspace;

uniform mat4 MVP;
uniform mat4 Model;
uniform mat4 View;
uniform vec3 lightPosition_worldspace;

void main()
{
    /* Output position of the vertex in clip space */
    gl_Position = MVP * vec4(vertexPosition_modelspace, 1);
    vertexPosition_worldspace = (Model * vec4(vertexPosition_modelspace,1)).xyz;
    
    /* vertex Normal to camera space */
    vertexNormal_cameraspace = (View * Model * vec4(vertexNormal_modelspace, 1)).xyz;
    
    eyeDirection_cameraspace = (View * Model * vec4(vertexPosition_modelspace, 1)).xyz - vec3(0, 0, 0);
    
    vec3 lightPosition_cameraspace = (View * Model * vec4(lightPosition_worldspace, 1)).xyz;
    lightDirection_cameraspace = lightPosition_cameraspace - eyeDirection_cameraspace;
    
}


