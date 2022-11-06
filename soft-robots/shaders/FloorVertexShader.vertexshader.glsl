#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;

uniform mat4 MVP;

out vec3 vertexPosition;

void main()
{
    gl_Position = MVP * vec4(vertexPosition_modelspace, 1.0);
    
    vertexPosition = vertexPosition_modelspace;
}


