//
//  main.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/10/21.
//

#include "setPoints.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
using namespace std;

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
using namespace glm;

#include "common/shader.hpp"
#include "common/controls.hpp"
#include "common/shader.hpp"
#include "utility.h"

/* window */
const char windowTitle[20] = "A jumping cube";
const int windowWidth = 1024;
const int windowHeight = 768;

/* lighting */
vec3 lightPosition = vec3(0, 0, 0);

/* color */
const GLfloat color_data[] = {0.9f, 0.3f, 0.3f};

/* object */
extern int numPoints;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN];

vector< vec3 > box_points_buffer_data;
vector< vec3 > box_vertex_buffer_data;
vector< vec3 > box_line_buffer_data;
vector< vec3 > normal;

void initializeObject();

int main( void )
{
    /* Initialise GLFW */
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    /* Open a window and create its OpenGL context */
    window = glfwCreateWindow( windowWidth, windowHeight, windowTitle, NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    
    /* Initialize GLEW */
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }
    
    /* Ensure we can capture the escape key being pressed below */
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    /* Enable unlimited movement on mouse */
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    /* set mouse at center */
    glfwPollEvents();
    glfwSetCursorPos(window, windowWidth/2, windowHeight/2);

    /* White background */
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    /* Create and compile our GLSL program from the shaders */
    GLuint programID = LoadShaders( "/Users/CJChen/Desktop/CourseworksF2022/soft-robots/soft-robots/shaders/SimpleVertexShader.vertexshader.glsl", "/Users/CJChen/Desktop/CourseworksF2022/soft-robots/soft-robots/shaders/SimpleFragmentShader.fragmentshader.glsl" );

    GLuint MatrixID = glGetUniformLocation(programID, "MVP");
    GLuint ModelMatrixID = glGetUniformLocation(programID, "Model");
    GLuint ViewMatrixID = glGetUniformLocation(programID, "View");
    
    /* set constant MVP matrix */
    vec3 cameraPosition = vec3(4, 3, 9);
    mat4 ProjectionMatrix = perspective(radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
    mat4 ViewMatrix = lookAt(cameraPosition, vec3(0, 0, 0), vec3(0, 1, 0));
    mat4 ModelMatrix = mat4(1.0);
    mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    initializePoints();
    initializeSprings();
    initializeObject();
    
    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, box_line_buffer_data.size() * sizeof(vec3), &box_line_buffer_data[0], GL_STATIC_DRAW);
    
//    GLuint normalVertexbuffer;
//    glGenBuffers(1, &normalVertexbuffer);
//    glBindBuffer(GL_ARRAY_BUFFER, normalVertexbuffer);
//    glBufferData(GL_ARRAY_BUFFER, normal.size() * sizeof(vec3), &normal[0], GL_STATIC_DRAW);
    
    int fragment_color_location = glGetUniformLocation(programID, "fragmentColor");
    int light_position_location = glGetUniformLocation(programID, "lightPosition_worldspace");

    do{

        /* Clear the screen */
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* Use our shader */
        glUseProgram(programID);

        /* compute the MVP matrix from keyboard and mouse input */
//        computeMatricesFromInputs();
//        mat4 ProjectionMatrix = getProjectionMatrix();
//        mat4 ViewMatrix = getViewMatrix();
//        mat4 ModelMatrix = mat4(1.0);
//        mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
//
        /* send transformation to shader */
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

        /* 1rst attribute buffer : vertices
         * Parameters: attribute, size, type, normalized?, stride, array buffer offset
         */
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
        
        /* 2nd attribute buffer : normal vectors */
//        glEnableVertexAttribArray(1);
//        glBindBuffer(GL_ARRAY_BUFFER, normalVertexbuffer);
//        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);

        /* draw lines */
        glDrawArrays(GL_LINES, 0, (unsigned int) box_vertex_buffer_data.size());
        
        /* color of box */
        glUniform3f(fragment_color_location, color_data[0], color_data[1], color_data[2]);
        glUniform3f(light_position_location, lightPosition.x, lightPosition.y, lightPosition.z);
        
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        
        /* Swap buffers */
        glfwSwapBuffers(window);
        glfwPollEvents();

    } /* Check if the ESC key was pressed or the window was closed */
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    /* Cleanup VBO */
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);

    /* Close OpenGL window and terminate GLFW */
    glfwTerminate();

    return 0;
}

void initializeObject()
{
    
    for(int i = 0; i < numPoints; i ++)
    {
        box_points_buffer_data.push_back(vec3(points[i].pos[0], points[i].pos[1], points[i].pos[2]));
    }
    
    for(int i = 0; i < box_points_buffer_data.size() - 2; i ++)
    {
        for(int j = i + 1; j < box_points_buffer_data.size() - 1; j ++)
        {
            for(int k = j + 1; k < box_points_buffer_data.size(); k ++)
            {
                box_vertex_buffer_data.push_back(box_points_buffer_data[i]);
                box_vertex_buffer_data.push_back(box_points_buffer_data[j]);
                box_vertex_buffer_data.push_back(box_points_buffer_data[k]);
            }
        }
    }
    
    for(int i = 0; i < box_vertex_buffer_data.size(); i += 3)
    {
        vec3 v1 = box_vertex_buffer_data[i];
        vec3 v2 = box_vertex_buffer_data[i + 1];
        vec3 v3 = box_vertex_buffer_data[i + 2];
        vec3 edge1 = v2 - v1;
        vec3 edge2 = v3 - v1;
        normal.push_back(normalize(cross(edge1, edge2)));
    }
    
    for(int i = 0; i < box_points_buffer_data.size() - 1; i ++)
    {
        for(int j = i + 1; j < box_points_buffer_data.size(); j ++)
        {
            box_line_buffer_data.push_back(box_points_buffer_data[i]);
            box_line_buffer_data.push_back(box_points_buffer_data[j]);
        }
    }
    
    return;
}
