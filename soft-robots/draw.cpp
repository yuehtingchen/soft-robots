//
//  draw.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/7.
//

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <time.h>
#include <set>
#include <unordered_map>
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

#include "draw.hpp"
#include "common/shader.hpp"
#include "common/controls.hpp"
#include "common/shader.hpp"
#include "utility.h"
#include "setPoints.hpp"
#include "evolve.hpp"

/* window */
const char windowTitle[20] = "A jumping cube";
const int windowWidth = 1920;
const int windowHeight = 1080;

/* video capture */
/* start ffmpeg telling it to expect raw rgba 720p-60hz frames
 -i - tells it to read frames from stdin */
bool writeVideo = false;
const char* cmd = "/opt/homebrew/bin/ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s 3840x2160 -i - "
                  "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip "
                  "/Users/CJChen/Desktop/CourseworksF2022/softRobotDocs/videos/random-cubes/output.mp4";

/* open pipe to ffmpeg's stdin in binary write mode */
int frameWidth = windowWidth * 2;
int frameHeigth = windowHeight * 2;
//FILE* ffmpeg = popen(cmd, "w");
FILE* ffmpeg = NULL;
int* buffer = new int[frameWidth * frameHeigth];

/* lighting */
vec3 initLightPosition = vec3(0, 0, 4);
vec3 initCameraPosition = vec3(-10, -2, 4);

/* color */
const GLfloat color_data[] = {1.0f, 0.3f, 0.3f};

/* object */
extern const double  TIME_STEP;
extern const double MAX_TIME;
extern double T;

int drawEvery = 160;
int drawCount = 0;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN_SQR];

const bool draw_surface = true;

vector< vec3 > box_points_buffer_data;
vector< vec3 > box_vertex_buffer_data;
vector< vec3 > box_line_buffer_data;
vector< vec3 > normal;
vector< vec3 > material_color_buffer_data;
unsigned long box_vertex_line_startIdx = 0;

/* floor */
int floor_buffer_len = 6;
vec3 floor_vertex_data[6] = {
    vec3(-100, -100, 0),
    vec3(100, -100, 0),
    vec3(-100, 100, 0),
    vec3(100, 100, 0),
    vec3(-100, 100, 0),
    vec3(100, -100, 0),
};

int draw( void );
int initializeWindow();
void updateObject();
void drawFloor(GLuint floorProgramID, GLuint groundbuffer, mat4 MVP);
void testObject();

int draw( void )
{
    T = 0;
    updateObject();
    
    if(initializeWindow() != 0)
    {
        return -1;
    }
    
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    /* Create and compile our GLSL program from the shaders */
    GLuint programID = LoadShaders( "/Users/CJChen/Desktop/CourseworksF2022/soft-robots/soft-robots/shaders/SimpleVertexShader.vertexshader.glsl", "/Users/CJChen/Desktop/CourseworksF2022/soft-robots/soft-robots/shaders/SimpleFragmentShader.fragmentshader.glsl" );
    
    /* Create and compile floor shaders */
    GLuint floorProgramID = LoadShaders( "/Users/CJChen/Desktop/CourseworksF2022/soft-robots/soft-robots/shaders/FloorVertexShader.vertexshader.glsl", "/Users/CJChen/Desktop/CourseworksF2022/soft-robots/soft-robots/shaders/FloorFragmentShader.fragmentshader.glsl" );

    GLuint MatrixID = glGetUniformLocation(programID, "MVP");
    GLuint ModelMatrixID = glGetUniformLocation(programID, "Model");
    GLuint ViewMatrixID = glGetUniformLocation(programID, "View");
    
    /* set constant MVP matrix */
    vec3 cameraPosition = initCameraPosition;
    mat4 ProjectionMatrix = perspective(radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
    mat4 ViewMatrix = lookAt(cameraPosition, vec3(0, 0, 0), vec3(0, 0, 1));
    mat4 ModelMatrix = mat4(1.0);
    mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    
    GLuint vertexbuffer;
    GLuint normalVertexbuffer;

    int fragment_color_location = glGetUniformLocation(programID, "uniformColor");
    int light_position_location = glGetUniformLocation(programID, "lightPosition_worldspace");
    
    GLuint groundbuffer;
    glGenBuffers(1, &groundbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, groundbuffer);
    glBufferData(GL_ARRAY_BUFFER, floor_buffer_len * sizeof(vec3), &floor_vertex_data[0], GL_STATIC_DRAW);
    
    do{
        while(drawCount ++ < drawEvery)
        {
            updatePoints();
            T += TIME_STEP;
        }
        drawCount = 0;

        updateObject();
        
        /* Clear the screen */
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        /* compute the MVP matrix from keyboard and mouse input */
//        computeMatricesFromInputs();
//        mat4 ProjectionMatrix = getProjectionMatrix();
//        mat4 ViewMatrix = getViewMatrix();
//        mat4 ModelMatrix = mat4(1.0);
//        mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
        
        /* compute MVP to follow object */
        double centerPos[3];
        getCenterOfMass(points, centerPos);
        cameraPosition = vec3(initCameraPosition.x + centerPos[0], initCameraPosition.y + centerPos[1], initCameraPosition.z);
        ViewMatrix = lookAt(cameraPosition, vec3(centerPos[0], centerPos[1], 0), vec3(0, 0, 1));
        MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
        
        /* draw floor */
        drawFloor(floorProgramID, groundbuffer, MVP);
        
        /* Use our shader */
        glUseProgram(programID);

        /* send transformation to shader */
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
        
        /* draw lines */
        if (!draw_surface)
        {
            glGenBuffers(1, &vertexbuffer);
            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
            glBufferData(GL_ARRAY_BUFFER, box_line_buffer_data.size() * sizeof(vec3), &box_line_buffer_data[0], GL_STATIC_DRAW);
        }
        /* draw surface */
        else
        {
            glGenBuffers(1, &vertexbuffer);
            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
            glBufferData(GL_ARRAY_BUFFER, box_vertex_buffer_data.size() * sizeof(vec3), &box_vertex_buffer_data[0], GL_STATIC_DRAW);

            glGenBuffers(1, &normalVertexbuffer);
            glBindBuffer(GL_ARRAY_BUFFER, normalVertexbuffer);
            glBufferData(GL_ARRAY_BUFFER, normal.size() * sizeof(vec3), &normal[0], GL_STATIC_DRAW);
        }
        
        /* 1rst attribute buffer : vertices
         * Parameters: attribute, size, type, normalized?, stride, array buffer offset
         */
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
        
        /* 2nd attribute buffer : normal vectors */
        if (draw_surface)
        {
            glEnableVertexAttribArray(1);
            glBindBuffer(GL_ARRAY_BUFFER, normalVertexbuffer);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
        }


        /* draw lines */
        if (!draw_surface)
        {
            glDrawArrays(GL_LINES, 0, (unsigned int) box_line_buffer_data.size());
        }
        /* draw triangles */
        else
        {
            glDrawArrays(GL_TRIANGLES, 0, (unsigned int) box_vertex_line_startIdx);
            glDrawArrays(GL_LINES, (int) box_vertex_line_startIdx, (unsigned int) box_vertex_buffer_data.size());
        }
        
        /* color of box */
        glUniform3f(fragment_color_location, color_data[0], color_data[1], color_data[2]);
        
        vec3 lightPosition = vec3(initLightPosition.x + centerPos[0], initLightPosition.y + centerPos[1], initLightPosition.z);
        glUniform3f(light_position_location, lightPosition.x, lightPosition.y, lightPosition.z);
        
        /* 3rd attribute buffer : color of lines
         * Parameters: attribute, size, type, normalized?, stride, array buffer offset
         */
        GLuint colorbuffer;
        glGenBuffers(1, &colorbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glBufferData(GL_ARRAY_BUFFER, material_color_buffer_data.size() * sizeof(vec3), &material_color_buffer_data[0], GL_STATIC_DRAW);
        
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
        
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        
        /* Swap buffers */
        glfwSwapBuffers(window);
        glfwPollEvents();
        
        if(writeVideo)
        {
            glReadPixels(0, 0, frameWidth, frameHeigth, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
            fwrite(buffer, sizeof(int) * frameWidth * frameHeigth, 1, ffmpeg);
        }

    } /* Check if the ESC key was pressed or the window was closed */
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 &&
           T < MAX_TIME);

    /* Cleanup VBO */
    glDeleteProgram(floorProgramID);
    glDeleteBuffers(1, &groundbuffer);
    
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteVertexArrays(1, &VertexArrayID);
    glDeleteProgram(programID);

    /* Close OpenGL window and terminate GLFW */
    glfwTerminate();

    return 0;
}

void drawFloor(GLuint floorProgramID, GLuint groundbuffer, mat4 MVP)
{
    GLuint FloorMatrixID = glGetUniformLocation(floorProgramID, "MVP");
    
    /* use floor shader */
    glUseProgram(floorProgramID);
    
    glUniformMatrix4fv(FloorMatrixID, 1, GL_FALSE, &MVP[0][0]);
    
    /* 0th attribute buffer : ground vertices
     * Parameters: attribute, size, type, normalized?, stride, array buffer offset
     */
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, groundbuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *) 0);
    
    /* draw triangles for ground */
    glDrawArrays(GL_TRIANGLES, 0, (unsigned int) floor_buffer_len);
    glDisableVertexAttribArray(0);
}


int initializeWindow()
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
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    
    return 0;
}

vec3 materialColor(struct Spring spring)
{
    if(spring.muscle)
    {
        double len = -1 * 0.5 * sin(spring.omega * T + spring.c) + 0.5;
        return vec3(1.0f, 0.6 * len, 0);
        
    }
    else if(spring.k > 5000)
    {
        return vec3(0.2f, 0.8f, 0.2f);
    }
    else
    {
        return vec3(0.0f, 0.0f, 1.0f);
    }
}

vec3 materialColor(struct Material material)
{
    struct Spring spring;
    spring.muscle = material.muscle;
    spring.len = material.len;
    spring.omega = material.omega;
    spring.c = material.c;
    spring.k = material.k;
    
    return materialColor(spring);
}

void updateObject()
{
    box_points_buffer_data.clear();
    box_vertex_buffer_data.clear();
    normal.clear();
    box_line_buffer_data.clear();
    material_color_buffer_data.clear();
    box_vertex_line_startIdx = 0;
    
    for(int i = 0; i < numPoints; i ++)
    {
        box_points_buffer_data.push_back(vec3(points[i].pos[0], points[i].pos[1], points[i].pos[2]));
    }
    
    if(!draw_surface)
    {
        for(int i = 0; i < numSprings; i ++)
        {
            box_line_buffer_data.push_back(vec3(springs[i].p1->pos[0], springs[i].p1->pos[1], springs[i].p1->pos[2]));
            box_line_buffer_data.push_back(vec3(springs[i].p2->pos[0], springs[i].p2->pos[1], springs[i].p2->pos[2]));
            
            material_color_buffer_data.push_back(materialColor(springs[i]));
            material_color_buffer_data.push_back(materialColor(springs[i]));
        }
    }
    else
    {
        unordered_map<Point*, vector<int>> hmap;
        
        hmap.clear();
        for(int i = 0; i < numSprings; i ++)
        {
            if(springs[i].len - 1 > 0.001) continue;
            
            if(hmap[springs[i].p1].empty())
            {
                hmap[springs[i].p1] = vector<int>(1, i);
            }
            else
            {
                vector<int> tmp = hmap[springs[i].p1];
                tmp.push_back(i);
                hmap[springs[i].p1] = tmp;
            }
            
            if(hmap[springs[i].p2].empty())
            {
                hmap[springs[i].p2] = vector<int>(1, i);
            }
            else
            {
                vector<int> tmp = hmap[springs[i].p2];
                tmp.push_back(i);
                hmap[springs[i].p2] = tmp;
            }
        }
        
        for(const pair<Point*, vector<int>> p: hmap)
        {
            vector<int> springsIdx = p.second;
            for(int i: springsIdx)
            {
                for(int j: springsIdx)
                {
                    
                    if(i == j)
                    {
                        continue;
                    }
                    
                    set< struct Point* > s;
                    vector< struct Point* > vec;
                    s.insert(springs[i].p1);
                    s.insert(springs[i].p2);
                    s.insert(springs[j].p1);
                    s.insert(springs[j].p2);
                    
                    for(Point* point: s)
                    {
                        vec.push_back(point);
                    }
                    
                    for(Point* point: vec)
                    {
                        material_color_buffer_data.push_back(materialColor(springs[i]));
                        box_vertex_buffer_data.push_back(vec3(point->pos[0], point->pos[1], point->pos[2]));
                    }
                    
                    /* don't know which direction so do both */
                    for(int i = int(vec.size()) - 1; i >= 0; i --)
                    {
                        Point* point = vec[i];
                        material_color_buffer_data.push_back(materialColor(springs[i]));
                        box_vertex_buffer_data.push_back(vec3(point->pos[0], point->pos[1], point->pos[2]));
                    }
                }
            }
        }
        
        /* add lines */
        box_vertex_line_startIdx = box_vertex_buffer_data.size();
        for(int i = 0; i < numSprings; i ++)
        {
            if(springs[i].len - 1 > 0.001) continue;
            box_vertex_buffer_data.push_back(vec3(springs[i].p1->pos[0], springs[i].p1->pos[1], springs[i].p1->pos[2]));
            box_vertex_buffer_data.push_back(vec3(springs[i].p2->pos[0], springs[i].p2->pos[1], springs[i].p2->pos[2]));

            material_color_buffer_data.push_back(vec3(0, 0, 0));
            material_color_buffer_data.push_back(vec3(0, 0, 0));
        }
        
        /* normal vectors */
        for(int i = 0; i < box_vertex_line_startIdx; i += 3)
        {
            vec3 v1 = box_vertex_buffer_data[i];
            vec3 v2 = box_vertex_buffer_data[i + 1];
            vec3 v3 = box_vertex_buffer_data[i + 2];
            vec3 edge1 = v2 - v1;
            vec3 edge2 = v3 - v1;
            normal.push_back((normalize(cross(edge1, edge2))));
        }
    }
    
    return;
}
