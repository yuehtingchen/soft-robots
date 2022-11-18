//
//  evolveBody.cpp
//  soft-robots
//
//  Created by Alice Chen on 2022/11/17.
//

#include <stdio.h>
#include <math.h>
#include <queue>
#include <set>
#include "utility.h"
#include "evolveBody.hpp"
#include "evolve.hpp"
#include "createObject.hpp"
using namespace std;

extern int numPoints;
extern int numSprings;
extern struct Point points[MAXN];
extern struct Spring springs[MAXN_SQR];

const int generatePercent = 60;
const int DIR[6][3] = {
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
};

const int CUBE[8][3] = {
    {0, 0, 0},
    {1, 0, 0},
    {0, 1, 0},
    {1, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {0, 1, 1},
    {1, 1, 1},
};

void generateOccupancy(
    bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6],
    bool occupancy[MAX_SIDE + 1][MAX_SIDE + 1][MAX_SIDE + 1],
    bool springOccupancy[MAX_SIDE][MAX_SIDE][MAX_SIDE]);
void randomRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
void printRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6]);
float logab(float a, float b);

void generateObject(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    bool occupancy[MAX_SIDE + 1][MAX_SIDE + 1][MAX_SIDE + 1];
    bool springOccupancy[MAX_SIDE][MAX_SIDE][MAX_SIDE];
    generateOccupancy(rules, occupancy, springOccupancy);
    
    numPoints = 0;
    numSprings = 0;
    
    int mapXYZtoPoints[MAX_SIDE + 1][MAX_SIDE + 1][MAX_SIDE + 1];
    
    /* generate points */
    for(int i = 0; i <= MAX_SIDE; i ++)
    {
        for(int j = 0; j <= MAX_SIDE; j ++)
        {
            for(int k = 0; k <= MAX_SIDE; k ++)
            {
                if(occupancy[i][j][k])
                {
                    points[numPoints].pos[0] = i;
                    points[numPoints].pos[1] = j;
                    points[numPoints].pos[2] = k;
                    mapXYZtoPoints[i][j][k] = numPoints;
                    numPoints ++;
                }
            }
        }
    }
    
    for(int i = 0; i < numPoints; i ++)
    {
        points[i].mass = 0.1;
        
        for(int j = 0; j < 3; j ++)
        {
            points[i].velocity[j] = 0;
            points[i].accel[j] = 0;
            points[i].force[j] = 0;
        }
    }
    
    for(int i = 0; i < MAX_SIDE; i ++)
    {
        for(int j = 0; j < MAX_SIDE; j ++)
        {
            for(int k = 0; k < MAX_SIDE; k ++)
            {
                if(!springOccupancy[i][j][k])
                {
                    continue;
                }
                
                struct Point* cubePoints[8];
                int cpIdx = 0;
                for(int cubeIdx = 0; cubeIdx < 8; cubeIdx ++)
                {
                    int x = i + CUBE[cubeIdx][0], y = j + CUBE[cubeIdx][1], z = k + CUBE[cubeIdx][2];
                    cubePoints[cpIdx ++] = &points[mapXYZtoPoints[x][y][z]];
                }
                
                initializeSpringsForEachCube(numSprings, cubePoints, cpIdx);
            }
        }
    }

    printf("%d %d\n", numPoints, numSprings);

    return;
}

void generateOccupancy(
    bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6],
    bool occupancy[MAX_SIDE + 1][MAX_SIDE + 1][MAX_SIDE + 1],
    bool springOccupancy[MAX_SIDE][MAX_SIDE][MAX_SIDE])
{
    
    for(int i = 0; i <= MAX_SIDE; i ++)
    {
        for(int j = 0; j <= MAX_SIDE; j ++)
        {
            for(int k = 0; k <= MAX_SIDE; k ++)
            {
                occupancy[j][k][i] = false;
            }
        }
    }
    
    for(int i = 0; i < MAX_SIDE; i ++)
    {
        for(int j = 0; j < MAX_SIDE; j ++)
        {
            for(int k = 0; k < MAX_SIDE; k ++)
            {
                springOccupancy[j][k][i] = false;
            }
        }
    }
    
    set< tuple<int, int, int> > frontier_explored;
    queue< tuple<int, int, int> > q;
    tuple<int, int, int> tmpPos{MAX_SIDE / 2, MAX_SIDE / 2, 0};
    q.push(tmpPos);
    frontier_explored.insert(tmpPos);
    
    while(!q.empty())
    {
        tuple<int, int, int> point = q.front();
        q.pop();
        
        int x = get<0>(point);
        int y = get<1>(point);
        int z = get<2>(point);
        
        springOccupancy[x][y][z] = true;
        for(int cubeIdx = 0; cubeIdx < 8; cubeIdx ++)
        {
            occupancy[x + CUBE[cubeIdx][0]][y + CUBE[cubeIdx][1]][z + CUBE[cubeIdx][2]] = true;
        }
        
        for(int i = 0; i < 6; i ++)
        {
            tuple<int, int, int> tmpPos{x + DIR[i][0], y + DIR[i][1], z + DIR[i][2]};
            if(rules[x][y][z][i] && !frontier_explored.count(tmpPos))
            {
                q.push(tmpPos);
                frontier_explored.insert(tmpPos);
            }
        }
    }
    
    return;
}

void randomRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    for(int i = 0; i < MAX_SIDE; i ++)
    {
        for(int j = 0; j < MAX_SIDE; j ++)
        {
            for(int k = 0; k < MAX_SIDE; k ++)
            {
                float disttoStart = abs(i - MAX_SIDE / 2) + abs(j - MAX_SIDE / 2) + abs(k);
                float prob = logab(generatePercent, disttoStart + 2) / logab(generatePercent, 2);
                
                for(int dir = 0; dir < 6; dir ++)
                {
                    rules[i][j][k][dir] = random(0, 100) / (100 - int(generatePercent * prob));
                    
                    /* check if index is out of bounds */
                    if(i + DIR[dir][0] < 0 || i + DIR[dir][0] >= MAX_SIDE ||
                       j + DIR[dir][1] < 0 || j + DIR[dir][1] >= MAX_SIDE ||
                       k + DIR[dir][2] < 0 || k + DIR[dir][2] >= MAX_SIDE)
                    {
                        rules[i][j][k][dir] = 0;
                    }
                }
            }
        }
    }
}

/* helper methods */
void printRules(bool rules[MAX_SIDE][MAX_SIDE][MAX_SIDE][6])
{
    for(int i = 0; i < MAX_SIDE; i ++)
    {
        for(int j = 0; j < MAX_SIDE; j ++)
        {
            for(int k = 0; k < MAX_SIDE; k ++)
            {
                printf("%d %d %d\n", i, j, k);
                for(int dir = 0; dir < 6; dir ++)
                {
                    printf("%d ", rules[i][j][k][dir]);
                }
                printf("\n");
            }
        }
    }
}

float logab(float a, float b)
{
    return log2(a) / log2(b);
}
