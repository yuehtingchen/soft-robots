#version 330 core

out vec4 color;

in vec3 vertexPosition;

void main()
{
//    color = vec4(0.5f, 0.5f, 0.5f, 1.0 * float(t > 0));
    
    vec4 floorColor;
    
    float tileProporation = 0.5;
    
    if((int(vertexPosition.x * tileProporation + 100000) % 2 == 0 && int(vertexPosition.y * tileProporation + 100000) % 2 == 1) ||
       (int(vertexPosition.x * tileProporation + 100000) % 2 == 1 && int(vertexPosition.y * tileProporation + 100000) % 2 == 0))
    {
        floorColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    }
    else
    {
        floorColor = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    }
    
    color = floorColor;
   
}
