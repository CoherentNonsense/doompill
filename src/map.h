#ifndef MAP_H
#define MAP_H

#include "types.h"

typedef struct Node {
    struct Node* left;
    struct Node* right;
} Node;

typedef struct BSP {
    Node* root;
} BSP;

typedef struct Vertex {
    fp32 x;
    fp32 y;
} Vertex;

typedef struct Wall {
    Vertex* vertices[2];
    bool portal;
} Wall;



typedef struct Map {
    Vertex vertices[10];
    Wall walls[10];
} Map;

void map_init(Map* map);

#endif // !MAP_H
