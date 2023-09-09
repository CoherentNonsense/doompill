#include "map.h"

void map_init(Map *map) {
    map->vertices[0] = (Vertex){ 0,  20 };
    map->vertices[1] = (Vertex){ 5,  10 };
    map->vertices[2] = (Vertex){ 20, 10 };
    map->vertices[3] = (Vertex){ 20, 13 };
    map->vertices[4] = (Vertex){ 25, 13 };
    map->vertices[5] = (Vertex){ 40, 20 };
    map->vertices[6] = (Vertex){ 20, 20 };
    map->vertices[7] = (Vertex){ 20, 30 };
    map->vertices[8] = (Vertex){ 0,  30 };
    map->vertices[9] = (Vertex){ 0,  20 };

    map->walls[0] = (Wall){ {&map->vertices[0], &map->vertices[1]}, false };
    map->walls[1] = (Wall){ {&map->vertices[1], &map->vertices[2]}, false };
    map->walls[2] = (Wall){ {&map->vertices[2], &map->vertices[3]}, false };
    map->walls[3] = (Wall){ {&map->vertices[3], &map->vertices[4]}, false };
    map->walls[4] = (Wall){ {&map->vertices[4], &map->vertices[5]}, false };
    map->walls[5] = (Wall){ {&map->vertices[5], &map->vertices[6]}, false };
    map->walls[6] = (Wall){ {&map->vertices[6], &map->vertices[7]}, false };
    map->walls[7] = (Wall){ {&map->vertices[7], &map->vertices[8]}, false };
    map->walls[8] = (Wall){ {&map->vertices[8], &map->vertices[0]}, false };

    // split
    map->walls[9] = (Wall){ {&map->vertices[3], &map->vertices[6]}, true };
}
