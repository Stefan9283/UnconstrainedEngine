//
// Created by Stefan on 21-Feb-21.
//

#ifndef TRIANGLE_OBJLOAD_H
#define TRIANGLE_OBJLOAD_H

#endif //TRIANGLE_OBJLOAD_H

#include "Common.h"
#include "Mesh.h"

typedef struct objVert {
    unsigned int pos, norm, tex;
} objVert;

Mesh* readObj(const char* filepath);
