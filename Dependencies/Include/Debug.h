//
// Created by Stefan on 02-Mar-21.
//

#ifndef TRIANGLE_DEBUG_H
#define TRIANGLE_DEBUG_H

#include "Common.h"

class Camera;
class Mesh;

void EditTransform(Camera* cam, glm::mat4* cameraView, glm::mat4* cameraProjection, Mesh* mesh);

#endif //TRIANGLE_DEBUG_H
