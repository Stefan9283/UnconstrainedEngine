//
// Created by Stefan on 02-Mar-21.
//

#ifndef TRIANGLE_DEBUG_H
#define TRIANGLE_DEBUG_H


#include "ImGuizmo.h"
#include "ImSequencer.h"
#include "ImZoomSlider.h"
#include "ImCurveEdit.h"

#include "glm/glm.hpp"
#include "Mesh.h"

void EditTransform(Camera* cam, glm::mat4* cameraView, glm::mat4* cameraProjection, Mesh* mesh);

#endif //TRIANGLE_DEBUG_H
