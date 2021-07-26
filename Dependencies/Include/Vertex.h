#pragma once

#include <glm/detail/type_quat.hpp>
#include "glm/glm.hpp"

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
};

struct transform {
    glm::vec3 tr, sc;
    glm::quat rot;
};