#pragma once

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/detail/type_quat.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <ctime>
#include <chrono>
#include <limits>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <stack>
#include <queue>
#include <deque>
#include <unordered_set>
#include <future>         // std::async, std::future

#include <iomanip>

#include <imconfig.h>
#include <imgui.h>
#include <ImGuizmo.h>
#include <ImSequencer.h>
#include <ImZoomSlider.h>
#include <ImCurveEdit.h>

#include <Eigen/Dense>

#include "Debug.h"

  #define EPS 0.00001
#define WEAK_EPS 0.0001
#define INF 2000000000.0f
