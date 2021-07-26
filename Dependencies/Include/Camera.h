#pragma once
#include "Common.h"

class Camera
{
public:
	glm::vec3 goRight, goUp, goFront;
    float speed, mouse_sensitivity;
    double lastx_mouse, lasty_mouse;
    float yaw, pitch, roll;
	glm::vec3 position, up;
    glm::mat4 proj, view;
    float fovy, zNear, zFar;
    GLFWwindow* window;
    Camera(GLFWwindow* window);
    ~Camera();

    void reset_camera();

    glm::mat4* getviewmatrix();
    glm::mat4* getprojmatrix();
    glm::vec3 getPosition();

    void Move(GLFWwindow* window);

    void update_proj(GLFWwindow* window);
private:
    void update_view();
    

};