#pragma once

#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
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
    Camera(GLFWwindow* window) {
        glfwGetCursorPos(window, &lastx_mouse, &lasty_mouse);
        speed = 0.5f;
        mouse_sensitivity = 0.05f;

        reset_camera();

        //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        this->window = window;
        update_proj(window);
        update_view();
    }
    ~Camera() {}

    void reset_camera() {
        position = glm::vec3(0.0f, 0.0f, 10.0f);
        up = glm::vec3(0.0f, 1.0f, 0.0f);

        yaw = -90.0f;
        pitch = 0.0f;
        roll = 0.0f;

        goFront = glm::vec3(0.0f, 0.0f, -1.0f);
        goUp = glm::vec3(0.0f, 1.0f, 0.0f);
        goRight = glm::vec3(1.0f, 0.0f, 0.0f);

        fovy = 45.0f; zNear = 0.01f; zFar = 10000.0f;
    }

    glm::mat4* getviewmatrix() {
        return &view;
    }    
    glm::mat4* getprojmatrix() {
        return &proj;
    }
    glm::vec3 getPosition() {
        return position;
    }

    void Move(GLFWwindow* window) {
        double current_x = lastx_mouse, current_y = lasty_mouse;
        glfwGetCursorPos(window, &lastx_mouse, &lasty_mouse);

        float offsetx = 0, offsety = 0;
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
        {
            offsetx = float(lastx_mouse - current_x);
            offsety = float(current_y - lasty_mouse);
        }

        if (offsetx || offsety)
        {

            offsetx *= mouse_sensitivity;
            offsety *= mouse_sensitivity;

            yaw += offsetx;
            pitch += offsety;

            goFront = glm::normalize(goFront * (float)cos(glm::radians(offsetx)) + goRight * (float)sin(glm::radians(offsetx)));
            goFront = glm::normalize(goFront);

            goFront = glm::normalize(goFront * (float)cos(glm::radians(offsety)) + goUp * (float)sin(glm::radians(offsety)));
            goFront = glm::normalize(goFront);

            goRight = -glm::normalize(glm::cross(goUp, goFront));


            if (pitch > 89.0f)
                pitch = 89.0f;
            if (pitch < -89.0f)
                pitch = -89.0f;

        }

        

        // front back
        if (glfwGetKey(window, GLFW_KEY_W))
            position += speed * goFront;
        if (glfwGetKey(window, GLFW_KEY_S))
            position -= speed * goFront;


        // right left
        if (glfwGetKey(window, GLFW_KEY_D))
            position += speed * goRight;
        if (glfwGetKey(window, GLFW_KEY_A))
            position -= speed * goRight;

        // up down
        if (glfwGetKey(window, GLFW_KEY_SPACE))
            position += speed * goUp;
        if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL))
            position -= speed * goUp;

        // reset
        if (glfwGetKey(window, GLFW_KEY_R))
            reset_camera();


        update_view();
        update_proj(window);
    }

    void update_proj(GLFWwindow* window)
    {
        int w, h;
        glfwGetWindowSize(window, &w, &h);
        //proj = glm::ortho(0.f, (float)w, -(float)h, 0.f, zNear, zFar);
        proj = glm::perspective(glm::radians(fovy), (float)w / (float)h, zNear, zFar);
        //glm::ortho(0.f, (float)w, 0.f, (float)h);
    }
private:
    void update_view()
    {
        view = glm::lookAt(position, position + goFront, goUp);
    }
    

};