//
// Created by felixfifi on 15.05.20.
//

#ifndef RTX_RAYTRACER_CAMERACONTROLLER_H
#define RTX_RAYTRACER_CAMERACONTROLLER_H

#define GLM_FORCE_RADIANS
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <SDL.h>

class CameraController {
public:
    glm::vec3 pos;
    glm::quat orientation;

    float speed = 0.5f;
    float mouseSensitivity;
public:
    CameraController(glm::vec3 pos, float mouseSensitivity, glm::vec3 upDir = glm::vec3(0, 0, 1)
            , glm::vec3 viewDirection = glm::vec3(1, 0, 0));

    CameraController() = default;

    void eventCallbackSDL(const SDL_Event &event);

    glm::mat4 getViewMatrix();

    glm::vec3 getForward();
    glm::vec3 getRight();
    glm::vec3 getUp();
};


#endif //RTX_RAYTRACER_CAMERACONTROLLER_H
