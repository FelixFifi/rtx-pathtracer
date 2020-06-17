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
private:
    bool cameraChanged;
public:
    glm::vec3 pos;

    void setPos(const glm::vec3 &pos);

    void setOrientation(const glm::quat &orientation);

    glm::quat orientation;

    float vfov;
    float aspectRatio;

    float speed = 0.5f;
    float mouseSensitivity;
public:
    CameraController(glm::vec3 pos, float mouseSensitivity, glm::vec3 upDir = glm::vec3(0, 0, 1),
                     glm::vec3 viewDirection = glm::vec3(1, 0, 0), float aspectRatio = 1);

    CameraController() = default;

    void eventCallbackSDL(const SDL_Event &event);

    glm::mat4 getViewMatrix();

    glm::vec3 getForward();
    glm::vec3 getRight();
    glm::vec3 getUp();
    bool hasCameraChanged() const;

    void resetStatus();

    void lookAt(glm::vec3 origin, glm::vec3 target, glm::vec3 upDir);

    glm::mat4 getProjMatrix();
};


#endif //RTX_RAYTRACER_CAMERACONTROLLER_H
