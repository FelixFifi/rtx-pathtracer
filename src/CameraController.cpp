//
// Created by felixfifi on 15.05.20.
//

#include "CameraController.h"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

CameraController::CameraController(glm::vec3 pos, float mouseSensitivity, glm::vec3 upDir, glm::vec3 viewDirection)
        : pos(pos), mouseSensitivity(mouseSensitivity) {
    orientation = glm::conjugate(glm::quatLookAt(viewDirection, upDir));
    resetStatus();
}

void CameraController::eventCallbackSDL(const SDL_Event &event) {
    switch (event.type) {
        case SDL_EventType::SDL_MOUSEMOTION:
            if (event.motion.state & SDL_BUTTON_RMASK) {
                float yaw, pitch;
                yaw = glm::radians(event.motion.xrel * mouseSensitivity);
                pitch = glm::radians(event.motion.yrel * mouseSensitivity);

                glm::quat qYaw = glm::angleAxis(yaw, orientation * glm::vec3(0.0f, 0.0f, 1.0f));
                glm::quat qPitch = glm::angleAxis(pitch, glm::vec3(1.0f, 0.0f, 0.0f));

                setOrientation(qPitch * (qYaw * orientation));
            }
            break;
        case SDL_EventType::SDL_KEYDOWN:
            switch (event.key.keysym.sym) {
                case SDLK_w:
                    setPos(pos + speed * getForward());
                    break;
                case SDLK_a:
                    setPos(pos - speed * getRight());
                    break;
                case SDLK_s:
                    setPos(pos - speed * getForward());
                    break;
                case SDLK_d:
                    setPos(pos + speed * getRight());
                    break;
                case SDLK_SPACE:
                case SDLK_LSHIFT:
                    setPos(pos + speed * getUp());
                    break;
                case SDLK_LCTRL:
                    setPos(pos - speed * getUp());
                    break;
            }

            if (event.key.keysym.sym == SDLK_w) {
                pos += glm::conjugate(orientation) * glm::vec3(0.0f, 0.0f, -1.0f) * speed;
            }
            break;
    }


}

glm::vec3 CameraController::getForward() {
    return glm::conjugate(orientation) * glm::vec3(0.0f, 0.0f, -1.0f);
}


glm::vec3 CameraController::getRight() {
    return glm::conjugate(orientation) * glm::vec3(1.0f, 0.0f, 0.0f);
}

glm::vec3 CameraController::getUp() {
    return glm::conjugate(orientation) * glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::mat4 CameraController::getViewMatrix() {

    return glm::translate(glm::mat4_cast(orientation), -pos);
}

void CameraController::resetStatus() {
    cameraChanged = false;
}

bool CameraController::hasCameraChanged() const {
    return cameraChanged;
}

void CameraController::setPos(const glm::vec3 &pos) {
    CameraController::pos = pos;
    cameraChanged = true;
}

void CameraController::setOrientation(const glm::quat &orientation) {
    CameraController::orientation = orientation;
    cameraChanged = true;
}
