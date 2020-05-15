//
// Created by felixfifi on 15.05.20.
//

#include "CameraController.h"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

CameraController::CameraController(glm::vec3 pos, float mouseSensitivity, glm::vec3 upDir, glm::vec3 viewDirection)
        : pos(pos), mouseSensitivity(mouseSensitivity) {
    orientation = glm::conjugate(glm::quatLookAt(viewDirection, upDir));
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
                orientation = qPitch * (qYaw * orientation);
            }
            break;
        case SDL_EventType::SDL_KEYDOWN:
            switch (event.key.keysym.sym) {
                case SDLK_w:
                    pos += speed * getForward();
                    break;
                case SDLK_a:
                    pos -= speed * getRight();
                    break;
                case SDLK_s:
                    pos -= speed * getForward();
                    break;
                case SDLK_d:
                    pos += speed * getRight();
                    break;
                case SDLK_SPACE:
                case SDLK_LSHIFT:
                    pos += speed * getUp();
                    break;
                case SDLK_LCTRL:
                    pos -= speed * getUp();
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
