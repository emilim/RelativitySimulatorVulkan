#pragma once

//libs
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// std
#include <array>
#include <stdexcept>
#include <math.h>
#include <iostream>

namespace lve {
	class Model {
	public:
        Model(const Model&) = delete;
        Model& operator=(const Model&) = delete;
        Model(Model&&) = default;
        Model& operator=(Model&&) = default;

        static std::unique_ptr<LveModel> createCircleModel(LveDevice& device, unsigned int numSides) {
            std::vector<LveModel::Vertex> uniqueVertices{};
            for (int i = 0; i < numSides; i++) {
                float angle = i * glm::two_pi<float>() / numSides;
                uniqueVertices.push_back({ {glm::cos(angle), glm::sin(angle), 0.f} });
            }
            uniqueVertices.push_back({});  // adds center vertex at 0, 0

            std::vector<LveModel::Vertex> vertices{};
            for (int i = 0; i < numSides; i++) {
                vertices.push_back(uniqueVertices[i]);
                vertices.push_back(uniqueVertices[(i + 1) % numSides]);
                vertices.push_back(uniqueVertices[numSides]);
            }
            return std::make_unique<LveModel>(device, vertices);
        }
		static void sierpinski(std::vector<LveModel::Vertex>& vertices, int depth, glm::vec3 left, glm::vec3 right, glm::vec3 top) {
            if (depth <= 0) {
                vertices.push_back({ top, {1.0, 0.0, 0.0} });
                vertices.push_back({ right, {0.0, 1.0, 0.0} });
                vertices.push_back({ left, {0.0, 0.0, 1.0} });
            }
            else {
                auto leftTop = 0.5f * (left + top);
                auto rightTop = 0.5f * (right + top);
                auto leftRight = 0.5f * (left + right);
                sierpinski(vertices, depth - 1, left, leftRight, leftTop);
                sierpinski(vertices, depth - 1, leftRight, right, rightTop);
                sierpinski(vertices, depth - 1, leftTop, rightTop, top);
            }
        }
        static std::unique_ptr<LveModel> createRectangleModel(LveDevice& device, glm::vec3 color) {
            std::vector<LveModel::Vertex> vertices{};
            //vertices for a rectangle
            vertices.push_back({ glm::vec3{ -1.0f, -1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 1.0f, -1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -1.0f, 1.0f, 0.0f }, color });

            vertices.push_back({ glm::vec3{ 1.0f, -1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 1.0f, 1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -1.0f, 1.0f, 0.0f }, color });

            return std::make_unique<LveModel>(device, vertices);
        }
		static std::unique_ptr<LveModel> createCrossModel(LveDevice& device, glm::vec3 color) {
            std::vector<LveModel::Vertex> vertices{};
            //vertices for a cross
            vertices.push_back({ glm::vec3{ -0.5f, -1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -0.5f, 1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 0.5f, 1.0f, 0.0f }, color  });

            vertices.push_back({ glm::vec3{ 0.5f, -1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 0.5f, 1.0f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -0.5f, -1.0f, 0.0f }, color });

            vertices.push_back({ glm::vec3{ -1.0f, -0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -1.0f, 0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -0.5f, 0.5f, 0.0f }, color });

            vertices.push_back({ glm::vec3{ -0.5f, 0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -1.0f, -0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ -0.5f, -0.5f, 0.0f }, color });

            vertices.push_back({ glm::vec3{ 0.5f, -0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 1.0f, -0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 0.5f, 0.5f, 0.0f }, color });

            vertices.push_back({ glm::vec3{ 1.0f, -0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 0.5f, 0.5f, 0.0f }, color });
            vertices.push_back({ glm::vec3{ 1.0f, 0.5f, 0.0f }, color });


            return std::make_unique<LveModel>(device, vertices);
        }
        static std::unique_ptr<LveModel> createCubeModel(LveDevice& device, glm::vec3 offset) {
            std::vector<LveModel::Vertex> vertices{

                // left face (white)
                {{-.5f, -.5f, -.5f}, {.9f, .9f, .9f}},
                {{-.5f, .5f, .5f}, {.9f, .9f, .9f}},
                {{-.5f, -.5f, .5f}, {.9f, .9f, .9f}},
                {{-.5f, -.5f, -.5f}, {.9f, .9f, .9f}},
                {{-.5f, .5f, -.5f}, {.9f, .9f, .9f}},
                {{-.5f, .5f, .5f}, {.9f, .9f, .9f}},

                // right face (yellow)
                {{.5f, -.5f, -.5f}, {.8f, .8f, .1f}},
                {{.5f, .5f, .5f}, {.8f, .8f, .1f}},
                {{.5f, -.5f, .5f}, {.8f, .8f, .1f}},
                {{.5f, -.5f, -.5f}, {.8f, .8f, .1f}},
                {{.5f, .5f, -.5f}, {.8f, .8f, .1f}},
                {{.5f, .5f, .5f}, {.8f, .8f, .1f}},

                // top face (orange, remember y axis points down)
                {{-.5f, -.5f, -.5f}, {.9f, .6f, .1f}},
                {{.5f, -.5f, .5f}, {.9f, .6f, .1f}},
                {{-.5f, -.5f, .5f}, {.9f, .6f, .1f}},
                {{-.5f, -.5f, -.5f}, {.9f, .6f, .1f}},
                {{.5f, -.5f, -.5f}, {.9f, .6f, .1f}},
                {{.5f, -.5f, .5f}, {.9f, .6f, .1f}},

                // bottom face (red)
                {{-.5f, .5f, -.5f}, {.8f, .1f, .1f}},
                {{.5f, .5f, .5f}, {.8f, .1f, .1f}},
                {{-.5f, .5f, .5f}, {.8f, .1f, .1f}},
                {{-.5f, .5f, -.5f}, {.8f, .1f, .1f}},
                {{.5f, .5f, -.5f}, {.8f, .1f, .1f}},
                {{.5f, .5f, .5f}, {.8f, .1f, .1f}},

                // nose face (blue)
                {{-.5f, -.5f, 0.5f}, {.1f, .1f, .8f}},
                {{.5f, .5f, 0.5f}, {.1f, .1f, .8f}},
                {{-.5f, .5f, 0.5f}, {.1f, .1f, .8f}},
                {{-.5f, -.5f, 0.5f}, {.1f, .1f, .8f}},
                {{.5f, -.5f, 0.5f}, {.1f, .1f, .8f}},
                {{.5f, .5f, 0.5f}, {.1f, .1f, .8f}},

                // tail face (green)
                {{-.5f, -.5f, -0.5f}, {.1f, .8f, .1f}},
                {{.5f, .5f, -0.5f}, {.1f, .8f, .1f}},
                {{-.5f, .5f, -0.5f}, {.1f, .8f, .1f}},
                {{-.5f, -.5f, -0.5f}, {.1f, .8f, .1f}},
                {{.5f, -.5f, -0.5f}, {.1f, .8f, .1f}},
                {{.5f, .5f, -0.5f}, {.1f, .8f, .1f}},

            };
            for (auto& v : vertices) {
                v.position += offset;
            }
            return std::make_unique<LveModel>(device, vertices);
        }
	};
}  // namespace lve