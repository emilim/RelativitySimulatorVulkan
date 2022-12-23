#include "first_app.hpp"
#include "simple_render_system.hpp"
#include "model.hpp"
#include "lve_camera.hpp"
#include "keyboard_controller.hpp"

//libs
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

//text library for vulkan


// std
#include <array>
#include <stdexcept>
#include <math.h>
#include <iostream>
#include <chrono>

namespace lve {
    class PhysicsSystem {
    public:
        PhysicsSystem(float gravity, float scale, LveDevice& device, GLFWwindow* window, std::vector<LveGameObject>& objects) : strengthGravity{ gravity }, unitScale{ scale }, lveDevice{ device }, lveWindow{ window }, gameObjects2{objects}
        {
            lve::LveGameObject centerOfMassObj = LveGameObject::createGameObject();
            centerOfMassObj.color = { 1.f, 1.f, 1.f };
            centerOfMassObj.model = Model::createCrossModel(lveDevice, centerOfMassObj.color);
            centerOfMassObj.transform.scale = glm::vec3{ 0.02f };
            centerOfMassObj.transform.translation.x = -1.f;
            objects.push_back(std::move(centerOfMassObj));    
        }
        const float strengthGravity;
        const float unitScale;
		LveDevice& lveDevice;
		GLFWwindow* lveWindow;
		std::vector<LveGameObject>& gameObjects2;


        void update(std::vector<LveGameObject>& gameObjects, float dt, unsigned int substeps = 1) {
            const float stepDelta = dt / substeps;
            for (int i = 0; i < substeps; i++) {
                stepSimulation(gameObjects, stepDelta);
            }
        }

        glm::vec3 computeForce(LveGameObject& obj1, LveGameObject& obj2, std::vector<LveGameObject>& gameObjects, std::vector<LveGameObject>::iterator& iterA, std::vector<LveGameObject>::iterator& iterB) const {
            auto distance = (obj1.transform.translation - obj2.transform.translation) * unitScale;
            float distanceSquared = glm::dot(distance, distance);
			float min = (glm::sqrt(glm::pow(obj1.transform.scale.x, 2) + glm::pow(obj1.transform.scale.y, 2))) * unitScale;
			float dis2 = obj1.transform.scale.x + obj2.transform.scale.x;
            float minDis = glm::dot(min, min);
            //std::cout << distanceSquared << " Obj: " << obj1.color.r << obj1.color.g << obj1.color.b << " obj2: " << obj2.color.r << obj2.color.g << obj2.color.b << std::endl;
            //if (glm::sqrt(distanceSquared) < glm::sqrt(minDis)) {
			if (glm::sqrt(distanceSquared) < dis2 * unitScale) {
                float masses = obj1.rigidBody.mass + obj2.rigidBody.mass;
                glm::vec3 finalVelocity = (obj1.rigidBody.velocity * obj1.rigidBody.mass + obj2.rigidBody.velocity * obj2.rigidBody.mass) / masses;

				obj1.transform.translation = obj1.rigidBody.mass > obj2.rigidBody.mass ? obj1.transform.translation : obj2.transform.translation;
                obj1.rigidBody.velocity = finalVelocity;
                obj1.rigidBody.mass = masses;
				obj1.color = glm::mix(obj1.color, obj2.color, obj2.rigidBody.mass / masses);
				//obj1.color = glm::vec3((obj1.color.r + obj2.color.r)/2.0f, (obj1.color.g + obj2.color.g)/2.0f, (obj1.color.b + obj2.color.b)/2.0f);
                obj1.transform.scale = glm::vec3(glm::sqrt(glm::pow(obj1.transform.scale.x, 2) + glm::pow(obj2.transform.scale.x, 2)));
                gameObjects.erase(iterB);
				iterB = iterA;
				
				return glm::vec3(0.0f, 0.0f, 0.0f);
            }
            float force = strengthGravity * obj1.rigidBody.mass * obj2.rigidBody.mass / distanceSquared;
            return force * distance / glm::sqrt(distanceSquared);
        }

    private:
        glm::vec3 centerOfMass{};
        glm::vec3 centerOfMassVelocity{};
        float totalMassStar{};
        float totalMass{};
        void stepSimulation(std::vector<LveGameObject>& gameObjects, float dt) {
            
            for (auto iterA = gameObjects.begin(); iterA != gameObjects.end(); ++iterA) {
                auto& objA = *iterA;
					
                for (auto iterB = iterA; iterB != gameObjects.end(); ++iterB) {
                    if (iterA == iterB) continue;
                    auto& objB = *iterB;

                    auto force = computeForce(objA, objB, gameObjects, iterA, iterB);
                    //force = glm::vec3(0.0f);
                    if (force != glm::vec3(0.0f, 0.0f, 0.0f)) {
                        objA.rigidBody.velocity += dt * -force / objA.rigidBody.mass;
                        objB.rigidBody.velocity += dt * force / objB.rigidBody.mass;
                    }
                    else {
                        continue;
					}
				}
			}
            
            LveGameObject& relativeObj = gameObjects[0];
            centerOfMass = glm::vec3(0.0f);
            centerOfMassVelocity = glm::vec3(0.0f);
            totalMassStar = 0.0f;
            totalMass = 0.0f;
            for (auto& obj : gameObjects) {
				obj.transform.translation += dt * (obj.rigidBody.velocity / unitScale);
                
                if (obj.getId() != relativeObj.getId() || gameObjects.size() == 1) {
                    centerOfMass += obj.rigidBody.mass * obj.transform.translation;
                    centerOfMassVelocity += obj.rigidBody.mass * (relativeObj.rigidBody.velocity - obj.rigidBody.velocity);
                    totalMassStar += obj.rigidBody.mass;
                }
                totalMass += obj.rigidBody.mass;
            }
			centerOfMass /= totalMassStar;
            centerOfMassVelocity /= totalMassStar;
			gameObjects2[0].transform.translation = centerOfMass;
			
			
            for (auto& obj : gameObjects) {
                auto effectiveVelocity = ((totalMass - obj.rigidBody.mass) / totalMass) * centerOfMassVelocity; // km/s
                //std::cout << effectiveVelocity.x << std::endl;
                effectiveVelocity *= 1000; //m/s
                effectiveVelocity *= 10; //augment relativistic effect 
				

				//Object dilation
				//obj.transform.scale *= glm::sqrt(1.0f - glm::pow(glm::length(effectiveVelocity) / 299792458.0f, 2));
            }
        }
    };

	/*
    class Vec2FieldSystem {
    public:
        void update(const PhysicsSystem& physicsSystem, std::vector<LveGameObject>& physicsObjs, std::vector<LveGameObject>& vectorField) {
            // For each field line we caluclate the net graviation force for that point in space
            for (auto& vf : vectorField) {
                glm::vec2 direction{};
                for (auto& obj : physicsObjs) {
					direction += physicsSystem.computeForce(obj, vf, physicsObjs);
                }

                // This scales the length of the field line based on the log of the length
                // values were chosen just through trial and error based on what i liked the look
                // of and then the field line is rotated to point in the direction of the field
                vf.transform.scale.x =
                    0.005f + 0.045f * glm::clamp(glm::log(glm::length(direction) + 1) / 3.f * physicsSystem.unitScale, 0.f, 1.f);
                vf.transform.rotation = atan2(direction.y, direction.x);
            }
        }
    };
	*/
    


    FirstApp::FirstApp() {
        //unit = 384000000.0f + 6371000.0f + 1737000.0f;
        unit = 1.0f;
        loadGameObjects();
    }

    void FirstApp::loadGameObjects() {
		/*
        std::vector<LveModel::Vertex> sierpinskiVert{};
        sierpinski(sierpinskiVert, 2, { -0.5f, 0.5f }, { 0.5f, 0.5f }, { 0.0f, -0.5f });
        std::shared_ptr<LveModel> lveModel = std::make_unique<LveModel>(lveDevice, sierpinskiVert);
        */
        circleModel = Model::createCircleModel(lveDevice, 64);
        std::shared_ptr<LveModel> lveRectangle = Model::createRectangleModel(lveDevice, glm::vec3(1.0f, 0.f, 0.f));
		
        auto obj = LveGameObject::createGameObject();
        obj.transform.translation = { 0.5f, -1.0f, 0.f };
        obj.color = { 0.0f, 1.0f, 0.0f };
        obj.rigidBody.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
        obj.model = circleModel;
        obj.rigidBody.mass = 1.f;
        obj.transform.scale = glm::vec3{ 0.1f };
        physicsObjects.push_back(std::move(obj));
        
        auto obj1 = LveGameObject::createGameObject();
        obj1.transform.translation = { -1.0f, .0f, 0.f };
        obj1.color = { 1.0f, 0.0f, 0.0f };
        obj1.rigidBody.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
        obj1.model = circleModel;
        obj1.rigidBody.mass = 1.f;
        obj1.transform.scale = glm::vec3{ 0.1f };
        physicsObjects.push_back(std::move(obj1));

        auto obj2 = LveGameObject::createGameObject();
        obj2.transform.translation = { 1.0f, 0.0f, 0.f };
        obj2.color = { 0.0f, 0.0f, 1.0f };
        obj2.rigidBody.velocity = { 0.0f, 0.0f, 0.f };
        obj2.model = circleModel;
        obj2.rigidBody.mass = 1.f;
        obj2.transform.scale = glm::vec3{ .1f };
        physicsObjects.push_back(std::move(obj2));
        
		/*
        for (int i = 0; i < 100; i++) {
            //random number between -1 and 1
            float x = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
            float y = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
            float z = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
            float t = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;

            auto obj2 = LveGameObject::createGameObject();
            obj2.transform.translation = { x, y, 0.f };
            obj2.color = { x, y, z };
            obj2.rigidBody.velocity = { 0.05f, t/5.0f, 0.f };
            obj2.model = circleModel;
            obj2.rigidBody.mass = 0.00001f;
            obj2.transform.scale = glm::vec3{ .01f };
            physicsObjects.push_back(std::move(obj2));
        }
		
        
        auto moon = LveGameObject::createGameObject();
        moon.transform.translation = { 0.0f, -1.0f, 0.f };
        moon.color = { 0.2f, 0.2f, 0.2f };
        moon.rigidBody.velocity = { 1022.0f, 0.f, 0.f };
        moon.model = circleModel;
        moon.rigidBody.mass = 7.34767309e22;
		moon.transform.scale = glm::vec3{ 10 * 1737000.0f / unit };
        physicsObjects.push_back(std::move(moon));

        auto earth = LveGameObject::createGameObject();
        earth.transform.translation = { .0f, .0f, 0.f };
        earth.color = { 0.f, 0.f, 1.f };
        earth.rigidBody.velocity = { .0f, .0f, 0.f };
        earth.model = circleModel;
		earth.rigidBody.mass = 5.972e24;
		earth.transform.scale = glm::vec3{ 10 * 6371000.0f / unit };
        physicsObjects.push_back(std::move(earth));
		*/
        
        /*
        for (int i = 0; i < 100; i++) {
		    //random number between -1 and 1
		    float x = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
		    float y = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
		    float z = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
            float t = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
			
		    auto obj2 = LveGameObject::createGameObject();
		    obj2.transform.translation = { x, y, 0.f };
		    obj2.color = { x, y, z };
		    obj2.rigidBody.velocity = { z * 2000.0f, t * 1000.0f, 0.f };
		    obj2.model = circleModel;
		    obj2.rigidBody.mass = 7.34767309e22/100.0;
		    obj2.transform.scale = glm::vec3{ .01f };
		    physicsObjects.push_back(std::move(obj2));
		}
		
        
        auto triangle = LveGameObject::createGameObject();
        triangle.transform.scale = glm::vec2{ .05f };
        triangle.transform.translation = { -.7f, 0.0f };
        triangle.color = { 0.f, 1.f, 0.f };
        triangle.rigidBody.velocity = { .0f, .0f };
        triangle.rigidBody.mass = 1000000.0f;
        triangle.model = lveModel;
        physicsObjects.push_back(std::move(triangle));
        
        std::vector<glm::vec3> colors{};
        std::vector<glm::vec3> colors2{};
        int N = 2000;
		for (int i = 0; i < N; i++) {
			colors.push_back({ i / static_cast<float_t>(N), i / static_cast<float_t>(N), i / static_cast<float_t>(N) });
			colors2.push_back({ i / static_cast<float_t>(N), i / static_cast<float_t>(N), i / static_cast<float_t>(N) });
		}
        for (auto& color : colors) {
            color = glm::pow(color, glm::vec3{ 2.2f });
        }
        for (int i = 0; i < colors.size(); i++) {

            float_t s = static_cast<float_t>(2.0 / colors.size());
			
			auto rectangle = LveGameObject::createGameObject();
			rectangle.model = lveRectangle;
            rectangle.transform.translation.x = -1.0f + i * s;
			rectangle.transform.translation.y = -1.0f;
			rectangle.transform.scale = glm::vec2{ s, 0.3f };
			rectangle.color = colors[i];
            rectObjects.push_back(std::move(rectangle));

            auto rectangle2 = LveGameObject::createGameObject();
            rectangle2.model = lveRectangle;
            rectangle2.transform.translation.x = -1.0f +  i * s;
            rectangle2.transform.translation.y = -0.7f;
            rectangle2.transform.scale = glm::vec2{ s, 0.3f };
            rectangle2.color = colors2[i];
            rectObjects.push_back(std::move(rectangle2));
        }
		

        // create vector field
        
        int gridCount = 40;
        for (int i = 0; i < gridCount; i++) {
            for (int j = 0; j < gridCount; j++) {
                auto vf = LveGameObject::createGameObject();
                vf.transform.scale = glm::vec3(0.005f);
                vf.transform.translation = {
                    -1.0f + (i + 0.5f) * 2.0f / gridCount,
                    -1.0f + (j + 0.5f) * 2.0f / gridCount,
				    0.0f
				};
                vf.color = glm::vec3(1.0f);
                vf.model = lveRectangle;
                vectorField.push_back(std::move(vf));
            }
        }
		*/
    }
    FirstApp::~FirstApp() {}

    void FirstApp::run() {
        PhysicsSystem gravitySystem{ /* 6.6743e-11f 0.81f */ 1.0f, unit, lveDevice, lveWindow.getGLFWwindow(), gameObjects};
        //Vec2FieldSystem vecFieldSystem{};
        SimpleRenderSystem simpleRenderSystem{ lveDevice, lveRenderer.getSwapChainRenderPass() };
		LveCamera camera{};
        auto viewerObject = LveGameObject::createGameObject();
		viewerObject.transform.translation = { 0.0f, 0.0f, -3.0f };
        LveKeyboardController cameraController{};
        
        float speedUp = 0.005f;
        glm::vec3 target{};

		auto currentTime = std::chrono::high_resolution_clock::now();
        while (!lveWindow.shouldClose()) {
            glfwPollEvents();
            auto newTime = std::chrono::high_resolution_clock::now();
            float frameTime = std::chrono::duration<float, std::chrono::seconds::period>(newTime - currentTime).count();
            currentTime = newTime;

            cameraController.moveInPlaneXZ(lveWindow.getGLFWwindow(), frameTime, viewerObject);
			//camera.setViewTarget(viewerObject.transform.translation, gameObjects[0].transform.translation);
			
            camera.setViewYXZ(gameObjects[0].transform.translation + viewerObject.transform.translation, viewerObject.transform.rotation);

            float aspect = lveRenderer.getAspectRatio();
            camera.setPerspectiveProjection(glm::radians(45.f), aspect, 0.1f, 100.f);
            //camera.setOrthographicProjection(-aspect, aspect, -1.f, 1.f, 0.0f, 100.f);
			

            if (auto commandBuffer = lveRenderer.beginFrame()) {
                gravitySystem.update(physicsObjects, (1.f / 60) * speedUp, 5);
                //vecFieldSystem.update(gravitySystem, physicsObjects, vectorField);

                lveRenderer.beginSwapChainRenderPass(commandBuffer);
                simpleRenderSystem.renderGameObjects(commandBuffer, gameObjects, camera);
                simpleRenderSystem.renderGameObjects(commandBuffer, vectorField, camera);
                simpleRenderSystem.renderGameObjects(commandBuffer, physicsObjects, camera);
				
                lveRenderer.endSwapChainRenderPass(commandBuffer);
                lveRenderer.endFrame();
            }
        }

        vkDeviceWaitIdle(lveDevice.device());
    }
}  // namespace lve