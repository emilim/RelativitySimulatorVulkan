#pragma once

#include "lve_device.hpp"
#include "lve_game_object.hpp"
#include "lve_renderer.hpp"
#include "lve_window.hpp"

// std
#include <memory>
#include <vector>

namespace lve {
	class FirstApp {
	public:
		static constexpr int WIDTH = 900;
		static constexpr int HEIGHT = 900;

		FirstApp();
		~FirstApp();

		FirstApp(const FirstApp&) = delete;
		FirstApp& operator=(const FirstApp&) = delete;

		void run();

	private:
		void loadGameObjects();

		LveWindow lveWindow{ WIDTH, HEIGHT, "Vulkan Tutorial" };
		LveDevice lveDevice{ lveWindow };
		LveRenderer lveRenderer{ lveWindow, lveDevice };
		
		std::vector<LveGameObject> gameObjects;
		std::vector<LveGameObject> vectorField{};
		std::vector<LveGameObject> physicsObjects;
		float unit;
		std::shared_ptr<LveModel> circleModel;
	};
}  // namespace lve