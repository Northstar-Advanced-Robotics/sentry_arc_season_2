#include <SDL2/SDL.h>
#include <rclcpp/rclcpp.hpp>

#include <stdio.h>

// 600mm diammeter
// 191mm wheels diameter

#include <geometry_msgs/msg/twist.hpp>
#include <stop_token>
#include <thread>

class Keyboard : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
    std::jthread keyboard_thread;

    static void thread_work(std::stop_token st, Keyboard* node) {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            return;
        }

        SDL_Window* window = SDL_CreateWindow(
            "Keyboard Input window",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            800, 600, SDL_WINDOW_SHOWN
        );

        if(window == NULL) {
            printf("%s", SDL_GetError());
        }

        SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
        if (!renderer) {
            printf("Renderer error: %s\n", SDL_GetError());
        }
        
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);
        SDL_RenderPresent(renderer);       SDL_Event event;

        bool running = true;

        while(running && !st.stop_requested()) {
            while(SDL_PollEvent(&event)) {
                switch(event.type) {
                    default:
                    break;
                    case SDL_QUIT:
                        running = false;
                    break;
                }

            }

            const uint8_t* keys = SDL_GetKeyboardState(NULL);
            geometry_msgs::msg::Twist cmd_vel;
            if (keys[SDL_SCANCODE_W]) {
                cmd_vel.linear.x += 1;
            }
            if (keys[SDL_SCANCODE_S]) {
                cmd_vel.linear.x -= 1;
            }
            if (keys[SDL_SCANCODE_A]) {
                cmd_vel.linear.y += 1;
            }
            if (keys[SDL_SCANCODE_D]) {
                cmd_vel.linear.y -= 1;
            }

            const auto mag = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
            if(mag != 0) {                
                cmd_vel.linear.x /= mag;
                cmd_vel.linear.y /= mag;
            }
        
            node->cmd_vel->publish(cmd_vel);

            SDL_Delay(8);
        }

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
  
public:
    Keyboard() : Node("keyboard_node") {
        cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS().reliable());
        keyboard_thread = std::jthread(thread_work, this);
    }

    ~Keyboard() {
        keyboard_thread.request_stop();
    }
};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Keyboard>());
    rclcpp::shutdown();
}
