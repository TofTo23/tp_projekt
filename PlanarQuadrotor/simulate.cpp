/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <algorithm>
#include <matplot/matplot.h>
#include <thread>
#include <SDL.h>
#include <windows.h>
#include <mmsystem.h>
#include <mciapi.h>
#pragma comment(lib, "winmm.lib")

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    R.row(0) << 0.1, 0.05;
    R.row(1) << 0.05, 0.1;
    Q.diagonal() << 1, 1, 10, 1, 1, 10;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}


void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void Droga_lotu(const std::vector<float>& x_history, const std::vector<float>& y_history) {
    using namespace matplot;
    std::string tempOutputFile = "lot.png";
    plot(x_history, y_history);
    xlabel("X");
    ylabel("Y");
    title("Lot Quadrotora");
    show();
}

void play_audio(char plik[30]) {
    PlaySound(plik, GetModuleHandle(NULL), SND_FILENAME | SND_ASYNC | SND_LOOP);
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);

    /* Timestep for the simulation */
    const float dt = 0.005f;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    bool graj1 = false;
    bool graj2 = false;
    float prev_xdot = 0;
    float prev_ydot = 0;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            /*while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }

            }*/
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse clicked at: (" << x << ", " << y << ")" << std::endl;

                    float scale = 50.0f;
                    float q_x = (x - SCREEN_WIDTH / 2) / scale;
                    float q_y = -(y - SCREEN_HEIGHT / 2) / scale;

                    std::cout << "Goal position in quadrotor frame: (" << q_x << ", " << q_y << ")" << std::endl;

                    goal_state << q_x, q_y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                    std::cout << "Goal state set to: " << goal_state.transpose() << std::endl;

                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) {
                    std::thread(Droga_lotu, x_history, y_history).detach();
                }

            }
            SDL_Delay((int)(dt * 100));

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            x_history.push_back(quadrotor.GetState()[0]);
            y_history.push_back(quadrotor.GetState()[1]);

            if (sqrt(pow(quadrotor.GetState()[3], 2) + pow(quadrotor.GetState()[4], 2)) > sqrt(pow(prev_xdot, 2) + pow(prev_ydot, 2)) && graj1 == false) {
                play_audio("dzwiek1.wav");
                graj1 = true;
                graj2 = true;
            }
            else if (sqrt(pow(quadrotor.GetState()[3], 2) + pow(quadrotor.GetState()[4], 2)) <= sqrt(pow(prev_xdot, 2) + pow(prev_ydot, 2)) && graj2 == true) {
                play_audio("dzwiek2.wav");
                graj2 = false;
                graj1 = false;
            };
            prev_xdot = quadrotor.GetState()[3];
            prev_ydot = quadrotor.GetState()[4];
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
