#include "planar_quadrotor_visualizer.h"
#include <cmath>


PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualization
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate propellers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    int scale = 50;
    int o_x = q_x * scale + SCREEN_WIDTH / 2;
    int o_y = -q_y * scale + SCREEN_HEIGHT / 2;

    //================

    // Define rect
    int rect_width = 80;
    int rect_height = 15;

    int half_width = rect_width / 2;
    int half_height = rect_height / 2;

    SDL_Point vertex[4];
    vertex[0].x = int(o_x + (-half_width * cos(q_theta) + half_height * sin(q_theta))),
    vertex[0].y = int(o_y + (-half_width * sin(q_theta) - half_height * cos(q_theta)));

    vertex[1].x = int(o_x + (half_width * cos(q_theta) + half_height * sin(q_theta))),
    vertex[1].y = int(o_y + (half_width * sin(q_theta) - half_height * cos(q_theta)));

    vertex[2].x = int(o_x + (half_width * cos(q_theta) - half_height * sin(q_theta))),
    vertex[2].y = int(o_y + (half_width * sin(q_theta) + half_height * cos(q_theta)));

    vertex[3].x = int(o_x + (-half_width * cos(q_theta) - half_height * sin(q_theta))),
    vertex[3].y = int(o_y + (-half_width * sin(q_theta) + half_height * cos(q_theta)));


    Sint16 vx[4] = { vertex[0].x, vertex[1].x, vertex[2].x, vertex[3].x };
    Sint16 vy[4] = { vertex[0].y, vertex[1].y, vertex[2].y, vertex[3].y };
    filledPolygonRGBA(gRenderer.get(), vx, vy, 4, 0x80, 0x00, 0x00, 0xFF);



    // Update propeller animation angle
    static float propellerAngle = 0;
    propellerAngle += 0.03;

    // Propeller properties
    int minPropellerWidth = 5;
    int maxPropellerWidth = 20;
    int propellerHeight = 3;

    // Calculate new propeller width over time
    int propellerWidth = minPropellerWidth + maxPropellerWidth * std::abs(std::sin(propellerAngle * 0.5));

    // prawe smiglo
    SDL_RenderDrawLine(gRenderer.get(), vertex[1].x, vertex[1].y - 20, vertex[1].x, vertex[1].y);
    filledEllipseRGBA(gRenderer.get(), vertex[1].x, vertex[1].y - 20, propellerWidth, propellerHeight, 0x00, 0x00, 0x00, 0xFF);

    // lewe smiglo
    SDL_RenderDrawLine(gRenderer.get(), vertex[0].x, vertex[0].y - 20, vertex[0].x, vertex[0].y);
    filledEllipseRGBA(gRenderer.get(), vertex[0].x, vertex[0].y - 20, propellerWidth, propellerHeight, 0x00, 0x00, 0x00, 0xFF);
}
