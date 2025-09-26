/* Angry-bird-style projectile simulation lab
   Uses raylib + raymath + raygui (like your original).
   Implements:
   - PhysicsBody struct (position, velocity, drag, mass, active)
   - PhysicsSimulation struct (deltaTime, time, gravity)
   - Simulation.UpdateAll(bodies) applying gravity each frame
   - Adjustable gravity (x,y) and launch parameters via sliders
   - SPACE launches a new body from launchPosition (repeatable)
   - Keys 1/2/3/4 quickly set angles to 0/45/60/90 degrees for demonstration
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"

#include <stdbool.h>

#define MAX_BODIES 64

typedef struct PhysicsBody {
    Vector2 position;
    Vector2 velocity;
    float drag; // stored (not applied in this lab)
    float mass; // stored (not used here)
    bool active;
    Color color;
} PhysicsBody;

typedef struct PhysicsSimulation {
    float deltaTime;
    float time;
    Vector2 gravity; // gravity direction & magnitude (px/s^2)
} PhysicsSimulation;

// Globals (kept simple like your original)
PhysicsBody bodies[MAX_BODIES];
PhysicsSimulation sim;

Vector2 launchPosition = { 200.0f, 0.0f };
float launchSpeed = 400.0f;   // px/s (tweakable)
float launchAngle = 30.0f;    // degrees
float trailPointSize = 2.0f;  // optional visual tweak

// helper: find a free body slot
int FindFreeBodySlot(void) {
    for (int i = 0; i < MAX_BODIES; ++i) if (!bodies[i].active) return i;
    return -1;
}

// spawn a new body at launchPosition with initial velocity from angle & speed
void SpawnBody(Vector2 pos, float angleDeg, float speed) {
    int idx = FindFreeBodySlot();
    if (idx < 0) return; // no free slot
    PhysicsBody* b = &bodies[idx];
    b->position = pos;
    float rad = angleDeg * DEG2RAD;
    // Note: screen Y increases downwards, so upward velocity is negative Y
    b->velocity = (Vector2{ cosf(rad) * speed, -sinf(rad) * speed });
    b->drag = 0.0f;    // store drag (not applied)
    b->mass = 1.0f;    // default mass
    b->active = true;
    b->color = RED;
}

// Simulation update: applies gravity * deltaTime to velocity, integrates position
void SimulationUpdate(PhysicsSimulation* s, PhysicsBody* bodyArray, int count) {
    // use s->deltaTime already set by caller
    for (int i = 0; i < count; ++i) {
        PhysicsBody* b = &bodyArray[i];
        if (!b->active) continue;

        // apply gravity to velocity (accel * dt)
        b->velocity.x += s->gravity.x * s->deltaTime;
        b->velocity.y += s->gravity.y * s->deltaTime;

        // integrate position
        b->position.x += b->velocity.x * s->deltaTime;
        b->position.y += b->velocity.y * s->deltaTime;
    }
}

void ClearAllBodies(void) {
    for (int i = 0; i < MAX_BODIES; ++i) bodies[i].active = false;
}

void update(void)
{
    // get real delta time each frame (more accurate than fixed-step)
    sim.deltaTime = GetFrameTime();
    sim.time += sim.deltaTime;

    // input: quick-angle presets for demonstration
    if (IsKeyPressed(KEY_ONE))  launchAngle = 0.0f;
    if (IsKeyPressed(KEY_TWO))  launchAngle = 45.0f;
    if (IsKeyPressed(KEY_THREE))launchAngle = 60.0f;
    if (IsKeyPressed(KEY_FOUR)) launchAngle = 90.0f;

    // press SPACE to launch (spawns a new body each time)
    if (IsKeyPressed(KEY_SPACE)) {
        SpawnBody(launchPosition, launchAngle, launchSpeed);
    }

    // update all active bodies with simulation
    SimulationUpdate(&sim, bodies, MAX_BODIES);
}

void draw(void)
{
    BeginDrawing();
    ClearBackground(DARKPURPLE);

    DrawText("Hoora Maleki 101579782 - Projectile Lab", 10, 10, 18, BLACK);

    // UI: show simulation time
    DrawText(TextFormat("Sim time: %.2f s", sim.time), GetScreenWidth() - 220, 10, 18, BLACK);

    // Sliders: launchPosition, angle, speed
    DrawText("Launch Position (X):", 10, 50, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 70, 300, 20 }, "", TextFormat("%.0f", launchPosition.x), & launchPosition.x, 0.0f, (float)GetScreenWidth());

    DrawText("Launch Position (Y):", 10, 100, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 120, 300, 20 }, "", TextFormat("%.0f", launchPosition.y), & launchPosition.y, 0.0f, (float)GetScreenHeight());

    DrawText("Launch Angle (deg):", 10, 150, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 170, 300, 20 }, "", TextFormat("%.1f", launchAngle), & launchAngle, -180.0f, 180.0f);

    DrawText("Launch Speed (px/s):", 10, 200, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 220, 300, 20 }, "", TextFormat("%.0f", launchSpeed), & launchSpeed, 0.0f, 2000.0f);

    // Gravity sliders (allow X & Y)
    DrawText("Gravity X:", 10, 260, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 280, 300, 20 }, "", TextFormat("%.1f", sim.gravity.x), & sim.gravity.x, -2000.0f, 2000.0f);

    DrawText("Gravity Y:", 10, 310, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 330, 300, 20 }, "", TextFormat("%.1f", sim.gravity.y), & sim.gravity.y, -2000.0f, 2000.0f);

    DrawText("*** SPACE = launch *** 1/2/3/4 = angles 0/45/60/90 ***", 10, 370, 14, RAYWHITE);

    // Draw all active bodies (projectiles)
    for (int i = 0; i < MAX_BODIES; ++i) {
        if (!bodies[i].active) continue;
        // draw the projectile
        DrawCircleV(bodies[i].position, 8.0f, bodies[i].color);
    }

    // Draw launch direction vector from launchPosition (visual aid)
    Vector2 initVel = { cosf(launchAngle * DEG2RAD) * launchSpeed, -sinf(launchAngle * DEG2RAD) * launchSpeed };
    Vector2 lineEnd = Vector2Add(launchPosition, Vector2Scale(initVel, 0.05f));
    DrawLineEx(launchPosition, lineEnd, 4.0f, RED);
    DrawCircleV(launchPosition, 6.0f, PINK);

    EndDrawing();
}

int main(void)
{
    InitWindow(InitialWidth, InitialHeight, "Projectile Simulation - Physics Lab");
    SetTargetFPS(60);

    // initialize simulation
    sim.deltaTime = 0.0f;
    sim.time = 0.0f;
    sim.gravity = (Vector2{ 0.0f, 980.0f }); // default gravity ~ 980 px/s^2 (pixels ~ cm analogy)

    // initialize launch position
    launchPosition = (Vector2{ 200.0f, GetScreenHeight() - 200.0f });
    // initialize bodies array
    ClearAllBodies();

    // start with no active bodies
    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}
