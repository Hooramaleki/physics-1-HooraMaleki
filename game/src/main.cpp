//Lab 3

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"
#include <stdbool.h>
#include <string>
#include <vector>

float dt = 1.0f / 60; //fixed timestep
float time = 0.0f;  

//base physics object
class physicsObject
{
public:
    Vector2 position = { 0,0 };
    Vector2 velocity = { 0,0 };
    float mass = 1;  //kg
    std::string name = "object";
    Color color = GREEN;

    virtual void draw() { DrawText(name.c_str(), position.x, position.y, 10, LIGHTGRAY); } //draw label
};


//circle object
class physicsObjectCircle : public physicsObject
{
public:
    float radius = 15;

    void draw() override
    {
        DrawCircle(position.x, position.y, radius, color);
        DrawText(name.c_str(), position.x, position.y, 10, LIGHTGRAY);
    }
};

//world to manage physics objects
class physicsWorld
{
private:
    unsigned int objecetCount = 0;

public:
    std::vector<physicsObject*> objects;
    Vector2 accelerationGravity = { 0, 9 }; //gravity

    void add(physicsObject* newObject)
    {
        newObject->name = std::to_string(objecetCount);
        objects.push_back(newObject);
        objecetCount++;
    }

    void update()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            objects[i]->position = objects[i]->position + objects[i]->velocity * dt; //move
            objects[i]->velocity = objects[i]->velocity + accelerationGravity * dt;   //apply gravity
        }
    }

    void checkCollision()
    {
        for (int i = 0; i < objects.size(); i++)
            objects[i]->color = GREEN; //reset color

        //check circle-circle collision
        for (int i = 0; i < objects.size(); i++)
        {
            for (int j = i + 1; j < objects.size(); j++)
            {
                physicsObjectCircle* circleA = (physicsObjectCircle*)objects[i];
                physicsObjectCircle* circleB = (physicsObjectCircle*)objects[j];

                float sumRadius = circleA->radius + circleB->radius;
                Vector2 displacement = circleA->position - circleB->position;
                float distance = Vector2Length(displacement);

                if (distance < sumRadius) //overlap
                {
                    circleA->color = RED;
                    circleB->color = RED;
                }
            }
        }
    }
};


float speed = 100;
float angle = 0;
float spawnX = 100;
float spawnY = 100;

physicsWorld world;

//remove objects offscreen
void cleanup()
{
    for (int i = 0; i < world.objects.size(); i++)
    {
        if (world.objects[i]->position.y > GetScreenHeight() ||
            world.objects[i]->position.y < 0 ||
            world.objects[i]->position.x > GetScreenWidth() ||
            world.objects[i]->position.x < 0)
        {
            delete world.objects[i];             //free memory
            world.objects.erase(world.objects.begin() + i);
            i--;
        }
    }
}

//update world and spawn
void update()
{
    dt = 1.0f / 60;
    time += dt;

    cleanup();
    world.update();

    if (IsKeyPressed(KEY_SPACE)) //spawn circle
    {
        physicsObjectCircle* newBird = new physicsObjectCircle();
        newBird->position = { spawnX, (float)GetScreenHeight() - spawnY };
        newBird->velocity = { speed * (float)cos(angle * DEG2RAD),
                              -speed * (float)sin(angle * DEG2RAD) };
        newBird->radius = 15;
        newBird->color = GREEN;

        world.add(newBird);
    }

    //sample angles
    if (IsKeyPressed(KEY_ONE))   angle = 0.0f;
    if (IsKeyPressed(KEY_TWO))   angle = 45.0f;
    if (IsKeyPressed(KEY_THREE)) angle = 60.0f;
    if (IsKeyPressed(KEY_FOUR))  angle = 90.0f;


    world.checkCollision(); //update collisions
}

//draw everything
void draw()
{
    BeginDrawing();
    ClearBackground(DARKPURPLE);
    DrawText("Hoora Maleki 101579782 - physics Lab 3", 10, 10, 18, BLACK);

    //spawn, speed, angle, gravity adjustment sliders
    DrawText("Spawn X:", 10, 40, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 40, 300, 20 }, "", TextFormat("%.0f", spawnX), &spawnX, 0.0f, (float)GetScreenWidth());

    DrawText("Spawn Y:", 10, 100, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 100, 300, 20 }, "", TextFormat("%.0f", spawnY), &spawnY, 0.0f, (float)GetScreenHeight());

    DrawText("Speed:", 10, 160, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 160, 300, 20 }, "", TextFormat("%.0f", speed), &speed, 0.0f, (float)GetScreenWidth());

    DrawText("Angle:", 10, 220, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 220, 300, 20 }, "", TextFormat("%.0f°", angle), &angle, 0.0f, 180.0f);

    DrawText("Gravity (Y):", 10, 280, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 280, 300, 20 }, "", TextFormat("%.1f", world.accelerationGravity.y), &world.accelerationGravity.y, -180.0f, 180.0f);

    DrawText("*** SPACE = launch *** press 1 for 0 degrees, 2 for 45, 3 for 60, 4 for 90", 10, 370, 14, RAYWHITE);

    //draw launch line
    Vector2 startPos = { spawnX, GetScreenHeight() - spawnY };
    Vector2 velocity = { speed * cos(angle * DEG2RAD), -speed * sin(angle * DEG2RAD) };
    DrawLineEx(startPos, startPos + velocity, 3, RED);

    //draw all objects
    for (int i = 0; i < world.objects.size(); i++)
        world.objects[i]->draw();

    EndDrawing();
}

//main loop
int main()
{
    InitWindow(InitialWidth, InitialHeight, "Physics Lab");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}

/*
//holds body data: position, velocity, drag (not used), mass (not used), active flag, and color
typedef struct PhysicsBody
{
    Vector2 position;
    Vector2 velocity;
    float drag; // stored (not applied in this lab)
    float mass; // stored (not used here)
    bool active;
    Color color;
}
PhysicsBody;


//holds deltaTime, time, and gravity (a Vector2), which apply to all bodies
typedef struct PhysicsSimulation
{
    float deltaTime;
    float time;
    Vector2 gravity; //gravity direction (on x,y) & magnitude (px/s^2)
}
PhysicsSimulation;


//stores multiple projectiles so we can spawn several simultaneously
//use FindFreeBodySlot() to locate an inactive slot for each new spawn.
PhysicsBody bodies[MAX_BODIES];
PhysicsSimulation sim;

Vector2 launchPosition = { 200.0f, 0.0f };
float launchSpeed = 400.0f;   // px/s (tweakable)
float launchAngle = 30.0f;    // degrees


//find a free body slot
int FindFreeBodySlot()
{
    //returns the index of the first body whose active flag is false (or -1 if full)
    for (int i = 0; i < MAX_BODIES; ++i) if (!bodies[i].active) return i; 
    return -1;
}


//spawn a new body at launchPosition with initial velocity from angle & speed
void SpawnBody(Vector2 pos, float angleDeg, float speed) 
{
    int idx = FindFreeBodySlot();
    if (idx < 0) return; // no free slot
    PhysicsBody* b = &bodies[idx];
    b->position = pos;
    float rad = angleDeg * DEG2RAD;
    b->velocity = (Vector2{ cosf(rad) * speed, -sinf(rad) * speed });
    b->drag = 0.0f;    //store drag (not used yet)
    b->mass = 1.0f;    //default mass (not used yet)
    b->active = true;
    b->color = RED;
}


//simulation update: applies gravity * deltaTime to velocity, integrates position
void SimulationUpdate(PhysicsSimulation* s, PhysicsBody* bodyArray, int count) 
{
    //use s->deltaTime already set by caller
    for (int i = 0; i < count; ++i)
    {
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



void update()
{
    //get real delta time each frame
    sim.deltaTime = GetFrameTime();
    sim.time += sim.deltaTime;

    //quick-angle presets for demonstration
    if (IsKeyPressed(KEY_ONE))  launchAngle = 0.0f;
    if (IsKeyPressed(KEY_TWO))  launchAngle = 45.0f;
    if (IsKeyPressed(KEY_THREE))launchAngle = 60.0f;
    if (IsKeyPressed(KEY_FOUR)) launchAngle = 90.0f;

    //press SPACE to spawn
    if (IsKeyPressed(KEY_SPACE)) {
        SpawnBody(launchPosition, launchAngle, launchSpeed);
    }

    //update all active bodies with simulation
    SimulationUpdate(&sim, bodies, MAX_BODIES);
}

void draw()
{
    BeginDrawing();
    ClearBackground(DARKPURPLE);

    DrawText("Hoora Maleki 101579782 - Projectile Lab", 10, 10, 18, BLACK);

    //show time
    DrawText(TextFormat("Sim time: %.2f s", sim.time), GetScreenWidth() - 220, 10, 18, BLACK);

    //sliders: launchPosition, angle, speed, gravity
    DrawText("Launch Position (X):", 10, 50, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 70, 300, 20 }, "", TextFormat("%.0f", launchPosition.x), & launchPosition.x, 0.0f, (float)GetScreenWidth());

    DrawText("Launch Position (Y):", 10, 100, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 120, 300, 20 }, "", TextFormat("%.0f", launchPosition.y), & launchPosition.y, 0.0f, (float)GetScreenHeight());


    DrawText("Launch Angle (deg):", 10, 150, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 170, 300, 20 }, "", TextFormat("%.1f", launchAngle), & launchAngle, -180.0f, 180.0f);


    DrawText("Launch Speed (px/s):", 10, 200, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 220, 300, 20 }, "", TextFormat("%.0f", launchSpeed), & launchSpeed, 0.0f, 2000.0f);

    
    DrawText("Gravity X:", 10, 260, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 280, 300, 20 }, "", TextFormat("%.1f", sim.gravity.x), & sim.gravity.x, -2000.0f, 2000.0f);

    DrawText("Gravity Y:", 10, 310, 14, RAYWHITE);
    GuiSliderBar(Rectangle { 10, 330, 300, 20 }, "", TextFormat("%.1f", sim.gravity.y), & sim.gravity.y, -2000.0f, 2000.0f);


    DrawText("*** SPACE = launch *** 1/2/3/4 = angles 0/45/60/90 ***", 10, 370, 14, RAYWHITE);

    // Draw all active bodies (projectiles)
    for (int i = 0; i < MAX_BODIES; ++i) 
    {
        if (!bodies[i].active) continue;
        // draw the projectile
        DrawCircleV(bodies[i].position, 8.0f, bodies[i].color);
    }

    //draw launch direction vector from launchPosition (visual aid)
    Vector2 initVel = { cosf(launchAngle * DEG2RAD) * launchSpeed, -sinf(launchAngle * DEG2RAD) * launchSpeed };
    Vector2 lineEnd = Vector2Add(launchPosition, Vector2Scale(initVel, 0.5f));
    DrawLineEx(launchPosition, lineEnd, 4.0f, RED);
    DrawCircleV(launchPosition, 6.0f, PINK);

    EndDrawing();
}

int main()
{
    InitWindow(InitialWidth, InitialHeight, "Projectile Simulation - Physics Lab");
    SetTargetFPS(60);

    //initialize simulation
    sim.deltaTime = 0.0f;
    sim.time = 0.0f;
    sim.gravity = (Vector2{ 0.0f, 980.0f }); // default gravity ~ 980 px/s^2 (pixels ~ cm analogy)

    //initialize launch position
    launchPosition = (Vector2{ 200.0f, GetScreenHeight() - 200.0f });

    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}
*/

