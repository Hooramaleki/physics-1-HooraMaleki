/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"

//global variables
const unsigned int TARGET_FPS = 50; // Frames per second
float dt = 1.0f / TARGET_FPS;       // Delta time 
float time = 0;                     

// the projectile launch parameters
Vector2 launchPosition = { 200.0f, 0.0f }; //start position (Y is set in main/draw)
float launchSpeed = 100.0f;                //speed in pixels/second
float launchAngle = 30.0f;                 //angle in degrees


void update()
{
    dt = 1.0f / TARGET_FPS;
    time += dt;
}


void draw()
{
    BeginDrawing();

    //clear screen and set background color
    ClearBackground(DARKPURPLE);


    DrawText("Hoora Maleki 101579782", 10, float(GetScreenHeight() - 30), 20, BLACK);

    //time slider and the text
    GuiSliderBar(Rectangle{ 10, 15, 1000, 20 }, "", TextFormat("%.2f", time), &time, 0, 240);
    DrawText(TextFormat("T: %6.2f", time), GetScreenWidth() - 140, 10, 30, BLACK);

    //set initial launch position Y if not yet set
    if (launchPosition.y == 0.0f)
        launchPosition.y = GetScreenHeight() - 200.0f;

    //sliders for adjusting position, angle, and speed with texts

    DrawText("Launch Position (X):", 10, 60, 14, RAYWHITE);
    //GuiSliderBar(Rectangle{ x, y, width, height }, leftText, rightText, &value, minValue, maxValue);
    GuiSliderBar(Rectangle{ 10, 80, 300, 20 }, "", TextFormat("%.0f", launchPosition.x), &launchPosition.x, 0.0f, (float)GetScreenWidth());

    DrawText("Launch Position (Y):", 10, 110, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 10, 130, 300, 20 }, "", TextFormat("%.0f", launchPosition.y), &launchPosition.y, 0.0f, (float)GetScreenHeight());

    DrawText("Launch Angle (deg):", 10, 160, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 10, 180, 300, 20 }, "", TextFormat("%.1f", launchAngle), &launchAngle, -180.0f, 180.0f);

    DrawText("Launch Speed (px/s):", 10, 210, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 10, 230, 300, 20 }, "", TextFormat("%.0f", launchSpeed), &launchSpeed, -500.0f, 2000.0f);


    //compute initial velocity vector (angle converted drom degree to radians)
    Vector2 initialVelocity = { cosf(launchAngle * DEG2RAD) * launchSpeed,
                               -sinf(launchAngle * DEG2RAD) * launchSpeed };

    //draw velocity vector
    const float drawScale = 0.5f;
    Vector2 lineEnd = Vector2Scale(initialVelocity, drawScale);
    lineEnd = Vector2Add(launchPosition, lineEnd);
    DrawLineEx(launchPosition, lineEnd, 4.0f, RED);

    // Draw launch point as a pink circle
    DrawCircleV(launchPosition, 6.0f, PINK);

    // Draw arrowhead for the velocity vector
    {
        Vector2 dir = Vector2Subtract(lineEnd, launchPosition);
        float len = Vector2Length(dir);
        if (len > 0.0f)
        {
            Vector2 nd = Vector2Scale(dir, 1.0f / len); // Normalize vector
            float ahSize = 12.0f; // Arrowhead size
            Vector2 left = Vector2Add(lineEnd, Vector2Scale(Vector2Rotate(nd, 150.0f * DEG2RAD), ahSize));
            Vector2 right = Vector2Add(lineEnd, Vector2Scale(Vector2Rotate(nd, -150.0f * DEG2RAD), ahSize));
            DrawLineEx(lineEnd, left, 3.0f, RED);
            DrawLineEx(lineEnd, right, 3.0f, RED);
        }
    }

    EndDrawing();
}


int main()
{
    //create window with given width/height from game.h
    InitWindow(InitialWidth, InitialHeight, "GAME2005 Hoora Maleki 101579782");
    SetTargetFPS(TARGET_FPS);

    //initialize launch position relative to screen size
    launchPosition = { 200.0f, GetScreenHeight() - 200.0f };

    //main game loop
    while (!WindowShouldClose()) //runs until user closes the window
    {
        update(); //update world state
        draw();   //render to screen
    }

    //cleanup and exit
    CloseWindow();
    return 0;
}