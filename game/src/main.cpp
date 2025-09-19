/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"

const unsigned int TARGET_FPS = 50; //frames/second
float dt = 1.0f / TARGET_FPS; //seconds/frame
float time = 0;
float x = 500;
float y = 500;
float frequency = 1;
float amplitude = 100;
float speed = 100;
float angle = 30;


//Changes world state
void update()
{
	dt = 1.0f / TARGET_FPS;
	time += dt;

	x = x + (-sin(time * frequency)) * frequency * amplitude * dt;
	y = y + (cos(time * frequency)) * frequency * amplitude * dt;
}

//Display world state
void draw()
{
	BeginDrawing();


	ClearBackground(DARKPURPLE);
	DrawText("Hoora Maleki 101579782", 10, float(GetScreenHeight() - 30), 20, BLACK);


	GuiSliderBar(Rectangle{ 10, 15, 1000, 20 }, "", TextFormat("%.2f", time), &time, 0, 240);
	DrawText(TextFormat("T: %6.2f", time), GetScreenWidth() - 140, 10, 30, BLACK);


	//slider for speed and angle
	GuiSliderBar(Rectangle{ 10, 100, 200, 100 }, "", TextFormat("%.0f", speed), &speed, -100, 1000);
	GuiSliderBar(Rectangle{ 10, 200, 200, 100 }, "", TextFormat("%.0f", angle), &angle, -180, 180);


	//DrawCircle(x, y, 70, PINK);
	//DrawCircle(500 + cos(time * frequency) * amplitude, 500 + sin(time * frequency) * amplitude, 70, DARKPURPLE);


	//drawing the line
	Vector2 startPOS = { 200, GetScreenHeight() - 200 };
	Vector2 velocity = { cos(angle * DEG2RAD) * speed, -sin(angle * DEG2RAD) * speed};

	DrawLineEx(startPOS, startPOS + velocity, 10, BLACK);

	EndDrawing();

}

int main()
{
	InitWindow(InitialWidth, InitialHeight, "GAME2005 Hoora Maleki 101579782");
	SetTargetFPS(TARGET_FPS);

	while (!WindowShouldClose()) // Loops TARGET_FPS times per second
	{
		update();
		draw();
	}

	CloseWindow();
	return 0;
}