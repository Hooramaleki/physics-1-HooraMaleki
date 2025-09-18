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
Vector2 startOfLine = { 100, 100 };
Vector2 endOfLine = { 700, 100 };

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

	ClearBackground(PURPLE);
	DrawText("Hoora Maleki 101579782", 10, float(GetScreenHeight() - 30), 20, BLACK);


	GuiSliderBar(Rectangle{ 10, 15, 1000, 20 }, "", TextFormat("%.2f", time), &time, 0, 240);
	DrawText(TextFormat("T: %6.2f", time), GetScreenWidth() - 140, 10, 30, BLACK);

	DrawCircle(x, y, 70, PINK);
	DrawCircle(500 + cos(time * frequency) * amplitude, 500 + sin(time * frequency) * amplitude, 70, DARKPURPLE);

	DrawLineEx(startOfLine, endOfLine, 10.0f, BLACK);

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