#include <raylib.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip> 

//Part 1
class Simulation {
public:
    Simulation(float gravity, float speed, float mass, Vector2 launchPos, float angleDeg, float scale)
    {
        this->gravity = gravity;
        this->speed = speed;
        this->mass = mass;
        this->launchPos = launchPos;
        this->scale = scale;

        angle = angleDeg * DEG2RAD; //degree to radians

        velocity.x = speed * cosf(angle);
        velocity.y = speed * sinf(angle);

        position = { 0.0f, 0.0f };
        totalTime = 0.0f;

        done = false;
    }

    void Update(float dt)
    {
        if (done) return;

        totalTime += dt;

        //update position
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        //apply gravity
        velocity.y -= gravity * dt;

        //stop when hitting the ground (after launching)
        if (position.y <= 0.0f && totalTime > 0.1f) {
            position.y = 0.0f;
            done = true;
        }
    }

    void Draw()
    {
        Vector2 drawPos = {
            launchPos.x + position.x * scale,
            (float)GetScreenHeight() - (launchPos.y + position.y) * scale
        };

        DrawCircleV(drawPos, 6, RED);
    }

    bool IsDone() const { return done; }
    float GetRange() const { return position.x; }
    float GetTime() const { return totalTime; }

private:
    float gravity;
    float speed;
    float mass;
    float angle;
    float scale;
    Vector2 launchPos;
    Vector2 position;
    Vector2 velocity;
    float totalTime;
    bool done;
};

int main()
{
    //SETTINGS
    
    //***********************************************
    //          CHANGE TO FALSE OR TRUE
    
    // true = generate CSV (Part 2), false = visual simulation (Part 1)
    
    bool runTestSuite = false; 
    
	//***********************************************


    float gravity = 1.6f;
    float speed = 120.0f;
    float mass = 10.0f;
    float scale = 0.1f;

    if (runTestSuite) {
        //part 2 - Generate CSV using analytic formulas
        std::vector<float> angles = {
            0, 3.19, 6.42, 9.736, 13.194, 16.874,
            20.905, 25.529, 31.367, 45, 58.633, 64.471,
            69.095, 73.126, 76.806, 80.264, 83.58, 86.81, 90
        };

        std::ofstream outFile("experimental_results.csv");
        if (!outFile) {
            std::cerr << "Error opening file!\n";
            return 1;
        }

        outFile << "Angle (deg),Range (m),Time in air (s)\n";
        outFile << std::fixed << std::setprecision(3);

        for (float angleDeg : angles) {
            float angleRad = angleDeg * DEG2RAD;

            //analytic formulas (no air resistance, launch from y = 0)
            float timeInAir = (2 * speed * sinf(angleRad)) / gravity;
            float range = speed * cosf(angleRad) * timeInAir;

            outFile << angleDeg << "," << range << "," << timeInAir << "\n";
        }

        outFile.close();
        std::cout << "Experimental results saved to experimental_results.csv\n";
    }

    
    else {
		//part1 - visual simulation
        const int screenWidth = 1000;
        const int screenHeight = 600;
        InitWindow(screenWidth, screenHeight, "Moon Launch Launch simulation");
        SetTargetFPS(60);

        float angle = 45.0f;   //degrees
        Vector2 launchPos = { 50, 500 }; //visual launch point

        Simulation sim(gravity, speed, mass, launchPos, angle, scale);

        while (!WindowShouldClose())
        {
            float dt = GetFrameTime();
            sim.Update(dt);

            BeginDrawing();
            ClearBackground(BLACK);

            //draw ground
            DrawLine(0, screenHeight - (int)(launchPos.y * scale), screenWidth,
                screenHeight - (int)(launchPos.y * scale), DARKGREEN);

            //draw projectile
            sim.Draw();

            //info
            DrawText(TextFormat("Angle: %.1f deg", angle), 20, 20, 20, RAYWHITE);
            DrawText(TextFormat("Time: %.2f s", sim.GetTime()), 20, 50, 20, RAYWHITE);
            DrawText(TextFormat("Range: %.2f m", sim.GetRange()), 20, 80, 20, RAYWHITE);

            //but it takes 106 seconds
            if (sim.IsDone())
                DrawText("Package has landed!", 20, 110, 20, GREEN);

            EndDrawing();
        }

        CloseWindow();
    }

    return 0;
}
