#include <raylib.h>
#include <cmath>


//part 1 
class Simulation {
public:
    Simulation(float gravity, float speed, float mass, Vector2 launchPos, float angleDeg, float scale)
    {
        this->gravity = gravity;
        this->speed = speed;
        this->mass = mass;
        this->launchPos = launchPos;
        this->scale = scale;

        angle = angleDeg * DEG2RAD; //convert to radians

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
    const int screenWidth = 1000;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Moon Launch Launch simulation");
    SetTargetFPS(60);

	//parameters
    float gravity = 1.6f;          
    float speed = 120.0f;          
    float mass = 10.0f;            
    float angle = 45.0f;   //degrees
    float scale = 0.1f; 

    Vector2 launchPos = { 50, 500 }; //launch point (placed there just for visuals)

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


        DrawText("It will land at 9000m after 106 seconds", 20, 110, 20, GREEN);

		//but it takes so long
        if (sim.IsDone())
            DrawText("Package has landed!", 20, 110, 20, GREEN);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
