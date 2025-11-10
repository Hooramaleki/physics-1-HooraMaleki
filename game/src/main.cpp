//Lab 5

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


//new shape type for HalfSpace collision object
enum physicsShapes
{
    circle,
	half_Space
};


//base physics object
class physicsObject
{
public:
	bool isStatic = false;
    Vector2 position = { 0,0 };
    Vector2 velocity = { 0,0 };
    float mass = 1;  //kg
    std::string name = "object";
    Color color = GREEN;

    virtual void draw() { DrawText(name.c_str(), position.x, position.y, 10, LIGHTGRAY); } //draw label

    virtual physicsShapes Shape() = 0;  //abstract func. must be defined in child classes
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

    virtual physicsShapes Shape() override
    {
		return circle;
    }
};

//new physics object representing halfspace
//which is static and defined by a position point and a surface normal
class physicsHalfSpace : public physicsObject
{
private:

    float rotation = 0;
    Vector2 normal = { 0, -1 };


public:

	//rotation of the halfspace in degrees
    void setRotationDegrees(float rotationInDegrees)
    {
		rotation = rotationInDegrees;
		normal = Vector2Rotate({ 0, -1 }, rotation * DEG2RAD);
	}
    float getRotation()
    {
		return rotation;
    }

    Vector2 getNormal()
    {
        return normal;
	}

    void draw() override
    {
        DrawCircle(position.x, position.y, 8, color);

		DrawLineEx(position, position + normal * 30, 1, color);
    
		Vector2 parallelToSurface = Vector2Rotate(normal, PI * 0.5f);
		DrawLineEx(position - parallelToSurface * 4000, position + parallelToSurface * 4000, 1, color);
    }

    virtual physicsShapes Shape() override
    {
        return half_Space;
    }

};

//######
//detects overlap between a circle and a half-space
bool circleHalfSpaceOverlap(physicsObjectCircle* circle, physicsHalfSpace* halfSpace)
{

    Vector2 normal = halfSpace->getNormal();
    Vector2 pointOnPlane = halfSpace->position;

    //compute the signed distance from circle center to the plane
    Vector2 signedDistance = circle->position - pointOnPlane;
    //#####
    //distance from the circle center to the plane
	float dot = Vector2DotProduct(signedDistance, normal);
    //this is the vector pointing from the plane toward the circle center
	Vector2 ProjectionDisplacementOnToNormal = normal * dot;

    //draw debug line showing distance to the plane
    DrawLineEx(circle->position, pointOnPlane, 1, GRAY);
    DrawText(TextFormat("d: %.2f", signedDistance), circle->position.x + 20, circle->position.y, 12, GRAY);

    //####
    //check overlap
    //check how much circle crosses into the plane
    float overlap = circle->radius - dot;
	
    if (overlap > 0)
    {
        //i dont know if we still need them to turn red touching the half-space
        //circle->color = RED;
        
        //compute the Minimum Translation Vector (MTV)
		//tells us how much to move the circle out of the half-space
		//mtv = insertion depth
		Vector2 mtv = normal * overlap; 
		circle->position += mtv; //move circle out of half-space
        return true;
    }
    else
    {
        return false;
    }
    //return dot < circle->radius;
}



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
            // Skip static objects (e.g., halfspaces)
            if (objects[i]->isStatic)
                continue;

            // Apply motion and gravity
            objects[i]->position = objects[i]->position + objects[i]->velocity * dt;
            objects[i]->velocity = objects[i]->velocity + accelerationGravity * dt;
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
                physicsObject* objA = objects[i];
                physicsObject* objB = objects[j];

                //ask obj what shape they are
                physicsShapes shapeA = objA->Shape();
                physicsShapes shapeB = objB->Shape();

                //#########
                //check if both objects are circles
                //because my last code, the spawned circles when
				//collide with halfspace's circle, pushed it downwards
                if (shapeA == circle && shapeA == circle)
                {
                    //turn these physics objects into circle objects to access their data
                    physicsObjectCircle* circleA = (physicsObjectCircle*)objA;
                    physicsObjectCircle* circleB = (physicsObjectCircle*)objB;


                    Vector2 displacement = circleB->position - circleA->position;
                    float sumRadius = circleA->radius + circleB->radius;
                    float distance = Vector2Length(displacement);

                    //#######
					//calculate overlap
                    float overlap = sumRadius - distance;
                    
                    if (overlap > 0)
                    {
                        Vector2 normalAtoB = displacement / distance;
                        Vector2 mtv = normalAtoB * overlap;

                        //only move non-static objects (not planes)
                        if (!circleA->isStatic)
                            circleA->position -= mtv * 0.5f; //move A half the overlap distance
                        if (!circleB->isStatic)
                            circleB->position += mtv * 0.5f; //move B the other half
                    
                        circleA->color = RED;
                        circleB->color = RED;
                    }
                }
                //if one is circle and one is half space
                else if (shapeA == circle && shapeB == half_Space)
                {
                    circleHalfSpaceOverlap((physicsObjectCircle*)objA, (physicsHalfSpace*)objB);
                }
                else if (shapeA == half_Space && shapeB == circle)
                {
                    circleHalfSpaceOverlap((physicsObjectCircle*)objB, (physicsHalfSpace*)objA);
                }
				
                
            }
        }
    }
};


float speed = 100;
float angle = 0;
float spawnX = 100;
float spawnY = 300;

physicsWorld world;
physicsHalfSpace halfSpace;
physicsHalfSpace halfSpace2; //##### to represent a bowl shape
float haldspaceAngle = 0;

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
    DrawText("Hoora Maleki 101579782 - physics Lab 5", 10, 10, 18, BLACK);

    //spawn line
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

	//half space
    //GUI sliders for adjusting half-space position and rotation
    GuiSliderBar(Rectangle{ 120, 340, 250, 20 }, "halfSpace x", TextFormat("%.0f", halfSpace.position.x), &halfSpace.position.x, 0.0f, (float)GetScreenWidth());
    GuiSliderBar(Rectangle{ 120, 400, 250, 20 }, "halfSpace Y", TextFormat("%.0f", halfSpace.position.y), &halfSpace.position.y, 0.0f, (float)GetScreenWidth());

	float halfspaceRotation = halfSpace.getRotation();
    GuiSliderBar(Rectangle{ 120, 460, 250, 20 }, "Rotation", TextFormat("%.0f", halfSpace.getRotation()), &halfspaceRotation, 0.0f, (float)GetScreenWidth());
	halfSpace.setRotationDegrees(halfspaceRotation);

    DrawText("*** SPACE = launch *** press 1 for 0 degrees, 2 for 45, 3 for 60, 4 for 90", 10, 500, 14, RAYWHITE);

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

    //create and add a static halfspace object
	halfSpace.isStatic = true;
	halfSpace.position = { 600, 900 };
	halfSpace.setRotationDegrees(20);
	world.add(&halfSpace);
    //#####
	//added a new static halfspace to represent a bowl shape
    halfSpace2.isStatic = true;
	halfSpace2.position = { 900, 900 };
	halfSpace2.setRotationDegrees(-20);
	world.add(&halfSpace2);

    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}

