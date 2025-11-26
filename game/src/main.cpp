//Lab 6

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
float coefficientOfFriction = 0.5f;


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
	Vector2 netForce = { 0,0 }; //N
    float grippinesss = 0.5f;
    std::string name = "object";
    Color color = GREEN;
    Color defaultColor = GREEN;

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

        //draw velocity vector in red
        float velocityDrawScale = 12.0f;
        Vector2 velocityEnd = { position.x + velocity.x * velocityDrawScale, position.y + velocity.y * velocityDrawScale };
        DrawLineEx(position, velocityEnd, 2, RED);
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


bool circleHalfSpaceOverlap(physicsObjectCircle* circle, physicsHalfSpace* halfSpace);

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

    //%%%%%%%%%%%%%%%
    //reset all forces on every object before calculating new forces, aach update, we start with netForce = 0
    void ResetNetForces()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            objects[i]->netForce = { 0,0 };
		}
	}

    //apply gravity to all non-static objects
    //gravity is treated as a constant force: F = m * g
    void addGravityForce()
    {
        for (int i = 0; i < objects.size(); i++)
        {
			physicsObject* obj = objects[i];

            if (objects[i]->isStatic)
                continue;

            //calculate gravityl force. Fgravity = mass * gravityAcceleration
			Vector2 Fgravity = accelerationGravity * objects[i]->mass;
            //add gravity to the object's total net force
            objects[i]->netForce += Fgravity;

            //draw the gravity force vector
            DrawLineEx(obj->position, obj->position + Fgravity, 2, PURPLE);
        }
	}

    //convert net force to acceleration using  a = F / m
    //update velocity using acceleration
    //update position using velocity
    void applyKinematics()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            physicsObject* obj = objects[i];

            if (obj->isStatic)
                continue;

            //move object with its current velocity
            obj->position = obj->position + obj->velocity * dt;

            //compute acceleration from the total net force
			Vector2 acceleration = obj->netForce / obj->mass; //F=m.a  -> a=F/m

            obj->velocity = obj->velocity + acceleration * dt;

			//DrawLineEx(obj->position, obj->position + obj->netForce, 2, PURPLE); //draw acceleration vector
            //%%%%%%%%%%%%%%%
        }

    }

    void update()
    {
		ResetNetForces(); //%%%%%%%%%%%%%%
		addGravityForce(); //%%%%%%%%%%%%%%
		checkCollision(); //%%%%%%%%%%%%%
		applyKinematics(); //%%%%%%%%%%%%% i moved the previous logic into apply kinematics
    }

    void checkCollision()
    {
        for (int i = 0; i < objects.size(); i++)
        {
            objects[i]->color = objects[i]->defaultColor; //reset the color
        }

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


float speed = 0;
float angle = 0;
float spawnX = 100;
float spawnY = 300;

physicsWorld world;
physicsHalfSpace halfSpace;
//physicsHalfSpace halfSpace2; //to represent a bowl shape
float haldspaceAngle = 0;



//detects overlap between a circle and a half-space
bool circleHalfSpaceOverlap(physicsObjectCircle* circle, physicsHalfSpace* halfSpace)
{

    Vector2 normal = halfSpace->getNormal();
    Vector2 pointOnPlane = halfSpace->position;

    //compute the signed distance from circle center to the plane
    Vector2 signedDistance = circle->position - pointOnPlane;

    //distance from the circle center to the plane
	float dot = Vector2DotProduct(signedDistance, normal);
    //this is the vector pointing from the plane toward the circle center
	Vector2 ProjectionDisplacementOnToNormal = normal * dot;

    //draw debug line showing distance to the plane
    //DrawLineEx(circle->position, pointOnPlane, 1, GRAY);
    //DrawText(TextFormat("d: %.2f", signedDistance), circle->position.x + 20, circle->position.y, 12, GRAY);

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


        //%%%%%%%%%%%%%%%%%%%
        //compute gravity force on this circle:  F = m * g
        Vector2 Fgravity = world.accelerationGravity * circle->mass;
        DrawLineEx(circle->position, circle->position + Fgravity, 2, PURPLE);   // render gravity

        Vector2 n = halfSpace->getNormal();
        Vector2 FgPrep = n * Vector2DotProduct(Fgravity, n); //project gravity onto the surface normal

        //normal force is opposite that normal component
        //it cancels the part of gravity pushing into the surface
        Vector2 Fnormal = FgPrep * -1;
        circle->netForce += Fnormal;  //add normal force to the object's net forces
        DrawLineEx(circle->position, circle->position + Fnormal, 1, GREEN);    // render normal

        //component of gravity along the surface
        //this is what tries to make the circle slide
        Vector2 FgPara = Fgravity - FgPrep;
        float FgParaLen = Vector2Length(FgPara);

        //coefficient of friction (object * plane)
        float u = circle->grippinesss * halfSpace->grippinesss;
        float frictionMagnitude = u * Vector2Length(Fnormal); //maximum possible friction 

        //determine friction direction:
        //prefer opposing in-plane velocity
        Vector2 vel = circle->velocity;
        Vector2 velParallel = vel - n * Vector2DotProduct(vel, n); //velocity projected onto plane
        float velParallelLen = Vector2Length(velParallel);

        //this will store the final friction vector
        Vector2 Ffriction = { 0, 0 };

        
        //case1 : here obj is sliding
        if (velParallelLen > 0.001f)
        {
            //kinetic friction : friction always opposes motion
            Vector2 frictionDirection = Vector2Scale(Vector2Normalize(velParallel), -1.0f);
            Ffriction = Vector2Scale(frictionDirection, frictionMagnitude);
        }

        //case2 : if obj is not sliding
        else
        {
            //object is either not moving or about to start sliding
            if (FgParaLen > 0.0001f)
            {
                //friction is strong enough to cancel in-plane gravity which means object remains not moving
                if (FgParaLen <= frictionMagnitude)
                {
                    //exactly cancel the in-plane gravity
                    Ffriction = Vector2Scale(Vector2Normalize(FgPara), -FgParaLen); // -FgPara (exact cancel)
                }
                else
                {
                    //gravity is too strong means static friction breaks
                    Vector2 frictionDirection = Vector2Scale(Vector2Normalize(FgPara), -1.0f);
                    Ffriction = Vector2Scale(frictionDirection, frictionMagnitude);
                }
            }
            else
            {
                //no in-plane forces and no sliding -> no friction
                Ffriction = { 0, 0 };
            }
        }
          

        //apply friction and draw
        circle->netForce += Ffriction;
        if (!(Ffriction.x == 0 && Ffriction.y == 0))
            DrawLineEx(circle->position, circle->position + Ffriction, 2, ORANGE); //render friction

        //%%%%%%%%%%%%%%%%%%%%%%%%%%%


        return true;
    }
    else
    {
        return false;
    }
    //return dot < circle->radius;
}

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

    /*
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
    */


    //%%%%%%%%%%%%%%%%%%%%%%%%%
    //spawn spheres
    if (IsKeyPressed(KEY_Q)) //red: 2kg, 0.1
    {
        physicsObjectCircle* s = new physicsObjectCircle();
        s->position = { spawnX, (float)GetScreenHeight() - spawnY };
        s->velocity = { 0, 0 };
        s->radius = 15;
        s->color = RED;
        s->defaultColor = RED;
        s->mass = 2.0f;
        s->grippinesss = 0.1f;
        world.add(s);
    }
    if (IsKeyPressed(KEY_W)) //green: 2kg, 0.8
    {
        physicsObjectCircle* s = new physicsObjectCircle();
        s->position = { spawnX + 24, (float)GetScreenHeight() - spawnY };
        s->velocity = { 0, 0 };
        s->radius = 15;
        s->color = GREEN;
        s->defaultColor = GREEN;
        s->mass = 2.0f;
        s->grippinesss = 0.8f;
        world.add(s);
    }
    if (IsKeyPressed(KEY_E)) //blue: 8kg, 0.1
    {
        physicsObjectCircle* s = new physicsObjectCircle();
        s->position = { spawnX + 48, (float)GetScreenHeight() - spawnY };
        s->velocity = { 0, 0 };
        s->radius = 15;
        s->color = BLUE;
        s->defaultColor = BLUE;
        s->mass = 8.0f;
        s->grippinesss = 0.1f;
        world.add(s);
    }
    if (IsKeyPressed(KEY_R)) //yellow: 8kg, 0.8
    {
        physicsObjectCircle* s = new physicsObjectCircle();
        s->position = { spawnX + 72, (float)GetScreenHeight() - spawnY };
        s->velocity = { 0, 0 };
        s->radius = 15;
        s->color = YELLOW;
        s->defaultColor = YELLOW;
        s->mass = 8.0f;
        s->grippinesss = 0.8f;
        world.add(s);
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

    //%%%%%%%%%%%%%%%%%%%
	//control coefficient of friction
	GuiSliderBar(Rectangle{ 80, 240, 200, 20 }, "u", TextFormat("%.2f", coefficientOfFriction), &coefficientOfFriction, 0.0f, 1.0f);
	//%%%%%%%%%%%%%%%%%%%

    //draw all objects
    for (int i = 0; i < world.objects.size(); i++)
        world.objects[i]->draw();

    /*
    //$$$$$$$$$$$$$
    //DrawFBD
	Vector2 location = { 300,900 };

	DrawCircle(location.x, location.y, 100, WHITE);

    //draw gravity, friction and normal

    //gravity
	Vector2 Fgravity = world.accelerationGravity * mass; 
	DrawLine(location.x, location.y, Fgravity.x, location.y + Fgravity.y, PURPLE);
	//normal
	Vector2 FgPrep = halfSpace.getNormal() * Vector2DotProduct(Fgravity, halfSpace.getNormal());
    Vector2 Fnormal = FgPrep * -1;
	DrawLine(location.x, location.y, location.x + Fnormal.x, location.y + Fnormal.y, GREEN);
    //friction
	Vector2 FgPara = Fgravity - FgPrep;
	Vector2 Ffriction = FgPara * -1;
	DrawLine(location.x, location.y, location.x + Ffriction.x, location.y + Ffriction.y, ORANGE);
    //$$$$$$$$$$$
    */

    EndDrawing();
}

//main loop
int main()
{
    InitWindow(InitialWidth, InitialHeight, "Physics Lab");
    SetTargetFPS(60);

    //create and add a static halfspace object
	halfSpace.isStatic = true;
	halfSpace.position = { 300, 800 };
	halfSpace.setRotationDegrees(0);
	world.add(&halfSpace);
	halfSpace.grippinesss = 1.0f; //%%%%%%%%%%%%%%%%%
	//added a new static halfspace to represent a bowl shape
    //halfSpace2.isStatic = true;
	//halfSpace2.position = { 900, 900 };
	//halfSpace2.setRotationDegrees(-20);
	//world.add(&halfSpace2);

    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}

