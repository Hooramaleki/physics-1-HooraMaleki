//FINAL ASSIGNMENT

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
float restitution = 0.9f;
float weight = 1.0f;



//new shape type for HalfSpace collision object
enum physicsShapes
{
    circle,
	half_Space,
    AABB 
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
	float bounciness = 0.9f; //for determining coefficient of restitution
    std::string name = "object";
    Color color = GREEN;
    Color defaultColor = GREEN;

    //@@@@@@@@@@@@@@@@@
	bool isPig = false;  //for declaring pig objects
	bool isDead = false; //destroyed pig
    float toughness = 0.0f;
    //@@@@@@@@@@@@@@@@@

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

//the ground. static
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
        //DrawCircle(position.x, position.y, 8, color);

		DrawLineEx(position, position + normal * 30, 1, color);
    
		Vector2 parallelToSurface = Vector2Rotate(normal, PI * 0.5f);
		DrawLineEx(position - parallelToSurface * 4000, position + parallelToSurface * 4000, 1, color);
    }

    virtual physicsShapes Shape() override
    {
        return half_Space;
    }

};

//AABB object
class physicsObjectAABB : public physicsObject
{
public:
    Vector2 size;        //full width/height (size.x = width, size.y = height)
    float invMass;       //inverse mass (0 for static)

    physicsObjectAABB(Vector2 pos, Vector2 fullSize, float m)
    {
        position = pos;     //center of the AABB
        size = fullSize;    //full width/height
        //set base-class mass and isStatic
        mass = m;
        isStatic = (m == 0.0f);
        invMass = (isStatic) ? 0.0f : 1.0f / mass;
    }

	//the edges of the AABB
    float MinX() const { return position.x - size.x * 0.5f; }
    float MaxX() const { return position.x + size.x * 0.5f; }
    float MinY() const { return position.y - size.y * 0.5f; }
    float MaxY() const { return position.y + size.y * 0.5f; }

    void draw() override
    {
        DrawRectangleLines(MinX(), MinY(), size.x, size.y, color);
    }

    virtual physicsShapes Shape() override
    {
        return AABB;
    }
};


bool circleHalfSpaceOverlap(physicsObjectCircle* circle, physicsHalfSpace* halfSpace);
bool AABBHalfSpaceOverlap(physicsObjectAABB* box, physicsHalfSpace* halfSpace);

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
        }

    }

    void update()
    {
		ResetNetForces(); 
		addGravityForce();
		applyKinematics();
		checkCollision(); 
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
                if (shapeA == circle && shapeB == circle)
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

                        
                        // ------------------------------
                        //circle-circle collision response

                        //this tells us how fast B is moving toward or away from A
                        Vector2 velocityBRelativeToA = circleB->velocity - circleA->velocity;
                        //this gives the 1D closing speed between the two bodies
						float closingVelocity1D = Vector2DotProduct(velocityBRelativeToA, normalAtoB);

                        //if dot is negative then they are colliding, if positive then they are not colliding
                        if (closingVelocity1D >= 0) return;
                        //combined bounciness
						float restitution = circleA->bounciness * circleB->bounciness ; 
                        //total mass
						float totalMass = circleA->mass + circleB->mass;

						float impulseMagnitude = ((1.0f + restitution) * closingVelocity1D * circleA->mass * circleB->mass) / totalMass;


						//########################## apply impulse to each object
                        //A--> <-B
                        //A gets pushed opposite the normal, B gets pushed along the normal
						Vector2 impulseB = normalAtoB * -impulseMagnitude;
						Vector2 impulseA = normalAtoB * impulseMagnitude;


						//apply impulse to velocities
                        //this adjusts each circle velocity based on how large the impulse was
						circleA->velocity += impulseA / circleA->mass;
						circleB->velocity += impulseB / circleB->mass;

						//@@@@@@@@@@@@@@@@@@@ momentum calculation
                        //compute scalar momentum of each circle (p = m * |v|)
                        float momentumA = circleA->mass * Vector2Length(circleA->velocity);
                        float momentumB = circleB->mass * Vector2Length(circleB->velocity);

                        //total collision momentum
                        //this is a simple approximation for impact strength.
                        float totalMomentum = momentumA + momentumB;

                        //did circleA die?
                        //check if Circle A is a pig, and if the collision impact exceeds its toughness
						//if so, mark it as dead
                        if (circleA->isPig && totalMomentum > circleA->toughness)
                        {
                            circleA->isDead = true;
                        }

                        //did circleB die?
                        if (circleB->isPig && totalMomentum > circleB->toughness)
                        {
                            circleB->isDead = true;
                        }

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
                

                

                //---------------------------------------------------------------------------------------
				//aabb-aabb collision
                else if (shapeA == AABB && shapeB == AABB)
                {
                    physicsObjectAABB* A = (physicsObjectAABB*)objA;
                    physicsObjectAABB* B = (physicsObjectAABB*)objB;

                    //compute overlap on X axis
                    float dx = B->position.x - A->position.x;
                    float px = (A->size.x * 0.5f + B->size.x * 0.5f) - fabsf(dx);
                    //if this is <= 0, boxes do not overlap on X
                    if (px <= 0.0f) continue;

                    //compute overlap on Y axis
                    float dy = B->position.y - A->position.y;
                    float py = (A->size.y * 0.5f + B->size.y * 0.5f) - fabsf(dy);
                    //if this is <= 0, boxes do not overlap on Y
                    if (py <= 0.0f) continue;

                    Vector2 normal;
                    float penetration;

                    //choose axis with less penetration
                    if (px < py)
                    {
                        //resolve horizontally
                        penetration = px;
                        normal = { (dx > 0.0f) ? 1.0f : -1.0f, 0.0f };
                    }
                    else
                    {
                        //resolve vertically
                        penetration = py;
                        normal = { 0.0f, (dy > 0.0f) ? 1.0f : -1.0f };
                    }

                    //computee inverse masses
                    //if static or mass=0 ,the invMass = 0
                    float invMassA = (A->isStatic || A->mass == 0.0f) ? 0.0f : 1.0f / A->mass;
                    float invMassB = (B->isStatic || B->mass == 0.0f) ? 0.0f : 1.0f / B->mass;


                    //position correcting
                    const float percent = 0.8f;
                    const float slop = 0.01f; //(ignore tiny penetrations)

                    //compute correction magnitude with mass weighting
                    float correctionMag = fmaxf(penetration - slop, 0.0f) / (invMassA + invMassB) * percent;
                    //correction vector along collision normal
                    Vector2 correction = Vector2Scale(normal, correctionMag);

                    //move objects apart based on their inverse mass
                    if (!A->isStatic)
                        A->position = Vector2Subtract(A->position, Vector2Scale(correction, invMassA));
                    if (!B->isStatic)
                        B->position = Vector2Add(B->position, Vector2Scale(correction, invMassB));

                    A->color = RED;
                    B->color = RED;


                    //VELOCITY IMPULSE
                    //Relative velocity between A and B
                    //this finds how fast B is moving relative to A
                    Vector2 rv = Vector2Subtract(B->velocity, A->velocity);
                    //moving toward each other: negative
                    //moving apart: positive
                    float velAlongNormal = Vector2DotProduct(rv, normal);

                    //if positive, objects are separating, no impulse needed
                    if (velAlongNormal > 0.0f) continue;

                    //Compute restitution (bounce is very low)
                    float e = A->bounciness * B->bounciness;
                    if (e < 0.0f) e = 0.0f;
                    if (e > 1.0f) e = 1.0f;

                    float j = -(1.0f + e) * velAlongNormal;
                    float invMassSum = invMassA + invMassB;
                    if (invMassSum > 0.0f)
                        j /= invMassSum;
                    else
                        j = 0.0f;

                    Vector2 impulse = Vector2Scale(normal, j);

                    //apply impulse based on inverse mass
                    //object A gets pushed opposite the normal
                    //object B gets pushed along the normal
                    //lighter objects(bigger inverse mass) change velocity more
                    //Static objects do not move
                    if (!A->isStatic)
                        A->velocity = Vector2Subtract(A->velocity, Vector2Scale(impulse, invMassA));
                    if (!B->isStatic)
                        B->velocity = Vector2Add(B->velocity, Vector2Scale(impulse, invMassB));

                    //@@@@@@@@@@@@@@@@@@@@@@@
                    //FRICTION IMPULSE CALCULATION
                    
                    //rv = relative velocity, normal = n, invMassA/B computed earlier
                    
                    //this section computes friction between two colliding bodies (A and B)
                    
                    //we already have:
                    //rv = relative velocity (vB - vA)
                    //normal = collision normal
                    //j = normal impulse magnitude (computed earlier)
                    //invMassA/B = inverse masses
                    //invMassSum = invMassA + invMassB


                    //compute the tangent direction:
                    //remove the normal component from the relative velocity
                    Vector2 tangent = Vector2Subtract(rv, Vector2Scale(normal, Vector2DotProduct(rv, normal)));
                    float tangentLen = Vector2Length(tangent);

					//normalize tangent
                    if (tangentLen > 1e-6f) tangent = Vector2Scale(tangent, 1.0f / tangentLen);
                    else tangent = { 0,0 };

                    //vt = relative velocity along the tangent direction (how fast the objects slide)
                    float vt = Vector2DotProduct(rv, tangent);

                    //compute friction impulse magnitude jt
                    //opposes sliding, so sign is reversed
                    float jt = 0.0f;
                    if (invMassSum > 0.0f) {
                        jt = -vt / invMassSum;
                        //static friction limit
                        float mu = sqrtf(A->grippinesss * B->grippinesss); 
                        float maxJt = j * mu;

                        //clamp friction impulse
                        //prevents unrealistic super friction
                        if (jt > maxJt) jt = maxJt;
                        if (jt < -maxJt) jt = -maxJt;
                        Vector2 frictionImpulse = Vector2Scale(tangent, jt);

                        //apply friction impulse to velocities
                        if (!A->isStatic) A->velocity = Vector2Subtract(A->velocity, Vector2Scale(frictionImpulse, invMassA));
                        if (!B->isStatic) B->velocity = Vector2Add(B->velocity, Vector2Scale(frictionImpulse, invMassB));
                    }

                }

                //AABB vs circle collision
                else if (shapeA == AABB && shapeB == circle)
                {
                    physicsObjectAABB* box = (physicsObjectAABB*)objA;
                    physicsObjectCircle* circ = (physicsObjectCircle*)objB;

                    //find nearest point on AABB to circle center
                    Vector2 half = { box->size.x * 0.5f, box->size.y * 0.5f };

                    //clamp circle center to inside the AABB bounds
                    float nx = fmaxf(box->position.x - half.x,
                        fminf(circ->position.x, box->position.x + half.x));
                    float ny = fmaxf(box->position.y - half.y,
                        fminf(circ->position.y, box->position.y + half.y));

                    Vector2 closest = { nx, ny };

                    //CHECKING IF CIRCLE IS COLLIDING WITH BOX
                    Vector2 diff = Vector2Subtract(circ->position, closest); //vector from box to circle
                    float distSq = diff.x * diff.x + diff.y * diff.y;
                    float r = circ->radius;

                    //if the distance is bigger than the radius then they are not colliding.
                    if (distSq >= r * r) continue; //no collision

                    //compute collision normal
                    float dist = sqrtf(distSq);
                    Vector2 normal;

                    if (dist != 0.0f)
                        normal = { diff.x / dist, diff.y / dist };
                    else
                        normal = { 0.0f, -1.0f }; //circle center exactly on box corner


					//compute penetration depth
                    float penetration = r - dist; //how deep the circle is inside the box

                    //inverse masses (same rule: static = 0 mass)
                    //used so lighter objects move more on collision
                    float invMassA = (box->isStatic || box->mass == 0.0f) ? 0.0f : 1.0f / box->mass;
                    float invMassB = (circ->isStatic || circ->mass == 0.0f) ? 0.0f : 1.0f / circ->mass;

                    //POSITION CORRECTION (removing overlap)
                    //small % of penetration is corrected
                    const float percent = 0.2f;
                    const float slop = 0.01f;

                    float correctionMag = fmaxf(penetration - slop, 0.0f) / (invMassA + invMassB) * percent;
                    Vector2 correction = Vector2Scale(normal, correctionMag);

					//push box backward and circle forward based on their inverse mass
                    if (!box->isStatic)
                        box->position = Vector2Subtract(box->position, Vector2Scale(correction, invMassA));
                    if (!circ->isStatic)
                        circ->position = Vector2Add(circ->position, Vector2Scale(correction, invMassB));

                    box->color = RED;
                    circ->color = RED;

                    //IMPULSE RESPONSE
                    Vector2 rv = Vector2Subtract(circ->velocity, box->velocity);

                    //project onto normal
                    float velAlongNormal = Vector2DotProduct(rv, normal);
                    if (velAlongNormal > 0) continue;

                    float restitution = box->bounciness * circ->bounciness; //combine bounciness

                    float j = -(1.0f + restitution) * velAlongNormal;
                    float invMassSum = invMassA + invMassB;

                    if (invMassSum > 0)
                        j /= invMassSum;
                    else
                        j = 0.0f;

                    Vector2 impulse = Vector2Scale(normal, j);

                    //apply impulse (heavier objects move less)
                    if (!box->isStatic)
                        box->velocity = Vector2Subtract(box->velocity, Vector2Scale(impulse, invMassA));
                    if (!circ->isStatic)
                        circ->velocity = Vector2Add(circ->velocity, Vector2Scale(impulse, invMassB));


                }

                //AABB - HalfSpace collision (boxes stay on ground)
                else if (shapeA == AABB && shapeB == half_Space)
                {
					AABBHalfSpaceOverlap((physicsObjectAABB*)objA, (physicsHalfSpace*)objB);
                }
                else if (shapeA == half_Space && shapeB == AABB)
                {
					AABBHalfSpaceOverlap((physicsObjectAABB*)objB, (physicsHalfSpace*)objA);
                }
            }
        }
    }
};


float speed = 80;
float angle = 0;
float spawnX = 100;
float spawnY = 300;


//slingshot / dragging state
bool isDragging = false;
float slingWidth = 20.0f;
float slingHeight = 100.0f;
Vector2 slingshotAnchor = { 150.0f,  (float)GetScreenHeight() - 200.0f };
Vector2 dragPos = { 0.0f, 0.0f };   //current pos mouse while dragging
float maxPull = 300.0f;             //clamp the pull distance
float launchSpeedScale = 2.0f;      //converts pull distance -> launch speed
Rectangle slingshotRect = { slingshotAnchor.x - 12.0f, slingshotAnchor.y - 24.0f, slingWidth, slingHeight }; //x,y,w,h for rectangle slingshot


//@@@@@@@@@@@ BIRD TYPE
enum BirdType
{ 
    BIRD_CIRCLE, 
    BIRD_BOX 
};

BirdType currentBird = BIRD_CIRCLE; //default selected bird


physicsWorld world;
physicsHalfSpace halfSpace;
//physicsHalfSpace halfSpace2; //to represent a bowl shape
float haldspaceAngle = 0;

//@@@@@@@@@@@   toggle bird type
void ToggleBirdType()
{
    currentBird = (currentBird == BIRD_CIRCLE) ? BIRD_BOX : BIRD_CIRCLE;
}

void SpawnBirdWithVelocity(Vector2 vel)
{
    //spawn position (same as your previous spawn location)
    Vector2 spawnPos = { spawnX, (float)GetScreenHeight() - spawnY };

    if (currentBird == BIRD_CIRCLE)
    {
        physicsObjectCircle* b = new physicsObjectCircle();
        b->position = spawnPos;
        b->velocity = vel;
        b->radius = 15.0f;
        b->mass = 1.0f;                 //light circular bird
        b->bounciness = restitution;
        b->grippinesss = 0.2f;         //tune friction
        b->color = RED;
        b->defaultColor = RED;
        world.add(b);
    }
    else //BIRD_BOX
    {
        //full width/height for constructor
        Vector2 boxSize = { 40.0f, 40.0f };
        float boxMass = 6.0f;           //heavier square bird

        physicsObjectAABB* b = new physicsObjectAABB(spawnPos, boxSize, boxMass);
        b->velocity = vel;
        b->bounciness = restitution;
        b->grippinesss = 0.4f;         //tune friction (higher than circle)
        b->color = BLUE;
        b->defaultColor = BLUE;
        world.add(b);
    }
}
//@@@@@@@@@@@


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

        //bouncing
        //from perspective of A
		//collision response of circle and half-space

        //project the circle velocity into the collision normal
		//this gives the circle moving into or away from the half-space
        float closingVelocity1D = Vector2DotProduct(circle->velocity, n);

        //if dot is negative then we are colliding, if positive then we are not colliding
        if (closingVelocity1D >= 0) return true;

        //the restitution is calculated by each of the objects bounciness
		float restitution = circle->bounciness * halfSpace->bounciness;
        
        //this flips the normal component of velocity and scales it by restitution
		//velFinal = velInitial + -(1 + restitution * velInitial)
		circle->velocity += n * closingVelocity1D * -(1.0f + restitution);

        

        //case1
        if (velParallelLen > 0.001f)
        {
            //kinetic friction : friction always opposes motion
            Vector2 frictionDirection = Vector2Scale(Vector2Normalize(velParallel), -1.0f);
            Ffriction = Vector2Scale(frictionDirection, frictionMagnitude);
        }

        //case2
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




        return true;
    }
    else
    {
        return false;
    }
    //return dot < circle->radius;
}

//detects overlap between a box and a half-space
bool AABBHalfSpaceOverlap(physicsObjectAABB* box, physicsHalfSpace* halfSpace)
{

    Vector2 normal = halfSpace->getNormal();
    Vector2 pointOnPlane = halfSpace->position;

    //compute the signed distance from box center to the plane
    Vector2 signedDistance = box->position - pointOnPlane;

    //distance from the box center to the plane
    float dot = Vector2DotProduct(signedDistance, normal);
    //this is the vector pointing from the plane toward the box center
    Vector2 ProjectionDisplacementOnToNormal = normal * dot;


    float dx = box->position.y - ProjectionDisplacementOnToNormal.y;
    float overlap = (box->size.y * 0.5f + box->position.y) - fabsf(dx);

    if (overlap > 0)
    {
        //i dont know if we still need them to turn red touching the half-space
        //box->color = RED;

        //compute the Minimum Translation Vector (MTV)
        //tells us how much to move the box out of the half-space
        //mtv = insertion depth
        Vector2 mtv = normal * overlap;
        box->position += mtv; //move box out of half-space



        //compute gravity force on this box:  F = m * g
        Vector2 Fgravity = world.accelerationGravity * box->mass;
        DrawLineEx(box->position, box->position + Fgravity, 2, PURPLE);   // render gravity

        Vector2 n = halfSpace->getNormal();
        Vector2 FgPrep = n * Vector2DotProduct(Fgravity, n); //project gravity onto the surface normal

        //normal force is opposite that normal component
        //it cancels the part of gravity pushing into the surface
        Vector2 Fnormal = FgPrep * -1;
        box->netForce += Fnormal;  //add normal force to the object's net forces
        DrawLineEx(box->position, box->position + Fnormal, 1, GREEN);    // render normal

        //component of gravity along the surface
        //this is what tries to make the box slide
        Vector2 FgPara = Fgravity - FgPrep;
        float FgParaLen = Vector2Length(FgPara);

        //coefficient of friction (object * plane)
        float u = box->grippinesss * halfSpace->grippinesss;
        float frictionMagnitude = u * Vector2Length(Fnormal); //maximum possible friction 

        //determine friction direction:
        //prefer opposing in-plane velocity
        Vector2 vel = box->velocity;
        Vector2 velParallel = vel - n * Vector2DotProduct(vel, n); //velocity projected onto plane
        float velParallelLen = Vector2Length(velParallel);

        //this will store the final friction vector
        Vector2 Ffriction = { 0, 0 };

        //bouncing
        //from perspective of A
        //collision response of box and half-space

        //project the box velocity into the collision normal
        //this gives the box moving into or away from the half-space
        float closingVelocity1D = Vector2DotProduct(box->velocity, n);

        //if dot is negative then we are colliding, if positive then we are not colliding
        if (closingVelocity1D >= 0) return true;

        //the restitution is calculated by each of the objects bounciness
        float restitution = box->bounciness * halfSpace->bounciness;

        //this flips the normal component of velocity and scales it by restitution
        //velFinal = velInitial + -(1 + restitution * velInitial)
        box->velocity += n * closingVelocity1D * -(1.0f + restitution);



        //case1
        if (velParallelLen > 0.001f)
        {
            //kinetic friction : friction always opposes motion
            Vector2 frictionDirection = Vector2Scale(Vector2Normalize(velParallel), -1.0f);
            Ffriction = Vector2Scale(frictionDirection, frictionMagnitude);
        }

        //case2
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
        box->netForce += Ffriction;
        if (!(Ffriction.x == 0 && Ffriction.y == 0))
            DrawLineEx(box->position, box->position + Ffriction, 2, ORANGE); //render friction




        return true;
    }
    else
    {
        return false;
    }
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


	//dragging logic @@@@@@@@@@@
    //mouse drag slingshot (drag & release)
    Vector2 mouse = GetMousePosition();

    //start dragging when left button pressed
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
    {
        //recompute rectangle position each click so its centered on the anchor
        slingshotRect.x = slingshotAnchor.x - slingshotRect.width * 0.5f;
        slingshotRect.y = slingshotAnchor.y - slingshotRect.height * 0.5f;

        //only start dragging if:
        //the mouse is inside the slingshot rectangle
        // or the mouse is near the slingshot anchor (within 40 pixels)
        if (CheckCollisionPointRec(mouse, slingshotRect) || Vector2Distance(mouse, slingshotAnchor) < 40.0f)
        {
            isDragging = true;
            dragPos = mouse;
        }
    }

    //UPDATE DRAG WHILE HELD
    //while dragging, update the current drag position (rubber-band end)
    if (isDragging && IsMouseButtonDown(MOUSE_LEFT_BUTTON))
    {
        dragPos = GetMousePosition();
    }


    // RELEASE = LAUNCH
    if (isDragging && IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
    {
        isDragging = false;
        //pull vector is direction and strength of slingshot pull
        //(From the dragged point to the anchor)
        Vector2 pull = Vector2Subtract(slingshotAnchor, dragPos);
        //how far the slingshot was stretched
        float pullLen = Vector2Length(pull);

        if (pullLen > 2.0f) //avoid tiny clicks
        {
			//clamp pull length (to prevent extreme launches)
            float clamped = fminf(pullLen, maxPull);

            //convert pull distance into launch speed
            float launchSpeed = clamped * launchSpeedScale;

			//launch direction
            Vector2 direction = Vector2Normalize(pull);
            //final launch velocity
            Vector2 launchVel = Vector2Scale(direction, launchSpeed);

            //spawn the selected bird with computed velocity
            SpawnBirdWithVelocity(launchVel);
        }
    }


    //for the existing keyboard spawn
    if (IsKeyPressed(KEY_SPACE))
    {
        Vector2 launchVel = {
            speed * cosf(angle * DEG2RAD),
            -speed * sinf(angle * DEG2RAD)
        };
        SpawnBirdWithVelocity(launchVel);
    }

    //press TAB to toggle bird type
    if (IsKeyPressed(KEY_TAB)) 
        ToggleBirdType();

    //@@@@@@@@@@@@@@@@@


    /*
    if (IsKeyPressed(KEY_SPACE)) //spawn circle
    {
        physicsObjectCircle* newBird = new physicsObjectCircle();
        newBird->position = { spawnX, (float)GetScreenHeight() - spawnY };
        newBird->velocity = { speed * (float)cos(angle * DEG2RAD),
                              -speed * (float)sin(angle * DEG2RAD) };
        newBird->radius = 15;
        newBird->color = GREEN;
		newBird->bounciness = restitution;
        newBird->mass = weight;

        world.add(newBird);
    }
    */

	//spawn AABB box with pressing B key
    /*
    if (IsKeyPressed(KEY_B))  // spawn an AABB projectile
    {
        Vector2 boxSize = { 40.0f, 40.0f };  // width, height (full size)

        physicsObjectAABB* newBox = new physicsObjectAABB(
            { spawnX, (float)GetScreenHeight() - spawnY },   // start position
            boxSize,                                         // full width/height
            weight                                           // mass
        );

        newBox->velocity =
        {
        (float)(speed * cos(angle * DEG2RAD)),
        (float)(-speed * sin(angle * DEG2RAD))
        };


        newBox->bounciness = restitution;
        newBox->color = GREEN;
        newBox->defaultColor = GREEN;

        world.add(newBox);
    }
    */

    /*
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
    */


    world.checkCollision(); //update collisions

    for (int i = world.objects.size() - 1; i >= 0; i--)
    {
        if (world.objects[i]->isPig && world.objects[i]->isDead)
        {
            delete world.objects[i];
            world.objects.erase(world.objects.begin() + i);
        }
    }

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

    DrawText("Spawn Y:", 10, 60, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 60, 300, 20 }, "", TextFormat("%.0f", spawnY), &spawnY, 0.0f, (float)GetScreenHeight());

    DrawText("Speed:", 10, 80, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 80, 300, 20 }, "", TextFormat("%.0f", speed), &speed, 0.0f, (float)GetScreenWidth());

    DrawText("Angle:", 10, 100, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 100, 300, 20 }, "", TextFormat("%.0f°", angle), &angle, 0.0f, 180.0f);

    DrawText("Gravity (Y):", 10, 120, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 120, 300, 20 }, "", TextFormat("%.1f", world.accelerationGravity.y), &world.accelerationGravity.y, -180.0f, 180.0f);

	//half space
    //GUI sliders for adjusting half-space position and rotation
    //DrawText("HalfSpace (X):", 10, 160, 14, RAYWHITE);
    //GuiSliderBar(Rectangle{ 120, 160, 250, 20 }, "", TextFormat("%.0f", halfSpace.position.x), &halfSpace.position.x, 0.0f, (float)GetScreenWidth());
    //DrawText("HalfSpace (y):", 10, 180, 14, RAYWHITE);
    //GuiSliderBar(Rectangle{ 120, 180, 250, 20 }, "", TextFormat("%.0f", halfSpace.position.y), &halfSpace.position.y, 0.0f, (float)GetScreenWidth());

	float halfspaceRotation = halfSpace.getRotation();
    //DrawText("Rotation:", 10, 200, 14, RAYWHITE);
    //GuiSliderBar(Rectangle{ 120, 200, 250, 20 }, "", TextFormat("%.0f", halfSpace.getRotation()), &halfspaceRotation, 0.0f, (float)GetScreenWidth());


	//control restitution 
    DrawText("Restitution:", 10, 240, 14, RAYWHITE);
	GuiSliderBar(Rectangle{ 120, 240, 250, 20 }, "", TextFormat("%.2f", restitution), &restitution, 0.0f, 1.0f);

	//mass adjusting slider
    DrawText("Mass:", 10, 280, 14, RAYWHITE);
    GuiSliderBar(Rectangle{ 120, 280, 300, 20 }, "", TextFormat("%.1f", weight), &weight, 0.1f, 20.0f);


    DrawText("*** SPACE = launch *** press 1 for 0 degrees, 2 for 45, 3 for 60, 4 for 90", 10, 310, 14, RAYWHITE);


	//reset world button
    /*
    if (GuiButton(Rectangle{ 1300, 40, 120, 30 }, "Reset World"))
    {
        for (int i = world.objects.size() - 1; i >= 0; i--)
        {
            if (!world.objects[i]->isStatic)
            {
                delete world.objects[i];
                world.objects.erase(world.objects.begin() + i);
            }
        }
    }
    */

    halfSpace.setRotationDegrees(halfspaceRotation);

    //bouncy balls
    /*
    //restitution = 1.0, friction low, mass small
    if (GuiButton(Rectangle{ 1300, 70, 120, 30 }, "Bouncy"))
    {
        restitution = 1.0f;
        coefficientOfFriction = 0.1f;
        weight = 1.0f;
    }

    //pool table
    //friction high, restitution around 0.3, mass medium
    if (GuiButton(Rectangle{ 1300, 100, 120, 30 }, "Pool Table"))
    {
        restitution = 0.3f;
        coefficientOfFriction = 0.8f;
        weight = 5.0f;

        //adding the stationary ball
        physicsObjectCircle* target = new physicsObjectCircle();

        target->position = { spawnX + 200, (float)GetScreenHeight() - spawnY + 85};
        target->velocity = { 0, 0 };    //stationary
        target->radius = 15;
        target->mass = weight;
        target->bounciness = restitution;
        target->color = BLUE;
        target->defaultColor = BLUE;

        world.add(target);
    }

    //galilean Cannon
    //heavy ball + light ball with different masses
    if (GuiButton(Rectangle{ 1300, 130, 120, 30 }, "Galilean Cannon"))
    {
        restitution = 0.9f;

        //spawn 2 balls
        //big one
        physicsObjectCircle* big = new physicsObjectCircle();

        big->position = { spawnX, (float)GetScreenHeight() - spawnY };
        big->radius = 20;
        big->mass = 10;
        big->velocity = { 0,0 };

        world.add(big);

        //small one
        physicsObjectCircle* small = new physicsObjectCircle();

        small->position = { spawnX, (float)GetScreenHeight() - spawnY - 50 };
        small->radius = 10;
        small->mass = 5;
        small->velocity = { 0,0 };

        world.add(small);
    }
    */


    //draw launch line
    Vector2 startPos = { spawnX, GetScreenHeight() - spawnY };
    //Vector2 velocity = { speed * cos(angle * DEG2RAD), -speed * sin(angle * DEG2RAD) };
    //DrawLineEx(startPos, startPos + velocity, 3, RED);

    //@@@@@@@@@@@@@@@@@@@@@@@@
    // SLINGSHOT RENDERING
    //draws the slingshot rectangle, rubber-band lines, and a preview of the bird
    //this runs every frame inside the Draw() section
    //construct the slingshot rectangle based on current anchor + user-defined size
    //this ensures the slingshot always stays centered on the anchor point
    Rectangle slingshotRect = {
    slingshotAnchor.x - slingWidth * 0.5f,
    slingshotAnchor.y - slingHeight * 0.5f,
    slingWidth,
    slingHeight
    };


    //rectangle outline
    DrawRectangleLinesEx(slingshotRect, 2, WHITE);

    //if dragging, draw rubber band
    if (isDragging)
    {
        //compute raw pull vector, used to determine stretch length
        Vector2 rawPull = Vector2Subtract(slingshotAnchor, dragPos);
        float pullLen = Vector2Length(rawPull);

        //clamp the drag position so the visual rubber band can't exceed maxPull
        Vector2 cappedDrag = dragPos;
        if (pullLen > maxPull)
        {
            cappedDrag = Vector2Add(slingshotAnchor, Vector2Scale(Vector2Normalize(Vector2Subtract(dragPos, slingshotAnchor)), maxPull));
        }

        //rubber band drawing
        Vector2 leftFork = { slingshotRect.x + 4.0f, slingshotRect.y + 50 * 0.3f };
        Vector2 rightFork = { slingshotRect.x + slingshotRect.width - 4.0f, slingshotRect.y + 50 * 0.3f };

        DrawLineEx(leftFork, cappedDrag, 4, BLACK);
        DrawLineEx(rightFork, cappedDrag, 4, BLACK);


        //preview bird at the slingshot anchor (small circle or box) showing what will be launched
		Vector2 adjustment = { 0.0f, -50.0f };

        if (currentBird == BIRD_CIRCLE)
        {
            DrawCircleV(slingshotAnchor + adjustment, 12, RED);
        }
        else
        {
            DrawRectanglePro(Rectangle{ slingshotAnchor.x - 12.0f, slingshotAnchor.y - 12.0f, 24.0f, 24.0f }, Vector2{ 12.0f,12.0f }, 0.0f, BLUE);
        }

        //show a tiny velocity preview vector
        Vector2 pullDir = Vector2Normalize(Vector2Subtract(slingshotAnchor, cappedDrag));
        float previewSpeed = fminf(Vector2Length(rawPull), maxPull) * launchSpeedScale;
        DrawLineEx(slingshotAnchor + adjustment, Vector2Add(slingshotAnchor, Vector2Scale(pullDir, previewSpeed * 0.9f)), 3, DARKGRAY);
    }
    //@@@@@@@@@@@@@@@@@@@@@@@@@@


	//control coefficient of friction
	//GuiSliderBar(Rectangle{ 80, 240, 200, 20 }, "u", TextFormat("%.2f", coefficientOfFriction), &coefficientOfFriction, 0.0f, 1.0f);


    //draw all objects
    for (int i = 0; i < world.objects.size(); i++)
        world.objects[i]->draw();

    /*
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
    */

    EndDrawing();
}

//loop
int main()
{
    InitWindow(InitialWidth, InitialHeight, "Physics Lab");
    SetTargetFPS(60);

    //create and add a static halfspace object
	halfSpace.isStatic = true;
	halfSpace.position = { 300, 800 };
	halfSpace.setRotationDegrees(0);
	world.add(&halfSpace);
	halfSpace.grippinesss = 1.0f;


    //slingshiot setup @@@@@@@@@@@
    slingshotAnchor = { spawnX, 750.0f };
    slingshotRect = { slingshotAnchor.x - 12.0f, slingshotAnchor.y - 24.0f, 24.0f, 48.0f };


    /*
	added a new static halfspace to represent a bowl shape
    halfSpace2.isStatic = true;
	halfSpace2.position = { 900, 900 };
	halfSpace2.setRotationDegrees(-20);
	world.add(&halfSpace2);
    */

	//AABB ground
    //tower
    
    //1.
    physicsObjectAABB* box1 = new physicsObjectAABB(
        { 530, 780},     //1 height above base
        { 50, 50 },            //size
        2.0f                   //mass
    );
    box1->bounciness = 0.1f;
    box1->color = WHITE;
    world.add(box1);

	//2.
    physicsObjectAABB* box2 = new physicsObjectAABB(
        { 530, 730},     //stacked above box1
        { 50, 50 },            //size
        2.0f                   //mass
    );
    box2->bounciness = 0.1f;
    box2->color = WHITE;
    world.add(box2);

	//3.
    physicsObjectAABB* box3 = new physicsObjectAABB(
        { 530, 680},    //stacked above box2
        { 50, 50 },            //size
        2.0f                   //mass
    );
    box3->bounciness = 0.1f;
    box3->color = WHITE;
    world.add(box3);

    //4.
    physicsObjectAABB* box4 = new physicsObjectAABB(
        { 670, 780 },     //stacked above box3
        { 50, 50 },            //size
        2.0f                   //mass
    );
    box4->bounciness = 0.1f;
    box4->color = WHITE;
    world.add(box4);

    //5.
    physicsObjectAABB* box5 = new physicsObjectAABB(
        { 670, 730 },     //stacked above box4
        { 50, 50 },            //size
        2.0f                   //mass
    );
    box5->bounciness = 0.1f;
    box5->color = WHITE;
    world.add(box5);

    //6
    physicsObjectAABB* box6 = new physicsObjectAABB(
        { 670, 680 },    //stacked above box5
        { 50, 50 },            //size
        2.0f                   //mass
    );
    box6->bounciness = 0.1f;
    box6->color = WHITE;
    world.add(box6);



	//roof box
    physicsObjectAABB* RoofBox = new physicsObjectAABB
    (
        { 600, 640 },          //position
        { 200, 40 },           //width, normal height
        5.0f                   //mass heavier
        );
    RoofBox->bounciness = 0.1f;
    RoofBox->color = WHITE;
    world.add(RoofBox);




    //@@@@@@@@@@@@@@@
    //pigs

	//pig1 (inside the fortress)
    physicsObjectCircle* pig1 = new physicsObjectCircle();
    pig1->position = { 580, 785 };
    pig1->radius = 15;
    pig1->mass = 1.0f;
    pig1->color = GREEN;
    pig1->isPig = true;
    pig1->toughness = 7.0f;   //the force needed to kill it

    world.add(pig1);

	//pig 2 (inside the fortress)
    physicsObjectCircle* pig2 = new physicsObjectCircle();
    pig2->position = { 620, 785 };
    pig2->radius = 15;
    pig2->mass = 1.0f;
    pig2->color = GREEN;
    pig2->isPig = true;
    pig2->toughness = 7.0f;

    world.add(pig2);

	//pig 3 (on the roof)
	physicsObjectCircle* pig3 = new physicsObjectCircle();
	pig3->position = { 600, 605 };
	pig3->radius = 15;
	pig3->mass = 1.0f;
	pig3->color = GREEN;
	pig3->isPig = true;
	pig3->toughness = 7.0f;

	world.add(pig3);




    while (!WindowShouldClose()) {
        update();
        draw();
    }

    CloseWindow();
    return 0;
}

