#define  _USE_MATH_DEFINES
#define SOKOL_IMPL
#define SOKOL_GLCORE33
#include "sokol_app.h"
#include "sokol_gfx.h"
#include "sokol_gp.h"
#include "sokol_glue.h"
#include "sokol_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <vector>

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

static const int num_segments = 60;

void sgp_draw_outline_circle(float cx, float cy, float radius) {
    const float step = 2.0f * M_PI / num_segments;
    for (int i = 0; i < num_segments; i++) {
        float a1 = step * i;
        float a2 = step * (i + 1);
        sgp_draw_line(
          cx + cosf(a1) * radius,
          cy + sinf(a1) * radius,
          cx + cosf(a2) * radius,
          cy + sinf(a2) * radius
        );
    }
}

void sgp_draw_filled_circle(float cx, float cy, float radius) {
    const float step = radius / num_segments;
    for (float y = -radius; y <= radius; y += step) {
        float x = sqrtf(radius * radius - y * y);
        sgp_draw_filled_rect(
          cx - x,
          cy + y,
          x * 2,
          step
        );
    }
}

struct color_t {
	float r,g,b;
};

color_t colors[] = {
	{0.8f, 0.0f, 0.0f},
	{0.0f, 0.8f, 0.0f},
	{0.0f, 0.0f, 0.8f},
	{0.8f, 0.0f, 0.8f},
	{0.8f, 0.8f, 0.0f},
	{0.0f, 0.8f, 0.8f},
	{0.6f, 0.8f, 0.6f},
	{0.5f, 0.5f, 0.8f},
	{0.0f, 0.5f, 0.5f},
	{0.5f, 0.0f, 0.5f},
	{0.5f, 0.5f, 0.5f},
};

typedef enum BoneId
{
    boneId_hip = 0,
    boneId_torso = 1,
    boneId_head = 2,
    boneId_upperLeftLeg = 3,
    boneId_lowerLeftLeg = 4,
    boneId_upperRightLeg = 5,
    boneId_lowerRightLeg = 6,
    boneId_upperLeftArm = 7,
    boneId_lowerLeftArm = 8,
    boneId_upperRightArm = 9,
    boneId_lowerRightArm = 10,
    boneId_count = 11,
} BoneId;

typedef struct Bone
{
    b2BodyId bodyId;
    b2JointId jointId;
    float frictionScale;
    int parentIndex;
} Bone;

typedef struct Human
{
    Bone bones[boneId_count];
    float scale;
    bool isSpawned;
} Human;

void CreateHuman( Human* human, b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz, float dampingRatio,
						   int groupIndex, void* userData, bool colorize )
{
	assert( human->isSpawned == false );

	for ( int i = 0; i < boneId_count; ++i )
	{
		human->bones[i].bodyId = b2_nullBodyId;
		human->bones[i].jointId = b2_nullJointId;
		human->bones[i].frictionScale = 1.0f;
		human->bones[i].parentIndex = -1;
	}

	human->scale = scale;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.sleepThreshold = 0.1f;
	bodyDef.userData = userData;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.friction = 0.2f;
	shapeDef.filter.groupIndex = -groupIndex;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ( 1 | 2 );

	b2ShapeDef footShapeDef = shapeDef;
	footShapeDef.friction = 0.05f;

	// feet don't collide with ragdolls
	footShapeDef.filter.categoryBits = 2;
	footShapeDef.filter.maskBits = 1;

	if ( colorize )
	{
		footShapeDef.customColor = b2_colorSaddleBrown;
	}

	float s = scale;
	float maxTorque = frictionTorque * s;
	bool enableMotor = true;
	bool enableLimit = true;
	float drawSize = 0.05f;

	b2HexColor shirtColor = b2_colorMediumTurquoise;
	b2HexColor pantColor = b2_colorDodgerBlue;

	b2HexColor skinColors[4] = { b2_colorNavajoWhite, b2_colorLightYellow, b2_colorPeru, b2_colorTan };
	b2HexColor skinColor = skinColors[groupIndex % 4];

	// hip
	{
		Bone* bone = human->bones + boneId_hip;
		bone->parentIndex = -1;

		bodyDef.position = b2Add( { 0.0f, 0.95f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.02f * s }, { 0.0f, 0.02f * s }, 0.095f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	// torso
	{
		Bone* bone = human->bones + boneId_torso;
		bone->parentIndex = boneId_hip;

		bodyDef.position = b2Add( { 0.0f, 1.2f * s }, position );
		bodyDef.linearDamping = 0.0f;
		// bodyDef.type = b2_staticBody;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;
		bodyDef.type = b2_dynamicBody;

		if ( colorize )
		{
			shapeDef.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.135f * s }, { 0.0f, 0.135f * s }, 0.09f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.0f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.25f * b2_pi;
		jointDef.upperAngle = 0.0f;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// head
	{
		Bone* bone = human->bones + boneId_head;
		bone->parentIndex = boneId_torso;

		bodyDef.position = b2Add( { 0.0f * s, 1.475f * s }, position );
		bodyDef.linearDamping = 0.1f;

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.25f;

		if ( colorize )
		{
			shapeDef.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.038f * s }, { 0.0f, 0.039f * s }, 0.075f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		//// neck
		// capsule = { { 0.0f, -0.12f * s }, { 0.0f, -0.08f * s }, 0.05f * s };
		// b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.4f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.3f * b2_pi;
		jointDef.upperAngle = 0.1f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper left leg
	{
		Bone* bone = human->bones + boneId_upperLeftLeg;
		bone->parentIndex = boneId_hip;

		bodyDef.position = b2Add( { 0.0f, 0.775f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 1.0f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.06f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 0.9f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.05f * b2_pi;
		jointDef.upperAngle = 0.4f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	b2Vec2 points[4] = {
		{ -0.03f * s, -0.185f * s },
		{ 0.11f * s, -0.185f * s },
		{ 0.11f * s, -0.16f * s },
		{ -0.03f * s, -0.14f * s },
	};

	b2Hull footHull = b2ComputeHull( points, 4 );
	b2Polygon footPolygon = b2MakePolygon( &footHull, 0.015f * s );

	// lower left leg
	{
		Bone* bone = human->bones + boneId_lowerLeftLeg;
		bone->parentIndex = boneId_upperLeftLeg;

		bodyDef.position = b2Add( { 0.0f, 0.475f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.155f * s }, { 0.0f, 0.125f * s }, 0.045f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		// b2Polygon box = b2MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
		// b2CreatePolygonShape(bone->bodyId, &shapeDef, &box);

		// capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
		// b2CreateCapsuleShape( bone->bodyId, &footShapeDef, &capsule );

		b2CreatePolygonShape( bone->bodyId, &footShapeDef, &footPolygon );

		b2Vec2 pivot = b2Add( { 0.0f, 0.625f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = -0.02f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper right leg
	{
		Bone* bone = human->bones + boneId_upperRightLeg;
		bone->parentIndex = boneId_hip;

		bodyDef.position = b2Add( { 0.0f, 0.775f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 1.0f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.06f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 0.9f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.05f * b2_pi;
		jointDef.upperAngle = 0.4f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// lower right leg
	{
		Bone* bone = human->bones + boneId_lowerRightLeg;
		bone->parentIndex = boneId_upperRightLeg;

		bodyDef.position = b2Add( { 0.0f, 0.475f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.155f * s }, { 0.0f, 0.125f * s }, 0.045f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		// b2Polygon box = b2MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
		// b2CreatePolygonShape(bone->bodyId, &shapeDef, &box);

		// capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
		// b2CreateCapsuleShape( bone->bodyId, &footShapeDef, &capsule );

		b2CreatePolygonShape( bone->bodyId, &footShapeDef, &footPolygon );

		b2Vec2 pivot = b2Add( { 0.0f, 0.625f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = -0.02f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper left arm
	{
		Bone* bone = human->bones + boneId_upperLeftArm;
		bone->parentIndex = boneId_torso;
		bone->frictionScale = 0.5f;

		bodyDef.position = b2Add( { 0.0f, 1.225f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );

		if ( colorize )
		{
			shapeDef.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.035f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.35f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.1f * b2_pi;
		jointDef.upperAngle = 0.8f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// lower left arm
	{
		Bone* bone = human->bones + boneId_lowerLeftArm;
		bone->parentIndex = boneId_upperLeftArm;

		bodyDef.position = b2Add( { 0.0f, 0.975f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.1f;

		if ( colorize )
		{
			shapeDef.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.03f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.1f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.referenceAngle = 0.25f * b2_pi;
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.2f * b2_pi;
		jointDef.upperAngle = 0.3f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper right arm
	{
		Bone* bone = human->bones + boneId_upperRightArm;
		bone->parentIndex = boneId_torso;

		bodyDef.position = b2Add( { 0.0f, 1.225f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.035f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.35f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.1f * b2_pi;
		jointDef.upperAngle = 0.8f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// lower right arm
	{
		Bone* bone = human->bones + boneId_lowerRightArm;
		bone->parentIndex = boneId_upperRightArm;

		bodyDef.position = b2Add( { 0.0f, 0.975f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.1f;

		if ( colorize )
		{
			shapeDef.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.03f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.1f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.referenceAngle = 0.25f * b2_pi;
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.2f * b2_pi;
		jointDef.upperAngle = 0.3f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	human->isSpawned = true;
}

void DestroyHuman( Human* human )
{
	assert( human->isSpawned == true );

	for ( int i = 0; i < boneId_count; ++i )
	{
		if ( B2_IS_NULL( human->bones[i].jointId ) )
		{
			continue;
		}

		b2DestroyJoint( human->bones[i].jointId );
		human->bones[i].jointId = b2_nullJointId;
	}

	for ( int i = 0; i < boneId_count; ++i )
	{
		if ( B2_IS_NULL( human->bones[i].bodyId ) )
		{
			continue;
		}

		b2DestroyBody( human->bones[i].bodyId );
		human->bones[i].bodyId = b2_nullBodyId;
	}

	human->isSpawned = false;
}

static struct {
    struct {
        b2Vec2 gravity = {0.0f, -5.0f};
        b2WorldId worldId = b2_nullWorldId;
        float timeStep = 1.0f / 60.0f;
        int32_t subStepCount = 4;
    } b2d_global_ctx;
    struct {
        b2BodyId bodyId = b2_nullBodyId;
        const float halfWidth = 50.0f;
        const float halfHeight = 5.0f;
    } s_ground;
    Human d_human;
} state;

static void init(void) {
    sg_desc sgdesc = {};
    sgdesc.context = sapp_sgcontext();
    sgdesc.logger.func = slog_func;

    sg_setup(&sgdesc);
    if(!sg_isvalid()) {
        fprintf(stderr, "Failed to create Sokol GFX context!\n");
        exit(-1);
    }

    sgp_desc sgpdesc = {0};
    sgp_setup(&sgpdesc);
    if(!sgp_is_valid()) {
        fprintf(stderr, "Failed to create Sokol GP context: %s\n", sgp_get_error_message(sgp_get_last_error()));
        exit(-1);
    }

    // world
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = state.b2d_global_ctx.gravity;

    state.b2d_global_ctx.worldId = b2CreateWorld(&worldDef);

    // ground
    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    groundBodyDef.position = {0.0f, -10.f};

    state.s_ground.bodyId = b2CreateBody(state.b2d_global_ctx.worldId, &groundBodyDef);

    b2Polygon shape = b2MakeBox(state.s_ground.halfWidth, state.s_ground.halfHeight);

    b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    b2CreatePolygonShape(state.s_ground.bodyId, &groundShapeDef, &shape);

    // human
	CreateHuman(&state.d_human, state.b2d_global_ctx.worldId, { 0.0f, 25.0f }, 10.0f, 0.03f, 5.0f, 0.5f, 1,
					 nullptr, false);
}

static void cleanup(void) {
	DestroyHuman(&state.d_human);
    b2DestroyWorld(state.b2d_global_ctx.worldId);
    sgp_shutdown();
    sg_shutdown();
}

static void frame(void) {
    int width = sapp_width(), height = sapp_height();
    float ratio = width/(float)height;

    sgp_begin(width, height);
    sgp_viewport(0, 0, width, height);

    const float halfDemoHeight = 30.0f;
    sgp_project(-ratio*halfDemoHeight, ratio*halfDemoHeight, halfDemoHeight, -halfDemoHeight);

    sgp_set_color(0.1f, 0.1f, 0.1f, 1.0f);
    sgp_clear();

    state.b2d_global_ctx.timeStep = static_cast<float>(sapp_frame_duration());
    b2World_Step(state.b2d_global_ctx.worldId, state.b2d_global_ctx.timeStep, state.b2d_global_ctx.subStepCount);

    // draw ground
    {
        b2Vec2 position = b2Body_GetPosition(state.s_ground.bodyId);
        sgp_push_transform();
        sgp_set_color(1.0f, 1.0f, 1.0f, 1.0f);
        sgp_translate(position.x, position.y);
        sgp_draw_filled_rect(-state.s_ground.halfWidth, -state.s_ground.halfHeight, state.s_ground.halfWidth*2, state.s_ground.halfHeight*2);
        sgp_pop_transform();
    }

    // draw human
    {
		for (int boneIdx = 0; boneIdx < boneId_count; ++boneIdx)
		{
			Bone& bone = state.d_human.bones[boneIdx];
			int bodyCount = b2Body_GetShapeCount(bone.bodyId);
			//assert(bodyCount == 1);

			std::vector<b2ShapeId> shapeIds(bodyCount);
			b2Body_GetShapes(bone.bodyId, shapeIds.data(), bodyCount);

			// draw joints
			{
				b2Capsule capsule = b2Shape_GetCapsule(shapeIds[bodyCount-1]);
				b2Vec2 position = b2Body_GetPosition(bone.bodyId);
				float angle = b2Rot_GetAngle(b2Body_GetRotation(bone.bodyId));

				sgp_push_transform();
				sgp_translate(position.x, position.y);
				sgp_rotate(angle);
				sgp_set_color(colors[boneIdx].r, colors[boneIdx].g, colors[boneIdx].b, 1.0f);
				float dx = fabsf(capsule.center1.x - capsule.center2.x);
				float dy = fabsf(capsule.center1.y - capsule.center2.y);
				float h = fmaxf(dx, dy);
				float w = 2.0f * capsule.radius;
				sgp_draw_filled_rect(-0.5f * w, -0.5f * h, w, h);
				sgp_draw_filled_circle(capsule.center1.x, capsule.center1.y, capsule.radius);
				sgp_draw_filled_circle(capsule.center2.x, capsule.center2.y, capsule.radius);
				sgp_pop_transform();
			}

			// draw foot
			if (bodyCount > 1)
			{
				b2Polygon foot = b2Shape_GetPolygon(shapeIds[0]);
				b2Vec2 position = b2Body_GetPosition(bone.bodyId);
				float angle = b2Rot_GetAngle(b2Body_GetRotation(bone.bodyId));

				sgp_push_transform();
				sgp_translate(position.x, position.y);
				sgp_rotate(angle);
				sgp_set_color(colors[boneIdx].r, colors[boneIdx].g, colors[boneIdx].b, 1.0f);
				std::vector<sgp_point> vertices(foot.count);
				for	(int32_t i = 0; i < foot.count; ++i)
				{
					vertices[i].x = foot.vertices[i].x;
					vertices[i].y = foot.vertices[i].y;
				}
				sgp_draw_lines_strip(vertices.data(), vertices.size());
				sgp_pop_transform();
			}
		}
    }

    sg_pass_action pass_action = {0};
    sg_begin_default_pass(&pass_action, width, height);

    sgp_flush();
    sgp_end();

    sg_end_pass();
    sg_commit();
}

sapp_desc sokol_main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    sapp_desc app_desc{};
    app_desc.init_cb = init;
    app_desc.frame_cb = frame;
    app_desc.cleanup_cb = cleanup;
    app_desc.window_title = "hello box2d";
    app_desc.logger.func = slog_func;
    app_desc.width = 1280;
    app_desc.height = 720;
    app_desc.high_dpi = true;
    return app_desc;
}