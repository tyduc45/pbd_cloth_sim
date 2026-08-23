#pragma once
#include "CoreMinimal.h"
struct FPBDParticle
{
	FVector3f Position;
	FVector3f Velocity;
	FVector3f PredictedPosition;
	float InvMass;
	FPBDParticle(FVector3f pos, float inv_m) 
	  : Position(pos), PredictedPosition(pos), Velocity({ 0.0f, 0.0f , 0.0f }), InvMass(inv_m)
	{}
	
};

struct FPBDTriangle
{
	int32 Index1 = INDEX_NONE;
	int32 Index2 = INDEX_NONE;
	int32 Index3 = INDEX_NONE;
};

struct FPBDConstraint
{
	int32 pA = INDEX_NONE;
	int32 pB = INDEX_NONE;
	float restLength;
};
