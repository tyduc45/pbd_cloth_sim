#pragma once

#include "CoreMinimal.h"
#include "PBDClothTypes.h"

struct FPBDConstraintContext
{
	TArrayView<FPBDParticle> Particles;
	float DeltaTime = 0.0f;
	int32 Iteration = 0;
};

class IPBDConstraintBatch
{
public:
	virtual ~IPBDConstraintBatch() = default;

	virtual FName GetDebugName() const = 0;

	virtual int32 GetSolvePriority() const = 0;

	virtual void PreStep(FPBDConstraintContext& Context) {}

	virtual void Solve(FPBDConstraintContext& Context) = 0;

	virtual void PostStep(FPBDConstraintContext& Context) {}

};



