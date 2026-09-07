#include "PBDDistanceConstraintBatch.h"

FName FPBDDistanceConstraintBatch::GetDebugName() const
{
    static const FName Name(TEXT("Distance"));
    return Name;
}

int32 FPBDDistanceConstraintBatch::GetSolvePriority() const
{
    return 100;
}

void FPBDDistanceConstraintBatch::PreStep(
    FPBDConstraintContext& Context)
{
    // Lambda 只在当前子时间步的多次迭代之间保留。
    for (auto& Constraint : Constraints)
    {
        Constraint.Lambda = 0.0f;
    }
}

void FPBDDistanceConstraintBatch::Solve(
    FPBDConstraintContext& Context)
{
    if (Context.DeltaTime <= UE_SMALL_NUMBER)
    {
        return;
    }
    const float DeltaTimeSquared = Context.DeltaTime * Context.DeltaTime;

    for (auto& Constraint : Constraints)
    {
        if (!Context.Particles.IsValidIndex(Constraint.ParticleA) ||
            !Context.Particles.IsValidIndex(Constraint.ParticleB))
        {
            continue;
        }
        FPBDParticle& ParticleA = Context.Particles[Constraint.ParticleA];
        FPBDParticle& ParticleB = Context.Particles[Constraint.ParticleB];
        const float WeightSum = ParticleA.InvMass + ParticleB.InvMass;
        if (WeightSum <= UE_SMALL_NUMBER)
        {
            continue;
        }
        const FVector3f Delta = ParticleB.PredictedPosition - ParticleA.PredictedPosition;
        const float LengthSquared = Delta.SizeSquared();
        if (LengthSquared <= UE_SMALL_NUMBER)
        {
            continue;
        }
        const float Length = FMath::Sqrt(LengthSquared);
        const FVector3f Direction = Delta / Length;

        const float ConstraintError = Length - Constraint.RestLength;
        const float Alpha = Constraint.Compliance / DeltaTimeSquared;
        const float DeltaLambda = (-ConstraintError - Alpha * Constraint.Lambda) / (WeightSum + Alpha);
        Constraint.Lambda += DeltaLambda;

        ParticleA.PredictedPosition -=
            ParticleA.InvMass *
            DeltaLambda *
            Direction;

        ParticleB.PredictedPosition +=
            ParticleB.InvMass *
            DeltaLambda *
            Direction;
    }
}

void FPBDDistanceConstraintBatch::Reserve(
    int32 ConstraintCount)
{
    Constraints.Reserve(ConstraintCount);
}

void FPBDDistanceConstraintBatch::AddConstraint(
    int32 ParticleA,
    int32 ParticleB,
    float RestLength,
    float Compliance)
{
    FPBDDistanceConstraint& Constraint =
        Constraints.AddDefaulted_GetRef();

    Constraint.ParticleA = ParticleA;
    Constraint.ParticleB = ParticleB;
    Constraint.RestLength = RestLength;
    Constraint.Compliance = FMath::Max(0.0f, Compliance);
}

const TArray<FPBDDistanceConstraint>&
FPBDDistanceConstraintBatch::GetConstraints() const
{
    return Constraints;
}