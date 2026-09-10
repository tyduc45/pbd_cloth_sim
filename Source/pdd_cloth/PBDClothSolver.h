#pragma once

#include "PBDClothTypes.h"
#include "PBDConstraintBatch.h"
#include "PBDDistanceConstraintBatch.h"
#include "PBDBendConstraintBatch.h"

class FPBDClothSolver
{
public:
    void InitializeGrid(
        int32 NumX,
        int32 NumY,
        float Spacing);

    void Step(
        float DeltaTime,
        int32 SolverIterations);

    void Reset();
    void SetGravity(const FVector3f& InGravity) { Gravity = InGravity; }
    void AddVelocity(const FVector3f& DeltaVelocity);
    const TArray<FPBDBendConstraint>& GetBendConstraints() const;

    // 外部新增约束类型时统一通过该接口注册。
    void RegisterConstraintBatch(
        TUniquePtr<IPBDConstraintBatch> Batch);

    const TArray<FPBDParticle>& GetParticles() const;
    const TArray<FPBDTriangle>& GetTriangles() const;

    const TArray<FPBDDistanceConstraint>&
        GetDistanceConstraints() const;

private:
    void Integrate(float DeltaTime);
    void UpdateVelocities(float DeltaTime);

    TArray<FPBDParticle> Particles;
    TArray<FPBDTriangle> Triangles;

    // 负责所有约束批次的生命周期。
    TArray<TUniquePtr<IPBDConstraintBatch>>
        ConstraintBatches;

    // 非拥有指针；实际对象由 ConstraintBatches 持有。
    FPBDDistanceConstraintBatch*
        DistanceConstraintBatch = nullptr;

    FPBDBendConstraintBatch* BendConstraintBatch = nullptr;

    FVector3f Gravity =
        FVector3f(0.0f, 0.0f, -980.0f);
    /*FVector3f Gravity =
        FVector3f(0.0f, 0.0f, -0.0f);*/
};