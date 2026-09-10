#pragma once

#include "PBDConstraintBatch.h"

enum class EPBDColliderShape : uint8
{
    Sphere,
    Cylinder,
    Capsule
};

// Geometry is expressed in solver space, in cm. Rotation must be a unit quaternion.
// Cylinder has flat caps; Capsule has hemispherical caps. Local axis is +Z.
struct FPBDCollider
{
    EPBDColliderShape Shape = EPBDColliderShape::Sphere;
    FVector3f Center = FVector3f::ZeroVector;
    FQuat4f Rotation = FQuat4f::Identity;
    float Radius = 1.0f;
    // Cylinder: half cap-to-cap length. Capsule: half central segment length.
    float HalfHeight = 0.0f;

    // False means invalid geometry/input. Normal points out of the solid.
    bool PhiWithNormal(const FVector3f& Position, float& Phi, FVector3f& Normal) const;
};

// Hard, unilateral contacts against kinematic geometry: C(P) = Phi(P) - Thickness >= 0.
// Update the collider array before every solver substep. No UObject access during Solve.
class FPBDCollisionConstraintBatch final : public IPBDConstraintBatch
{
public:
    virtual FName GetDebugName() const override;
    virtual int32 GetSolvePriority() const override { return 300; }
    virtual void Solve(FPBDConstraintContext& Context) override;

    void SetColliders(TArray<FPBDCollider> InColliders) { Colliders = MoveTemp(InColliders); }
    const TArray<FPBDCollider>& GetColliders() const { return Colliders; }
    void SetThickness(float InThickness);
    float GetThickness() const { return Thickness; }

private:
    TArray<FPBDCollider> Colliders;
    float Thickness = 0.0f;
};
