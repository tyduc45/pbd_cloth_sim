#include "PBDCollisionConstraintBatch.h"

bool FPBDCollider::PhiWithNormal(const FVector3f& Position, float& Phi, FVector3f& Normal) const
{
    Phi = 0.0f;
    Normal = FVector3f::ZeroVector;
    if (Position.ContainsNaN() || Center.ContainsNaN() || Rotation.ContainsNaN() ||
        !Rotation.IsNormalized() || !FMath::IsFinite(Radius) || Radius <= 0.0f ||
        !FMath::IsFinite(HalfHeight) || HalfHeight < 0.0f)
    {
        return false;
    }

    const FVector3f Local = Rotation.UnrotateVector(Position - Center);
    FVector3f LocalNormal;
    switch (Shape)
    {
    case EPBDColliderShape::Sphere:
    case EPBDColliderShape::Capsule:
    {
        const float SegmentZ = Shape == EPBDColliderShape::Capsule
            ? FMath::Clamp(Local.Z, -HalfHeight, HalfHeight) : 0.0f;
        const FVector3f Delta = Local - FVector3f(0.0f, 0.0f, SegmentZ);
        const float Distance = Delta.Size();
        Phi = Distance - Radius;
        // The gradient is ambiguous at the center/segment. Choose a stable unit direction.
        LocalNormal = Distance > UE_SMALL_NUMBER ? Delta / Distance : FVector3f(1, 0, 0);
        break;
    }
    case EPBDColliderShape::Cylinder:
    {
        if (HalfHeight <= 0.0f)
        {
            return false;
        }
        const float Rho = FVector2f(Local.X, Local.Y).Size();
        const FVector3f RadialNormal = Rho > UE_SMALL_NUMBER
            ? FVector3f(Local.X / Rho, Local.Y / Rho, 0) : FVector3f(1, 0, 0);
        const FVector3f CapNormal(0, 0, Local.Z >= 0.0f ? 1.0f : -1.0f);
        const float SideDistance = Rho - Radius;
        const float CapDistance = FMath::Abs(Local.Z) - HalfHeight;
        const float OutsideSide = FMath::Max(SideDistance, 0.0f);
        const float OutsideCap = FMath::Max(CapDistance, 0.0f);
        const float OutsideDistance = FVector2f(OutsideSide, OutsideCap).Size();
        Phi = OutsideDistance + FMath::Min(FMath::Max(SideDistance, CapDistance), 0.0f);
        if (OutsideDistance > UE_SMALL_NUMBER)
        {
            // Outside a rim, the closest feature is the circular edge, not just a plane.
            LocalNormal = (OutsideSide * RadialNormal + OutsideCap * CapNormal) / OutsideDistance;
        }
        else
        {
            LocalNormal = SideDistance >= CapDistance ? RadialNormal : CapNormal;
        }
        break;
    }
    default:
        return false;
    }

    Normal = Rotation.RotateVector(LocalNormal);
    return FMath::IsFinite(Phi) && !Normal.ContainsNaN();
}

FName FPBDCollisionConstraintBatch::GetDebugName() const
{
    static const FName Name(TEXT("Collision"));
    return Name;
}

void FPBDCollisionConstraintBatch::SetThickness(float InThickness)
{
    Thickness = FMath::IsFinite(InThickness) ? FMath::Max(InThickness, 0.0f) : 0.0f;
}

void FPBDCollisionConstraintBatch::Solve(FPBDConstraintContext& Context)
{
    for (FPBDParticle& Particle : Context.Particles)
    {
        if (!FMath::IsFinite(Particle.InvMass) || Particle.InvMass <= 0.0f)
        {
            continue;
        }
        for (const FPBDCollider& Collider : Colliders)
        {
            float Phi;
            FVector3f Normal;
            if (Collider.PhiWithNormal(Particle.PredictedPosition, Phi, Normal) && Phi < Thickness)
            {
                Particle.PredictedPosition += (Thickness - Phi) * Normal;
            }
        }
    }
}
