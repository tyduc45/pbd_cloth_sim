#include "PBDPhysicsAssetAdapter.h"

#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"

namespace
{
bool HasSupportedTransform(const FTransform& Transform)
{
    const FVector Scale = Transform.GetScale3D();
    return !Transform.ContainsNaN() && Transform.GetRotation().IsNormalized() &&
        Scale.X > UE_SMALL_NUMBER &&
        FMath::IsNearlyEqual(Scale.X, Scale.Y, 1.e-6) &&
        FMath::IsNearlyEqual(Scale.X, Scale.Z, 1.e-6);
}
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::Initialize(
    const UPhysicsAsset* Asset, const FReferenceSkeleton& Skeleton)
{
    Bindings.Reset();
    FPBDPhysicsAssetReport Report;
    if (!Asset)
    {
        Report.Issues.Add(TEXT("No Physics Asset supplied."));
        return Report;
    }

    for (const USkeletalBodySetup* Body : Asset->SkeletalBodySetups)
    {
        if (!Body)
        {
            Report.Issues.Add(TEXT("Null skeletal body setup."));
            continue;
        }
        const int32 BoneIndex = Skeleton.FindBoneIndex(Body->BoneName);
        if (BoneIndex == INDEX_NONE)
        {
            Report.Issues.Add(FString::Printf(TEXT("Missing bone: %s"), *Body->BoneName.ToString()));
            continue;
        }

        const auto Add = [&](const FPBDCollider& Collider)
        {
            float Phi;
            FVector3f Normal;
            if (!Collider.PhiWithNormal(Collider.Center, Phi, Normal))
            {
                Report.Issues.Add(FString::Printf(TEXT("Invalid shape on bone: %s"), *Body->BoneName.ToString()));
                return;
            }
            Bindings.Add({Body->BoneName, BoneIndex, Collider});
        };
        const FKAggregateGeom& Geometry = Body->AggGeom;
        for (const FKSphereElem& Sphere : Geometry.SphereElems)
        {
            FPBDCollider Collider;
            Collider.Center = FVector3f(Sphere.Center);
            Collider.Radius = Sphere.Radius;
            Add(Collider);
        }
        for (const FKSphylElem& Capsule : Geometry.SphylElems)
        {
            FPBDCollider Collider;
            Collider.Shape = EPBDColliderShape::Capsule;
            Collider.Center = FVector3f(Capsule.Center);
            Collider.Rotation = FQuat4f(Capsule.Rotation.Quaternion());
            Collider.Radius = Capsule.Radius;
            Collider.HalfHeight = Capsule.Length * 0.5f;
            Add(Collider);
        }
        const int32 Unsupported = Geometry.GetElementCount() - Geometry.SphereElems.Num() - Geometry.SphylElems.Num();
        if (Unsupported > 0)
        {
            Report.Issues.Add(FString::Printf(TEXT("Skipped %d unsupported shapes on bone %s (only sphere/sphyl supported)."),
                Unsupported, *Body->BoneName.ToString()));
        }
    }
    Report.NumColliders = Bindings.Num();
    return Report;
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::InitializeFromComponent(const USkeletalMeshComponent& Component)
{
    if (const USkeletalMesh* Mesh = Component.GetSkeletalMeshAsset())
    {
        return Initialize(Component.GetPhysicsAsset(), Mesh->GetRefSkeleton());
    }
    Bindings.Reset();
    FPBDPhysicsAssetReport Report;
    Report.Issues.Add(TEXT("Component has no skeletal mesh."));
    return Report;
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::Update(
    TConstArrayView<FTransform> ComponentSpaceBones, const FTransform& ComponentToWorld,
    const FTransform& SolverToWorld, TArray<FPBDCollider>& OutColliders) const
{
    OutColliders.Reset();
    FPBDPhysicsAssetReport Report;
    if (!HasSupportedTransform(ComponentToWorld) || !HasSupportedTransform(SolverToWorld))
    {
        Report.Issues.Add(TEXT("Component and solver transforms require finite, positive uniform scale and unit rotation."));
        return Report;
    }
    OutColliders.Reserve(Bindings.Num());
    for (const FBinding& Binding : Bindings)
    {
        if (!ComponentSpaceBones.IsValidIndex(Binding.BoneIndex) ||
            !HasSupportedTransform(ComponentSpaceBones[Binding.BoneIndex]))
        {
            Report.Issues.Add(FString::Printf(TEXT("Missing pose or unsupported bone transform: %s"), *Binding.BoneName.ToString()));
            continue;
        }
        const FTransform& Bone = ComponentSpaceBones[Binding.BoneIndex];
        FPBDCollider Collider = Binding.BoneLocalCollider;
        const FVector WorldCenter = ComponentToWorld.TransformPosition(Bone.TransformPosition(FVector(Collider.Center)));
        Collider.Center = FVector3f(SolverToWorld.InverseTransformPosition(WorldCenter));
        Collider.Rotation = FQuat4f(SolverToWorld.GetRotation().Inverse() * ComponentToWorld.GetRotation() *
            Bone.GetRotation() * FQuat(Collider.Rotation));
        Collider.Rotation.Normalize();
        const double Scale = Bone.GetScale3D().X * ComponentToWorld.GetScale3D().X / SolverToWorld.GetScale3D().X;
        Collider.Radius *= Scale;
        Collider.HalfHeight *= Scale;
        float Phi;
        FVector3f Normal;
        if (!Collider.PhiWithNormal(Collider.Center, Phi, Normal))
        {
            Report.Issues.Add(FString::Printf(TEXT("Transformed collider is not finite/valid: %s"), *Binding.BoneName.ToString()));
            continue;
        }
        OutColliders.Add(Collider);
    }
    Report.NumColliders = OutColliders.Num();
    return Report;
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::UpdateFromComponent(
    const USkeletalMeshComponent& Component, const FTransform& SolverToWorld,
    TArray<FPBDCollider>& OutColliders) const
{
    const USkeletalMesh* Mesh = Component.GetSkeletalMeshAsset();
    for (const FBinding& Binding : Bindings)
    {
        if (!Mesh || Mesh->GetRefSkeleton().FindBoneIndex(Binding.BoneName) != Binding.BoneIndex)
        {
            OutColliders.Reset();
            FPBDPhysicsAssetReport Report;
            Report.Issues.Add(TEXT("Reference skeleton changed; reinitialize the Physics Asset adapter."));
            return Report;
        }
    }
    return Update(Component.GetComponentSpaceTransforms(), Component.GetComponentTransform(), SolverToWorld, OutColliders);
}
