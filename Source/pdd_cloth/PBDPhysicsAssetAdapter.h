#pragma once

#include "PBDCollisionConstraintBatch.h"

class UPhysicsAsset;
class USkeletalMeshComponent;
struct FReferenceSkeleton;

struct FPBDPhysicsAssetReport
{
    int32 NumColliders = 0;
    TArray<FString> Issues;
    bool IsComplete() const { return Issues.IsEmpty(); }
};

// Read Physics Asset once, update the resulting kinematic geometry each substep.
// Extraction/component access belongs on the game thread after animation evaluation.
// No asset pointers are retained. Reinitialize when asset or reference skeleton changes.
class FPBDPhysicsAssetAdapter
{
public:
    FPBDPhysicsAssetReport Initialize(const UPhysicsAsset* Asset, const FReferenceSkeleton& Skeleton);
    FPBDPhysicsAssetReport InitializeFromComponent(const USkeletalMeshComponent& Component);

    // Bone transforms are COMPONENT space, indexed by the initialization reference skeleton.
    // Replaces OutColliders, including on failure, so old collision bodies cannot remain active.
    // Supports positive uniform scale only. Other scales are explicitly reported and skipped.
    FPBDPhysicsAssetReport Update(TConstArrayView<FTransform> ComponentSpaceBones, const FTransform& ComponentToWorld, const FTransform& SolverToWorld, TArray<FPBDCollider>& OutColliders) const;
    FPBDPhysicsAssetReport UpdateFromComponent(const USkeletalMeshComponent& Component, const FTransform& SolverToWorld, TArray<FPBDCollider>& OutColliders) const;

private:
    struct FBinding
    {
        FName BoneName;
        int32 BoneIndex = INDEX_NONE;
        FPBDCollider BoneLocalCollider;
    };
    TArray<FBinding> Bindings;
};
