#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../PBDCollisionConstraintBatch.h"
#include "../PBDPhysicsAssetAdapter.h"
#include "../PBDClothSolver.h"
#include "Chaos/Sphere.h"
#include "Chaos/Cylinder.h"
#include "Chaos/Capsule.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "ReferenceSkeleton.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDCollisionGeometryTest, "PBD.Collision.GeometryAgainstChaos",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDCollisionGeometryTest::RunTest(const FString& Parameters)
{
    const Chaos::FSphere Sphere(Chaos::FVec3(0), 2);
    const Chaos::FCylinder Cylinder(Chaos::FVec3(0, 0, -3), Chaos::FVec3(0, 0, 3), 2);
    const Chaos::FCapsule Capsule(Chaos::FVec3(0, 0, -3), Chaos::FVec3(0, 0, 3), 2);
    const Chaos::FImplicitObject* References[] = { &Sphere, &Cylinder, &Capsule };
    FRandomStream Random(90210);
    for (int32 ShapeIndex = 0; ShapeIndex < 3; ++ShapeIndex)
    {
        FPBDCollider Collider;
        Collider.Shape = static_cast<EPBDColliderShape>(ShapeIndex);
        Collider.Radius = 2;
        Collider.HalfHeight = 3;
        Collider.Center = FVector3f(7, -4, 11);
        Collider.Rotation = FQuat4f(FVector3f(1, 2, 3).GetSafeNormal(), 0.73f);
        for (int32 Sample = 0; Sample < 256; ++Sample)
        {
            const FVector3f Local(Random.FRandRange(-5, 5), Random.FRandRange(-5, 5), Random.FRandRange(-6, 6));
            const FVector3f Point = Collider.Center + Collider.Rotation.RotateVector(Local);
            float Phi;
            FVector3f Normal;
            TestTrue(TEXT("Finite distance query"), Collider.PhiWithNormal(Point, Phi, Normal));
            Chaos::FVec3 ReferenceNormal;
            const double ReferencePhi = References[ShapeIndex]->PhiWithNormal(Chaos::FVec3(Local), ReferenceNormal);
            TestTrue(TEXT("SDF agrees with Chaos"), FMath::Abs(Phi - ReferencePhi) < 2.e-5);
            const FVector3f ExpectedNormal = Collider.Rotation.RotateVector(FVector3f(ReferenceNormal));
            TestTrue(TEXT("Normal agrees with Chaos"), Normal.Equals(ExpectedNormal, 2.e-5f));
            // Independently check that a signed-distance projection lands on the surface.
            float SurfacePhi;
            FVector3f SurfaceNormal;
            Collider.PhiWithNormal(Point - Phi * Normal, SurfacePhi, SurfaceNormal);
            TestTrue(TEXT("Projection reaches surface"), FMath::Abs(SurfacePhi) < 2.e-5f);
        }
        float Phi;
        FVector3f Normal;
        TestTrue(TEXT("Center is handled"), Collider.PhiWithNormal(Collider.Center, Phi, Normal));
        TestTrue(TEXT("Center fallback is unit"), FMath::IsNearlyEqual(Normal.Size(), 1.0f));
        TestTrue(TEXT("Center is inside"), Phi < 0);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDCollisionProjectionTest, "PBD.Collision.ProjectionAndEdgeCases",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDCollisionProjectionTest::RunTest(const FString& Parameters)
{
    for (EPBDColliderShape Shape : {EPBDColliderShape::Sphere, EPBDColliderShape::Cylinder, EPBDColliderShape::Capsule})
    {
        FPBDCollider Collider;
        Collider.Shape = Shape;
        Collider.Radius = 2;
        Collider.HalfHeight = 3;
        FPBDCollisionConstraintBatch Batch;
        Batch.SetColliders({Collider});
        Batch.SetThickness(0.5f);
        TArray<FPBDParticle> Particles;
        // Center, side, upper cap, lower cap, rounded thickness at rim, exterior, fixed point.
        for (const FVector3f P : {FVector3f(0), FVector3f(1, 0, 0), FVector3f(0, 0, 2.9f),
            FVector3f(0, 0, -2.9f), FVector3f(2.1f, 0, 3.1f), FVector3f(8, 0, 0)})
        {
            Particles.Emplace(P, 1.0f);
        }
        Particles.Emplace(FVector3f(0), 0.0f);
        FPBDConstraintContext Context;
        Context.Particles = MakeArrayView(Particles);
        Context.DeltaTime = 1.0f / 60;
        Batch.Solve(Context);
        for (int32 I = 0; I < Particles.Num() - 1; ++I)
        {
            float Phi;
            FVector3f Normal;
            Collider.PhiWithNormal(Particles[I].PredictedPosition, Phi, Normal);
            TestTrue(TEXT("All movable particles separated by thickness"), Phi >= 0.5f - 1.e-5f);
            TestEqual(TEXT("Projection does not commit old position"), Particles[I].Velocity, FVector3f::ZeroVector);
        }
        TestEqual(TEXT("Fixed point unchanged"), Particles.Last().PredictedPosition, FVector3f(0));
        TestEqual(TEXT("Exterior unchanged"), Particles[5].PredictedPosition, FVector3f(8, 0, 0));
        const FVector3f Projected = Particles[0].PredictedPosition;
        Batch.Solve(Context);
        TestTrue(TEXT("Repeated projection is idempotent"), Projected.Equals(Particles[0].PredictedPosition, 1.e-5f));
    }
    FPBDCollider Bad;
    float Phi;
    FVector3f Normal;
    Bad.Radius = -1;
    TestFalse(TEXT("Negative radius rejected"), Bad.PhiWithNormal(FVector3f(0), Phi, Normal));
    Bad.Radius = 1;
    Bad.Shape = EPBDColliderShape::Cylinder;
    TestFalse(TEXT("Zero-height cylinder rejected"), Bad.PhiWithNormal(FVector3f(0), Phi, Normal));
    Bad.Shape = EPBDColliderShape::Capsule;
    TestTrue(TEXT("Zero-length capsule is a sphere"), Bad.PhiWithNormal(FVector3f(2, 0, 0), Phi, Normal));
    TestEqual(TEXT("Zero-length capsule distance"), Phi, 1.0f);
    TestFalse(TEXT("NaN input rejected"), Bad.PhiWithNormal(FVector3f(std::numeric_limits<float>::quiet_NaN(), 0, 0), Phi, Normal));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDPhysicsAssetAdapterTest, "PBD.Collision.PhysicsAssetAdapter",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDPhysicsAssetAdapterTest::RunTest(const FString& Parameters)
{
    FReferenceSkeleton Skeleton;
    {
        // The modifier rebuilds the final bone lookup when its scope ends.
        FReferenceSkeletonModifier Modifier(Skeleton, nullptr);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("arm"), TEXT("arm"), 0), FTransform::Identity);
    }
    UPhysicsAsset* Asset = NewObject<UPhysicsAsset>();
    USkeletalBodySetup* Body = NewObject<USkeletalBodySetup>(Asset);
    Body->BoneName = TEXT("arm");
    FKSphereElem Sphere(2);
    Sphere.Center = FVector(1, 0, 0);
    Body->AggGeom.SphereElems.Add(Sphere);
    FKSphylElem Capsule(1, 6);
    Capsule.Center = FVector(0, 2, 0);
    Capsule.Rotation = FRotator(0, 0, 90);
    Body->AggGeom.SphylElems.Add(Capsule);
    Asset->SkeletalBodySetups.Add(Body);
    FPBDPhysicsAssetAdapter Adapter;
    FPBDPhysicsAssetReport Report = Adapter.Initialize(Asset, Skeleton);
    TestTrue(TEXT("Supported extraction complete"), Report.IsComplete());
    TestEqual(TEXT("Two shapes extracted"), Report.NumColliders, 2);

    const FTransform Bone(FQuat(FVector::UpVector, UE_PI / 2), FVector(10, 0, 0), FVector(2));
    TArray<FTransform> Bones = {FTransform::Identity, Bone};
    const FTransform Component(FQuat(FVector::UpVector, UE_PI / 2), FVector(100, 200, 0), FVector(3));
    const FTransform Solver(FQuat::Identity, FVector(100, 200, 0), FVector(2));
    TArray<FPBDCollider> Colliders;
    Report = Adapter.Update(Bones, Component, Solver, Colliders);
    TestTrue(TEXT("Pose conversion complete"), Report.IsComplete());
    if (!TestEqual(TEXT("Two runtime shapes"), Colliders.Num(), 2)) { return false; }
    TestTrue(TEXT("Sphere center applies bone/component/solver spaces"), Colliders[0].Center.Equals(FVector3f(-3, 15, 0), 1.e-4f));
    TestEqual(TEXT("Uniform scale radius"), Colliders[0].Radius, 6.0f);
    TestTrue(TEXT("Physics Asset sphyl stays capsule"), Colliders[1].Shape == EPBDColliderShape::Capsule);
    TestEqual(TEXT("Length excludes hemispheres"), Colliders[1].HalfHeight, 9.0f);
    const FVector ExpectedEndpoint = Solver.InverseTransformPosition(Component.TransformPosition(Bone.TransformPosition(
        Capsule.Center + Capsule.Rotation.RotateVector(FVector(0, 0, 3)))));
    const FVector3f ActualEndpoint = Colliders[1].Center + Colliders[1].Rotation.RotateVector(FVector3f(0, 0, 9));
    TestTrue(TEXT("Capsule orientation/endpoints follow bone"), ActualEndpoint.Equals(FVector3f(ExpectedEndpoint), 1.e-4f));
    Bones[1].AddToTranslation(FVector(4, 0, 0));
    Adapter.Update(Bones, Component, Solver, Colliders);
    TestTrue(TEXT("Next pose moves existing shapes"), Colliders[0].Center.Equals(FVector3f(-3, 21, 0), 1.e-4f));

    Bones[1].SetScale3D(FVector(1, 2, 1));
    Report = Adapter.Update(Bones, Component, Solver, Colliders);
    TestFalse(TEXT("Non-uniform bone scale reported"), Report.IsComplete());
    TestTrue(TEXT("Old colliders cleared on bad pose"), Colliders.IsEmpty());
    Bones.SetNum(1);
    Report = Adapter.Update(Bones, Component, Solver, Colliders);
    TestFalse(TEXT("Missing pose reported"), Report.IsComplete());
    Report = Adapter.Update(Bones, Component, FTransform(FQuat::Identity, FVector(0), FVector(-1)), Colliders);
    TestFalse(TEXT("Mirrored solver rejected"), Report.IsComplete());

    Body->AggGeom.BoxElems.Add(FKBoxElem(1, 1, 1));
    USkeletalBodySetup* Missing = NewObject<USkeletalBodySetup>(Asset);
    Missing->BoneName = TEXT("missing");
    Missing->AggGeom.SphereElems.Add(FKSphereElem(1));
    Asset->SkeletalBodySetups.Add(Missing);
    Report = Adapter.Initialize(Asset, Skeleton);
    TestEqual(TEXT("Unsupported shape and missing bone reported"), Report.Issues.Num(), 2);
    TestEqual(TEXT("Valid shapes retained"), Report.NumColliders, 2);
    Report = Adapter.Initialize(nullptr, Skeleton);
    TestFalse(TEXT("Null asset reported"), Report.IsComplete());
    Adapter.Update({}, FTransform::Identity, FTransform::Identity, Colliders);
    TestTrue(TEXT("Reinitialization clears old bindings"), Colliders.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDCollisionSolverTest, "PBD.Collision.ClothSolverIntegration",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDCollisionSolverTest::RunTest(const FString& Parameters)
{
    for (EPBDColliderShape Shape : {EPBDColliderShape::Sphere, EPBDColliderShape::Cylinder})
    {
        FPBDClothSolver Solver;
        Solver.InitializeGrid(8, 8, 2);
        FPBDCollider Collider;
        Collider.Shape = Shape;
        Collider.Center = FVector3f(0, 0, -8);
        Collider.Radius = 4;
        Collider.HalfHeight = 2;
        auto Batch = MakeUnique<FPBDCollisionConstraintBatch>();
        Batch->SetThickness(0.2f);
        Batch->SetColliders({Collider});
        Solver.RegisterConstraintBatch(MoveTemp(Batch));
        const FVector3f Pin = Solver.GetParticles()[0].Position;
        for (int32 Step = 0; Step < 120; ++Step)
        {
            Solver.Step(1.0f / 120, 8);
            for (const FPBDParticle& Particle : Solver.GetParticles())
            {
                TestFalse(TEXT("Position finite"), Particle.Position.ContainsNaN());
                TestFalse(TEXT("Velocity finite"), Particle.Velocity.ContainsNaN());
                if (Particle.InvMass > 0)
                {
                    float Phi;
                    FVector3f Normal;
                    Collider.PhiWithNormal(Particle.Position, Phi, Normal);
                    TestTrue(TEXT("Collision remains enforced after internal constraints"), Phi >= 0.2f - 1.e-4f);
                }
            }
        }
        TestEqual(TEXT("Pinned corner unchanged"), Solver.GetParticles()[0].Position, Pin);
    }
    return true;
}

#endif
