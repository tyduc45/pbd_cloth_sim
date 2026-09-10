#include "PBDClothComparisonActor.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/Canvas.h"
#include "CanvasTypes.h"
#include "Debug/DebugDrawService.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Chaos/PBDSoftsSolverParticles.h"
#include "Chaos/PBDSpringConstraints.h"
#include "Chaos/PBDBendingConstraints.h"
#include "Chaos/PerParticlePBDCollisionConstraint.h"
#include "Chaos/Sphere.h"
#include "Chaos/Cylinder.h"
#include "Chaos/Capsule.h"

class FChaosClothComparisonState
{
public:
    Chaos::Softs::FSolverParticles Particles;
    TUniquePtr<Chaos::Softs::FPBDSpringConstraints> Distance;
    TUniquePtr<Chaos::Softs::FPBDBendingConstraints> Bend;
    Chaos::Softs::FSolverCollisionParticles Bodies;
    Chaos::TPBDActiveView<Chaos::Softs::FSolverCollisionParticles> BodyView{Bodies};
    TArray<bool> Collided;
    TArray<uint32> DynamicGroups, BodyGroups;
    TArray<float> Thickness{0.5f}, Friction{0.0f};

    void UpdateColliders(const TArray<FPBDCollider>& Colliders, float InThickness)
    {
        using namespace Chaos;
        BodyView.Reset();
        Bodies.RemoveAt(0, Bodies.Size());
        Bodies.AddParticles(Colliders.Num());
        BodyView.AddRange(Colliders.Num());
        BodyGroups.Init(0, Colliders.Num());
        Collided.Init(false, Colliders.Num());
        Thickness[0] = InThickness;
        for (int32 I = 0; I < Colliders.Num(); ++I)
        {
            const FPBDCollider& C = Colliders[I];
            Bodies.X(I) = C.Center;
            Bodies.SetR(I, Softs::FSolverRotation3(C.Rotation));
            Bodies.V(I) = Bodies.W(I) = FVector3f::ZeroVector;
            const FVec3 A(0, 0, -C.HalfHeight), B(0, 0, C.HalfHeight);
            FImplicitObjectPtr Geometry;
            switch (C.Shape)
            {
            case EPBDColliderShape::Sphere: Geometry = new Chaos::FSphere(FVec3(0), C.Radius); break;
            case EPBDColliderShape::Cylinder: Geometry = new FCylinder(A, B, C.Radius); break;
            case EPBDColliderShape::Capsule: Geometry = new FCapsule(A, B, C.Radius); break;
            }
            Bodies.SetGeometry(I, Geometry);
        }
    }

    void Initialize(const FPBDClothSolver& Source)
    {
        using namespace Chaos;
        using namespace Chaos::Softs;
        const TArray<FPBDParticle>& SourceParticles = Source.GetParticles();
        Particles.AddParticles(SourceParticles.Num());
        DynamicGroups.Init(0, SourceParticles.Num());
        for (int32 I = 0; I < SourceParticles.Num(); ++I)
        {
            const FPBDParticle& P = SourceParticles[I];
            Particles.X(I) = P.Position;
            Particles.P(I) = P.PredictedPosition;
            Particles.V(I) = P.Velocity;
            Particles.M(I) = P.InvMass > 0.0f ? 1.0f / P.InvMass : 0.0f;
            Particles.InvM(I) = P.InvMass;
            Particles.PAndInvM(I).InvM = P.InvMass;
            Particles.Acceleration(I) = FSolverVec3(0.0f);
        }

        TArray<TVec2<int32>> Edges;
        for (const FPBDDistanceConstraint& C : Source.GetDistanceConstraints())
        {
            Edges.Add(TVec2<int32>(C.ParticleA, C.ParticleB));
        }
        TArray<TVec4<int32>> Hinges;
        for (const FPBDBendConstraint& C : Source.GetBendConstraints())
        {
            Hinges.Add(TVec4<int32>(C.Particle1, C.Particle2, C.Particle3, C.Particle4));
        }

        const TConstArrayView<FRealSingle> NoWeights;
        // Stiffness=1 matches our Compliance=0 projection. No tether,
        // damping, aerodynamics, animation drive, or additional material constraints.
        Distance = MakeUnique<FPBDSpringConstraints>(Particles, 0, SourceParticles.Num(),
            Edges, NoWeights, FSolverVec2(1.0f), false, false);
        Bend = MakeUnique<FPBDBendingConstraints>(Particles, 0, SourceParticles.Num(),
            MoveTemp(Hinges), NoWeights, NoWeights, NoWeights, NoWeights,
            FSolverVec2(1.0f), FSolverVec2(0.0f), FSolverVec2(1.0f), FSolverVec2(0.0f),
            FPBDBendingConstraintsBase::ERestAngleConstructionType::Use3DRestAngles, false);
        check(Distance->GetConstraints().Num() == Source.GetDistanceConstraints().Num());
        check(Bend->GetConstraints().Num() == Source.GetBendConstraints().Num());
    }

    void AddVelocity(const FVector3f& DeltaVelocity)
    {
        for (int32 I = 0; I < static_cast<int32>(Particles.Size()); ++I)
        {
            if (Particles.InvM(I) > 0.0f)
            {
                Particles.V(I) += DeltaVelocity;
            }
        }
    }

    void Step(float Dt, int32 Iterations, const FVector3f& Acceleration)
    {
        for (int32 I = 0; I < static_cast<int32>(Particles.Size()); ++I)
        {
            if (Particles.InvM(I) <= 0.0f)
            {
                Particles.V(I) = FVector3f::ZeroVector;
                Particles.P(I) = Particles.X(I);
                continue;
            }
            Particles.V(I) += Acceleration * Dt;
            Particles.P(I) = Particles.X(I) + Particles.V(I) * Dt;
        }

        Distance->ApplyProperties(Dt, Iterations);
        Bend->ApplyProperties(Dt, Iterations);
        Bend->Init(Particles);
        Chaos::Softs::FPerParticlePBDCollisionConstraint Collision(
            BodyView, Collided, DynamicGroups, BodyGroups, Thickness, Friction);
        for (int32 Iteration = 0; Iteration < Iterations; ++Iteration)
        {
            Distance->Apply(Particles, Dt);
            Bend->Apply(Particles, Dt);
            if (Bodies.Size()) { Collision.ApplyRange(Particles, Dt, 0, Particles.Size()); }
        }
        const float InverseDt = 1.0f / Dt;
        for (int32 I = 0; I < static_cast<int32>(Particles.Size()); ++I)
        {
            if (Particles.InvM(I) <= 0.0f)
            {
                Particles.V(I) = FVector3f::ZeroVector;
                Particles.P(I) = Particles.X(I);
                continue;
            }
            Particles.V(I) = (Particles.P(I) - Particles.X(I)) * InverseDt;
            Particles.X(I) = Particles.P(I);
        }
    }
};

APBDClothComparisonActor::APBDClothComparisonActor()
{
    PrimaryActorTick.bCanEverTick = true;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    ComparisonCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ComparisonCamera"));
    ComparisonCamera->SetupAttachment(RootComponent);
    ComparisonCamera->SetRelativeLocation(FVector(0.0, -650.0, 20.0));
    ComparisonCamera->SetRelativeRotation(FRotator(0.0, 90.0, 0.0));
    ComparisonCamera->FieldOfView = 55.0f;
    ProjectileSpheres = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("ProjectileSpheres"));
    ProjectileCylinders = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("ProjectileCylinders"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    ProjectileSpheres->SetStaticMesh(SphereMesh.Object);
    ProjectileCylinders->SetStaticMesh(CylinderMesh.Object);
    for (UInstancedStaticMeshComponent* Mesh : {ProjectileSpheres.Get(), ProjectileCylinders.Get()})
    {
        Mesh->SetupAttachment(RootComponent);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mesh->SetCanEverAffectNavigation(false);
    }
}

APBDClothComparisonActor::~APBDClothComparisonActor() = default;

void APBDClothComparisonActor::BeginPlay()
{
    Super::BeginPlay();
    ResetComparison();
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
    {
        PC->SetViewTarget(this);
        FInputModeGameAndUI InputMode;
        InputMode.SetHideCursorDuringCapture(false);
        PC->SetInputMode(InputMode);
        PC->bShowMouseCursor = true;
        PC->DefaultMouseCursor = EMouseCursor::Crosshairs;
        PC->CurrentMouseCursor = EMouseCursor::Crosshairs;
    }
    CrosshairHandle = UDebugDrawService::Register(TEXT("Game"),
        FDebugDrawDelegate::CreateUObject(this, &APBDClothComparisonActor::DrawCrosshair));
}

void APBDClothComparisonActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    UDebugDrawService::Unregister(CrosshairHandle);
    Super::EndPlay(EndPlayReason);
}

void APBDClothComparisonActor::DrawCrosshair(UCanvas* Canvas, APlayerController* PlayerController)
{
    APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
    float X, Y;
    if (!Canvas || !PC || PC->GetViewTarget() != this || !PC->GetMousePosition(X, Y)) { return; }
    const float DpiScale = Canvas->Canvas ? Canvas->Canvas->GetDPIScale() : 1.0f;
    X /= DpiScale;
    Y /= DpiScale;
    // Draw in viewport pixels as well as requesting a system cross cursor, so custom
    // Windows cursor themes cannot remove the aiming reticle.
    for (const FVector2D Axis : {FVector2D(1, 0), FVector2D(0, 1)})
    {
        for (float Sign : {-1.0f, 1.0f})
        {
            const FVector2D A = FVector2D(X, Y) + Axis * (4 * Sign);
            const FVector2D B = FVector2D(X, Y) + Axis * (13 * Sign);
            Canvas->K2_DrawLine(A, B, 4, FLinearColor::Black);
            Canvas->K2_DrawLine(A, B, 2, FLinearColor::White);
        }
    }
}

void APBDClothComparisonActor::ResetComparison()
{
    NumX = FMath::Clamp(NumX, 2, 64);
    NumY = FMath::Clamp(NumY, 2, 64);
    Spacing = FMath::Clamp(Spacing, 0.1f, 1000.0f);
    CustomSolver.InitializeGrid(NumX, NumY, Spacing);
    auto NewCollision = MakeUnique<FPBDCollisionConstraintBatch>();
    CollisionBatch = NewCollision.Get();
    CustomSolver.RegisterConstraintBatch(MoveTemp(NewCollision));
    Shots.Reset();
    ProjectileSpheres->ClearInstances();
    ProjectileCylinders->ClearInstances();
    ShotRandom.Initialize(RandomSeed);
    ShotsFired = 0;
    LastShape = TEXT("none");
    ChaosState = MakePimpl<FChaosClothComparisonState>();
    ChaosState->Initialize(CustomSolver);
    Accumulator = 0.0;
    SimulationTime = 0.0;
    StepCount = 0;
    bInvalidState = false;
    ViewSeparation = (NumX - 1) * Spacing + 80.0f;
    const float HalfWidth = (ViewSeparation + (NumX - 1) * Spacing) * 0.5f;
    const float HalfHeight = (NumY - 1) * Spacing * 0.5f + 50.0f;
    const float CameraDistance = FMath::Max(HalfWidth, HalfHeight * 1.8f) / FMath::Tan(FMath::DegreesToRadians(27.5f)) * 1.15f;
    ComparisonCamera->SetRelativeLocation(FVector(0.0, -CameraDistance, 20.0));
    UE_LOG(LogTemp, Display, TEXT("[PBDComparison] Reset: %d particles; %d distance; %d bend per side. Custom Compliance=0; Chaos stiffness=1; distance/bend/collision; friction=0; CCD off."),
        CustomSolver.GetParticles().Num(), CustomSolver.GetDistanceConstraints().Num(), CustomSolver.GetBendConstraints().Num());
}

void APBDClothComparisonActor::PushBoth()
{
    if (!ChaosState || bInvalidState) { return; }
    const FVector3f DeltaVelocity(PushVelocity);
    CustomSolver.AddVelocity(DeltaVelocity);
    ChaosState->AddVelocity(DeltaVelocity);
}

void APBDClothComparisonActor::FireAtLocalTarget(FVector Target, int32 Shape)
{
    if (!ChaosState) { ResetComparison(); }
    if (bInvalidState || Target.ContainsNaN() || Shots.Num() >= 12) { return; }
    FShot Shot;
    Shot.Collider.Shape = static_cast<EPBDColliderShape>(Shape >= 0 && Shape <= 2 ? Shape : ShotRandom.RandRange(0, 2));
    Shot.Collider.Radius = ShotRandom.FRandRange(12.0f, 20.0f);
    Shot.Collider.HalfHeight = ShotRandom.FRandRange(15.0f, 28.0f);
    Shot.Collider.Rotation = FQuat4f(FRotator(ShotRandom.FRandRange(-70, 70),
        ShotRandom.FRandRange(-70, 70), ShotRandom.FRandRange(-70, 70)).Quaternion());
    Target.Y = 0;
    // Parallel launch lanes: each visual copy has precisely the same solver-space path.
    Shot.Collider.Center = FVector3f(Target + FVector(0, -180, 0));
    const float Speed = FMath::IsFinite(ProjectileSpeed) ? FMath::Clamp(ProjectileSpeed, 30.0f, 600.0f) : 180.0f;
    Shot.Velocity = FVector3f(0, Speed, 0);
    Shots.Add(Shot);
    ++ShotsFired;
    const TCHAR* Names[] = {TEXT("Sphere"), TEXT("Cylinder"), TEXT("Capsule")};
    LastShape = Names[static_cast<int32>(Shot.Collider.Shape)];
}

void APBDClothComparisonActor::FireFromMouse()
{
    APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
    FVector Origin, Direction;
    if (!PC || !PC->DeprojectMousePositionToWorld(Origin, Direction)) { return; }
    const FTransform Transform = GetActorTransform();
    Origin = Transform.InverseTransformPosition(Origin);
    Direction = Transform.InverseTransformVector(Direction);
    if (FMath::Abs(Direction.Y) < UE_SMALL_NUMBER) { return; }
    const double T = -Origin.Y / Direction.Y;
    if (T <= 0) { return; }
    FVector Target = Origin + Direction * T;
    const float Offset = bOverlay ? 0.0f : ViewSeparation * 0.5f;
    Target.X -= Target.X >= 0 ? Offset : -Offset;
    FireAtLocalTarget(Target);
}

void APBDClothComparisonActor::StepOnce()
{
    if (!ChaosState) { ResetComparison(); }
    if (bInvalidState) { return; }
    const float Dt = FMath::Clamp(FixedDeltaTime, 0.001f, 1.0f / 30.0f);
    const int32 Iterations = FMath::Clamp(SolverIterations, 1, 64);
    FVector Acceleration = bGravityEnabled ? Gravity : FVector::ZeroVector;
    if (bOscillatingForce)
    {
        Acceleration += ForceAcceleration * FMath::Sin(SimulationTime * 2.0 * UE_PI * 0.5);
    }
    CustomSolver.SetGravity(FVector3f(Acceleration));
    TArray<FPBDCollider> Colliders;
    for (FShot& Shot : Shots)
    {
        Shot.Age += Dt;
        Shot.Collider.Center += Shot.Velocity * Dt;
    }
    Shots.RemoveAll([](const FShot& Shot) { return Shot.Age > 6.0f; });
    for (const FShot& Shot : Shots) { Colliders.Add(Shot.Collider); }
    const float ThicknessValue = FMath::IsFinite(CollisionThickness) ? FMath::Max(0.0f, CollisionThickness) : 0.5f;
    CollisionBatch->SetThickness(ThicknessValue);
    CollisionBatch->SetColliders(Colliders);
    ChaosState->UpdateColliders(Colliders, ThicknessValue);
    CustomSolver.Step(Dt, Iterations);
    ChaosState->Step(Dt, Iterations, FVector3f(Acceleration));
    SimulationTime += Dt;
    ++StepCount;
    for (int32 I = 0; I < CustomSolver.GetParticles().Num(); ++I)
    {
        if (CustomSolver.GetParticles()[I].Position.ContainsNaN() ||
            CustomSolver.GetParticles()[I].Velocity.ContainsNaN() ||
            ChaosState->Particles.X(I).ContainsNaN() || ChaosState->Particles.V(I).ContainsNaN())
        {
            bInvalidState = true;
            bPaused = true;
            UE_LOG(LogTemp, Error, TEXT("[PBDComparison] Non-finite state at step %d particle %d; both sides paused. Reset with R."), StepCount, I);
            break;
        }
    }
}

FString APBDClothComparisonActor::GetComparisonReport() const
{
    if (!ChaosState) { return TEXT("Not initialized"); }
    double SumSquared = 0.0;
    double Maximum = 0.0;
    double CustomSpeed = 0.0;
    double ChaosSpeed = 0.0;
    double CustomPinDrift = 0.0;
    double ChaosPinDrift = 0.0;
    const TArray<FPBDParticle>& P = CustomSolver.GetParticles();
    for (int32 I = 0; I < P.Num(); ++I)
    {
        const double Error = (FVector(P[I].Position) - FVector(ChaosState->Particles.X(I))).Size();
        SumSquared += Error * Error;
        Maximum = FMath::Max(Maximum, Error);
        CustomSpeed = FMath::Max(CustomSpeed, FVector(P[I].Velocity).Size());
        ChaosSpeed = FMath::Max(ChaosSpeed, FVector(ChaosState->Particles.V(I)).Size());
        if (P[I].InvMass == 0.0f)
        {
            const FVector Rest((I % NumX - (NumX - 1) * 0.5f) * Spacing, 0.0, (NumY - 1) * Spacing * 0.5f);
            CustomPinDrift = FMath::Max(CustomPinDrift, (FVector(P[I].Position) - Rest).Size());
            ChaosPinDrift = FMath::Max(ChaosPinDrift, (FVector(ChaosState->Particles.X(I)) - Rest).Size());
        }
    }
    double CustomStretch = 0.0;
    double ChaosStretch = 0.0;
    float CustomPenetration = 0, ChaosPenetration = 0;
    for (const FShot& Shot : Shots)
    {
        for (int32 I = 0; I < P.Num(); ++I)
        {
            if (P[I].InvMass <= 0) { continue; }
            float Phi; FVector3f Normal;
            Shot.Collider.PhiWithNormal(P[I].Position, Phi, Normal);
            CustomPenetration = FMath::Max(CustomPenetration, CollisionBatch->GetThickness() - Phi);
            Shot.Collider.PhiWithNormal(FVector3f(ChaosState->Particles.X(I)), Phi, Normal);
            ChaosPenetration = FMath::Max(ChaosPenetration, CollisionBatch->GetThickness() - Phi);
        }
    }
    for (const FPBDDistanceConstraint& C : CustomSolver.GetDistanceConstraints())
    {
        const double L = C.RestLength;
        CustomStretch = FMath::Max(CustomStretch, FMath::Abs((FVector(P[C.ParticleA].Position) - FVector(P[C.ParticleB].Position)).Size() / L - 1.0));
        ChaosStretch = FMath::Max(ChaosStretch, FMath::Abs((FVector(ChaosState->Particles.X(C.ParticleA)) - FVector(ChaosState->Particles.X(C.ParticleB))).Size() / L - 1.0));
    }
    return FString::Printf(TEXT("step=%d t=%.3fs | %s | RMS=%.4f cm max=%.4f cm\nCustom / Chaos: max speed %.2f / %.2f cm/s | edge error %.2f / %.2f %% | pin drift %.6f / %.6f cm\nfinite=%s gravity=%s gust=%s | h=%.6f iterations=%d | particles=%d edges=%d hinges=%d\nDistance + Bend + Collision | penetration=%.5f / %.5f cm | active=%d fired=%d last=%s | friction=0 CCD=off"),
        StepCount, SimulationTime, bPaused ? TEXT("PAUSED") : TEXT("RUNNING"),
        FMath::Sqrt(SumSquared / P.Num()), Maximum, CustomSpeed, ChaosSpeed,
        100.0 * CustomStretch, 100.0 * ChaosStretch, CustomPinDrift, ChaosPinDrift,
        bInvalidState ? TEXT("NO") : TEXT("yes"), bGravityEnabled ? TEXT("on") : TEXT("off"),
        bOscillatingForce ? TEXT("on") : TEXT("off"), FMath::Clamp(FixedDeltaTime, 0.001f, 1.0f / 30.0f),
        FMath::Clamp(SolverIterations, 1, 64), P.Num(), CustomSolver.GetDistanceConstraints().Num(), CustomSolver.GetBendConstraints().Num(),
        CustomPenetration, ChaosPenetration, Shots.Num(), ShotsFired, *LastShape);
}

void APBDClothComparisonActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
    {
        if (PC->GetViewTarget() != this) { PC->SetViewTarget(this); }
        if (PC->WasInputKeyJustPressed(EKeys::R)) { ResetComparison(); }
        if (PC->WasInputKeyJustPressed(EKeys::P)) { bPaused = !bPaused; Accumulator = 0.0; }
        if (PC->WasInputKeyJustPressed(EKeys::SpaceBar)) { PushBoth(); }
        if (PC->WasInputKeyJustPressed(EKeys::G)) { bGravityEnabled = !bGravityEnabled; }
        if (PC->WasInputKeyJustPressed(EKeys::F)) { bOscillatingForce = !bOscillatingForce; }
        if (PC->WasInputKeyJustPressed(EKeys::O)) { bOverlay = !bOverlay; }
        if (PC->WasInputKeyJustPressed(EKeys::LeftMouseButton)) { FireFromMouse(); }
        if (bPaused && PC->WasInputKeyJustPressed(EKeys::N)) { StepOnce(); }
    }
    if (!bPaused)
    {
        // A shared accumulator keeps the two simulations on exactly the same clock.
        // Limit catch-up after a debugger/editor stall; never advance just one side.
        const float Dt = FMath::Clamp(FixedDeltaTime, 0.001f, 1.0f / 30.0f);
        Accumulator = FMath::Min(Accumulator + FMath::Max(0.0f, DeltaTime), 8.0 * Dt);
        while (Accumulator + 1.e-9 >= Dt && !bPaused)
        {
            StepOnce();
            Accumulator -= Dt;
        }
    }
    DrawComparison();
}

void APBDClothComparisonActor::DrawComparison()
{
    if (!ChaosState) { return; }
    const FTransform Transform = GetActorTransform();
    const float Offset = bOverlay ? 0.0f : ViewSeparation * 0.5f;
    const FColor Colors[] = { FColor::Cyan, FColor(255, 160, 45) };
    ProjectileSpheres->ClearInstances();
    ProjectileCylinders->ClearInstances();
    for (int32 Side = 0; Side < 2; ++Side)
    {
        const FVector Translation(Side == 0 ? Offset : -Offset, 0.0, 0.0);
        for (const FShot& Shot : Shots)
        {
            const FPBDCollider& C = Shot.Collider;
            const FVector Center = Transform.TransformPosition(FVector(C.Center) + Translation);
            const FQuat Rotation = Transform.GetRotation() * FQuat(C.Rotation);
            const float Scale = Transform.GetScale3D().X;
            const FColor Color = FColor(255, 225, 95);
            const FVector SphereScale(C.Radius * Scale / 50.0f);
            if (C.Shape == EPBDColliderShape::Sphere)
            {
                ProjectileSpheres->AddInstance(FTransform(Rotation, Center, SphereScale), true);
            }
            else
            {
                ProjectileCylinders->AddInstance(FTransform(Rotation, Center,
                    FVector(C.Radius * Scale / 50.0f, C.Radius * Scale / 50.0f, C.HalfHeight * Scale / 50.0f)), true);
                if (C.Shape == EPBDColliderShape::Capsule)
                {
                    const FVector Axis = Rotation.GetAxisZ() * C.HalfHeight * Scale;
                    ProjectileSpheres->AddInstance(FTransform(Rotation, Center - Axis, SphereScale), true);
                    ProjectileSpheres->AddInstance(FTransform(Rotation, Center + Axis, SphereScale), true);
                }
            }
            if (C.Shape == EPBDColliderShape::Sphere)
            {
                DrawDebugSphere(GetWorld(), Center, C.Radius * Scale, 24, Color, false, 0, 0, 1.5f);
            }
            else if (C.Shape == EPBDColliderShape::Capsule)
            {
                DrawDebugCapsule(GetWorld(), Center, (C.HalfHeight + C.Radius) * Scale,
                    C.Radius * Scale, Rotation, Color, false, 0, 0, 1.5f);
            }
            else
            {
                const FVector Axis = Rotation.GetAxisZ() * C.HalfHeight * Scale;
                DrawDebugCylinder(GetWorld(), Center - Axis, Center + Axis, C.Radius * Scale, 24, Color, false, 0, 0, 1.5f);
            }
        }
        const auto WorldPosition = [&](int32 I)
        {
            const FVector Local = Side == 0 ? FVector(CustomSolver.GetParticles()[I].Position) : FVector(ChaosState->Particles.X(I));
            return Transform.TransformPosition(Local + Translation);
        };
        for (const FPBDTriangle& T : CustomSolver.GetTriangles())
        {
            const FVector A = WorldPosition(T.Index1);
            const FVector B = WorldPosition(T.Index2);
            const FVector C = WorldPosition(T.Index3);
            if (A.ContainsNaN() || B.ContainsNaN() || C.ContainsNaN()) { continue; }
            DrawDebugLine(GetWorld(), A, B, Colors[Side], false, 0.0f, 0, 1.2f);
            DrawDebugLine(GetWorld(), B, C, Colors[Side], false, 0.0f, 0, 1.2f);
            DrawDebugLine(GetWorld(), C, A, Colors[Side], false, 0.0f, 0, 1.2f);
        }
        for (int32 I = 0; I < CustomSolver.GetParticles().Num(); ++I)
        {
            if (CustomSolver.GetParticles()[I].InvMass == 0.0f)
            {
                DrawDebugPoint(GetWorld(), WorldPosition(I), 14.0f, FColor::Red, false, 0.0f);
            }
        }
        const FVector Label = Transform.TransformPosition(Translation + FVector(0.0, 0.0, (NumY - 1) * Spacing * 0.5f + 20.0f));
        DrawDebugString(GetWorld(), Label, Side == 0 ? TEXT("CUSTOM") : TEXT("CHAOS PBD"), nullptr, Colors[Side], 0.0f, true);
    }
    if (GEngine)
    {
        const uint64 Key = static_cast<uint64>(GetUniqueID()) << 1;
        GEngine->AddOnScreenDebugMessage(Key, 0.0f, FColor::White,
            TEXT("LMB: fire paired random shape (aim either cloth) | P: pause | N: step | R: reset | O: overlay\nSpace: push | F: gust | G: gravity | yellow: shared kinematic projectiles\n") + GetComparisonReport());
    }
}
