#include "PBDClothComparisonActor.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Chaos/PBDSoftsSolverParticles.h"
#include "Chaos/PBDSpringConstraints.h"
#include "Chaos/PBDBendingConstraints.h"

class FChaosClothComparisonState
{
public:
    Chaos::Softs::FSolverParticles Particles;
    TUniquePtr<Chaos::Softs::FPBDSpringConstraints> Distance;
    TUniquePtr<Chaos::Softs::FPBDBendingConstraints> Bend;

    void Initialize(const FPBDClothSolver& Source)
    {
        using namespace Chaos;
        using namespace Chaos::Softs;
        const TArray<FPBDParticle>& SourceParticles = Source.GetParticles();
        Particles.AddParticles(SourceParticles.Num());
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
        // Stiffness=1 matches our Compliance=0 projection. No tether, collision,
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
        for (int32 Iteration = 0; Iteration < Iterations; ++Iteration)
        {
            Distance->Apply(Particles, Dt);
            Bend->Apply(Particles, Dt);
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
}

APBDClothComparisonActor::~APBDClothComparisonActor() = default;

void APBDClothComparisonActor::BeginPlay()
{
    Super::BeginPlay();
    ResetComparison();
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
    {
        PC->SetViewTarget(this);
        PC->SetInputMode(FInputModeGameOnly());
    }
}

void APBDClothComparisonActor::ResetComparison()
{
    NumX = FMath::Clamp(NumX, 2, 64);
    NumY = FMath::Clamp(NumY, 2, 64);
    Spacing = FMath::Clamp(Spacing, 0.1f, 1000.0f);
    CustomSolver.InitializeGrid(NumX, NumY, Spacing);
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
    UE_LOG(LogTemp, Display, TEXT("[PBDComparison] Reset: %d particles; %d distance; %d bend per side. Custom Compliance=0; Chaos PBD stiffness=1; angle cap=45deg; only these two constraint types."),
        CustomSolver.GetParticles().Num(), CustomSolver.GetDistanceConstraints().Num(), CustomSolver.GetBendConstraints().Num());
}

void APBDClothComparisonActor::PushBoth()
{
    if (!ChaosState || bInvalidState) { return; }
    const FVector3f DeltaVelocity(PushVelocity);
    CustomSolver.AddVelocity(DeltaVelocity);
    ChaosState->AddVelocity(DeltaVelocity);
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
    for (const FPBDDistanceConstraint& C : CustomSolver.GetDistanceConstraints())
    {
        const double L = C.RestLength;
        CustomStretch = FMath::Max(CustomStretch, FMath::Abs((FVector(P[C.ParticleA].Position) - FVector(P[C.ParticleB].Position)).Size() / L - 1.0));
        ChaosStretch = FMath::Max(ChaosStretch, FMath::Abs((FVector(ChaosState->Particles.X(C.ParticleA)) - FVector(ChaosState->Particles.X(C.ParticleB))).Size() / L - 1.0));
    }
    return FString::Printf(TEXT("step=%d t=%.3fs | %s | RMS=%.4f cm max=%.4f cm\nCustom / Chaos: max speed %.2f / %.2f cm/s | edge error %.2f / %.2f %% | pin drift %.6f / %.6f cm\nfinite=%s gravity=%s gust=%s | h=%.6f iterations=%d | particles=%d edges=%d hinges=%d"),
        StepCount, SimulationTime, bPaused ? TEXT("PAUSED") : TEXT("RUNNING"),
        FMath::Sqrt(SumSquared / P.Num()), Maximum, CustomSpeed, ChaosSpeed,
        100.0 * CustomStretch, 100.0 * ChaosStretch, CustomPinDrift, ChaosPinDrift,
        bInvalidState ? TEXT("NO") : TEXT("yes"), bGravityEnabled ? TEXT("on") : TEXT("off"),
        bOscillatingForce ? TEXT("on") : TEXT("off"), FMath::Clamp(FixedDeltaTime, 0.001f, 1.0f / 30.0f),
        FMath::Clamp(SolverIterations, 1, 64), P.Num(), CustomSolver.GetDistanceConstraints().Num(), CustomSolver.GetBendConstraints().Num());
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
    for (int32 Side = 0; Side < 2; ++Side)
    {
        const FVector Translation(Side == 0 ? Offset : -Offset, 0.0, 0.0);
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
            TEXT("Space: push both | F: oscillating force | G: gravity | P: pause | N: step | R: reset | O: overlay\n") + GetComparisonReport());
    }
}
