#include "PBDClothSolver.h"
#include "PBDBendConstraintBatch.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<int32> CVarPBDStageDebug(
    TEXT("p.PBD.StageDebug"), 0,
    TEXT("Log integration displacement, each batch correction, and final speed on every step."));

void FPBDClothSolver::InitializeGrid(
    int32 NumX,
    int32 NumY,
    float Spacing)
{
    Particles.Reset();
    Triangles.Reset();
    ConstraintBatches.Reset();
    DistanceConstraintBatch = nullptr;

    Particles.Reserve(NumX * NumY);

    const float HalfWidth =
        (NumX - 1) * Spacing * 0.5f;

    const float HalfHeight =
        (NumY - 1) * Spacing * 0.5f;

    for (int32 Row = 0; Row < NumY; ++Row)
    {
        for (int32 Col = 0; Col < NumX; ++Col)
        {
            const float X =
                Col * Spacing - HalfWidth;

            const float Z =
                HalfHeight - Row * Spacing;

            FPBDParticle Particle(
                FVector3f(X, 0.0f, Z),
                1.0f);

            if (Row == 0 &&
                (Col == 0 || Col == NumX - 1))
            {
                Particle.InvMass = 0.0f;
            }

            Particles.Add(Particle);
        }
    }

    const int32 ExpectedConstraintCount =
        (NumX - 1) * NumY +
        NumX * (NumY - 1);

    TUniquePtr<FPBDDistanceConstraintBatch>
        NewDistanceBatch =
        MakeUnique<FPBDDistanceConstraintBatch>();

    DistanceConstraintBatch =
        NewDistanceBatch.Get();

    DistanceConstraintBatch->Reserve(
        ExpectedConstraintCount);

    for (int32 Row = 0; Row < NumY; ++Row)
    {
        for (int32 Col = 0; Col < NumX; ++Col)
        {
            const int32 CurrentIndex =
                Row * NumX + Col;

            if (Col + 1 < NumX)
            {
                const int32 NeighborIndex =
                    CurrentIndex + 1;

                DistanceConstraintBatch->AddConstraint(
                    CurrentIndex,
                    NeighborIndex,
                    Spacing);
            }

            if (Row + 1 < NumY)
            {
                const int32 NeighborIndex =
                    CurrentIndex + NumX;

                DistanceConstraintBatch->AddConstraint(
                    CurrentIndex,
                    NeighborIndex,
                    Spacing);
            }
        }
    }

    RegisterConstraintBatch(
        MoveTemp(NewDistanceBatch));

    const int32 ExpectedTriangleCount =
        2 * (NumX - 1) * (NumY - 1);

    Triangles.Reserve(ExpectedTriangleCount);

    for (int32 Row = 0; Row < NumY - 1; ++Row)
    {
        for (int32 Col = 0; Col < NumX - 1; ++Col)
        {
            const int32 A = Row * NumX + Col;
            const int32 B = A + 1;
            const int32 C = A + NumX;
            const int32 D = C + 1;

            FPBDTriangle TriangleABC;
            TriangleABC.Index1 = A;
            TriangleABC.Index2 = B;
            TriangleABC.Index3 = C;

            FPBDTriangle TriangleBDC;
            TriangleBDC.Index1 = B;
            TriangleBDC.Index2 = D;
            TriangleBDC.Index3 = C;

            Triangles.Add(TriangleABC);
            Triangles.Add(TriangleBDC);
        }
    }

    // Each interior edge contributes one hinge: two edge endpoints and
    // the opposite vertex from each of its two triangles.
    TUniquePtr<FPBDBendConstraintBatch> NewBendBatch =
        MakeUnique<FPBDBendConstraintBatch>();
    NewBendBatch->Reserve(Triangles.Num() * 3 / 2);

    struct FEdgeNeighbors
    {
        int32 Opposite1 = INDEX_NONE;
        int32 Opposite2 = INDEX_NONE;
        int32 TriangleCount = 0;
    };

    TMap<TPair<int32, int32>, FEdgeNeighbors> EdgeNeighbors;
    const auto AddEdge = [&EdgeNeighbors](int32 A, int32 B, int32 Opposite)
    {
        const TPair<int32, int32> Key(FMath::Min(A, B), FMath::Max(A, B));
        FEdgeNeighbors& Neighbors = EdgeNeighbors.FindOrAdd(Key);
        if (Neighbors.TriangleCount == 0)
        {
            Neighbors.Opposite1 = Opposite;
        }
        else if (Neighbors.TriangleCount == 1)
        {
            Neighbors.Opposite2 = Opposite;
        }
        ++Neighbors.TriangleCount;
    };

    for (const FPBDTriangle& Triangle : Triangles)
    {
        AddEdge(Triangle.Index1, Triangle.Index2, Triangle.Index3);
        AddEdge(Triangle.Index2, Triangle.Index3, Triangle.Index1);
        AddEdge(Triangle.Index3, Triangle.Index1, Triangle.Index2);
    }

    for (const auto& Entry : EdgeNeighbors)
    {
        const FEdgeNeighbors& Neighbors = Entry.Value;
        // Boundary edges have no second face. Non-manifold edges need an
        // explicit pairing policy; they cannot occur in this regular grid.
        if (Neighbors.TriangleCount != 2)
        {
            continue;
        }

        const int32 P1 = Entry.Key.Key;
        const int32 P2 = Entry.Key.Value;
        const int32 P3 = Neighbors.Opposite1;
        const int32 P4 = Neighbors.Opposite2;
        const FVector3f E = Particles[P2].Position - Particles[P1].Position;
        const FVector3f U = Particles[P3].Position - Particles[P1].Position;
        const FVector3f V = Particles[P4].Position - Particles[P1].Position;
        const FVector3f Cross1 = FVector3f::CrossProduct(E, U);
        const FVector3f Cross2 = FVector3f::CrossProduct(V, E);
        const float A = Cross1.Size();
        const float B = Cross2.Size();
        const float EdgeLength = E.Size();
        if (!FMath::IsFinite(A) || !FMath::IsFinite(B) || !FMath::IsFinite(EdgeLength) ||
            A <= UE_SMALL_NUMBER || B <= UE_SMALL_NUMBER || EdgeLength <= UE_SMALL_NUMBER)
        {
            continue;
        }

        // Match Solve's signed atan2 convention: a flat hinge has angle zero.
        const FVector3f N1 = Cross1 / A;
        const FVector3f N2 = Cross2 / B;
        const float CosTheta = FMath::Clamp(FVector3f::DotProduct(N1, N2), -1.0f, 1.0f);
        const float SinTheta = FMath::Clamp(FVector3f::DotProduct(
            FVector3f::CrossProduct(N2, N1), E / EdgeLength), -1.0f, 1.0f);
        NewBendBatch->AddConstraint(P1, P2, P3, P4, FMath::Atan2(SinTheta, CosTheta));
    }

    const int32 BendConstraintCount = NewBendBatch->GetConstraints().Num();
    RegisterConstraintBatch(MoveTemp(NewBendBatch));

    UE_LOG(
        LogTemp,
        Display,
        TEXT(
            "PBD grid initialized: "
            "%d particles, "
            "%d distance constraints, "
            "%d bend constraints, "
            "%d triangles"),
        Particles.Num(),
        DistanceConstraintBatch
        ->GetConstraints()
        .Num(),
        BendConstraintCount,
        Triangles.Num());
}

void FPBDClothSolver::Step(
    float DeltaTime,
    int32 SolverIterations)
{
    if (DeltaTime <= UE_SMALL_NUMBER ||
        Particles.IsEmpty())
    {
        return;
    }

    const bool bDebugStages = CVarPBDStageDebug.GetValueOnGameThread() != 0;
    TArray<FVector3f> BeforeBatch;
    if (bDebugStages)
    {
        BeforeBatch.SetNumUninitialized(Particles.Num());
    }
    const auto LogStage = [&](const TCHAR* Stage, int32 Iteration, bool bVelocity, bool bBatchDelta)
    {
        double Maximum = 0.0;
        int32 MaxParticle = INDEX_NONE;
        int32 FirstNonFinite = INDEX_NONE;
        int32 NonFiniteCount = 0;
        for (int32 I = 0; I < Particles.Num(); ++I)
        {
            const FVector Value = bVelocity ? FVector(Particles[I].Velocity) :
                FVector(Particles[I].PredictedPosition) -
                FVector(bBatchDelta ? BeforeBatch[I] : Particles[I].Position);
            if (Value.ContainsNaN())
            {
                if (FirstNonFinite == INDEX_NONE) { FirstNonFinite = I; }
                ++NonFiniteCount;
                continue;
            }
            const double Magnitude = Value.Size();
            if (MaxParticle == INDEX_NONE || Magnitude > Maximum)
            {
                Maximum = Magnitude;
                MaxParticle = I;
            }
        }
        UE_LOG(LogTemp, Warning,
            TEXT("[PBDStage] Solver=%p Frame=%llu Dt=%.9g Iteration=%d Stage=%s MaxFinite=%.9g %s Particle=%d NonFiniteCount=%d FirstNonFinite=%d"),
            static_cast<const void*>(this), static_cast<unsigned long long>(GFrameCounter),
            DeltaTime, Iteration, Stage, Maximum, bVelocity ? TEXT("cm/s") : TEXT("cm"),
            MaxParticle, NonFiniteCount, FirstNonFinite);
    };

    Integrate(DeltaTime);
    if (bDebugStages)
    {
        LogStage(TEXT("Integrate"), INDEX_NONE, false, false);
    }

    FPBDConstraintContext Context;
    Context.Particles = MakeArrayView(Particles);
    Context.DeltaTime = DeltaTime;

    for (TUniquePtr<IPBDConstraintBatch>& Batch :
        ConstraintBatches)
    {
        Batch->PreStep(Context);
    }
    // 进行多次迭代修正，在Integrate（）得出的的基础上（这是方程简化的重要条件）
    for (int32 Iteration = 0;
        Iteration < SolverIterations;
        ++Iteration)
    {
        Context.Iteration = Iteration;
        //按顺序求解每一个constraint
        for (TUniquePtr<IPBDConstraintBatch>& Batch :
            ConstraintBatches)
        {
            if (bDebugStages)
            {
                for (int32 I = 0; I < Particles.Num(); ++I)
                {
                    BeforeBatch[I] = Particles[I].PredictedPosition;
                }
            }
            Batch->Solve(Context);
            if (bDebugStages)
            {
                LogStage(*Batch->GetDebugName().ToString(), Iteration, false, true);
            }
        }
    }

    for (TUniquePtr<IPBDConstraintBatch>& Batch :
        ConstraintBatches)
    {
        Batch->PostStep(Context);
    }

    UpdateVelocities(DeltaTime);
    if (bDebugStages)
    {
        LogStage(TEXT("UpdateVelocities"), INDEX_NONE, true, false);
    }
}

void FPBDClothSolver::RegisterConstraintBatch(
    TUniquePtr<IPBDConstraintBatch> Batch)
{
    if (!Batch)
    {
        return;
    }

    ConstraintBatches.Add(MoveTemp(Batch));

    ConstraintBatches.StableSort(
        [](
            const TUniquePtr<IPBDConstraintBatch>& Left,
            const TUniquePtr<IPBDConstraintBatch>& Right)
        {
            return
                Left->GetSolvePriority() <
                Right->GetSolvePriority();
        });
}
// 算出初始预测位置
void FPBDClothSolver::Integrate(float DeltaTime)
{
    for (FPBDParticle& Particle : Particles)
    {
        if (Particle.InvMass <= 0.0f)
        {
            Particle.Velocity =
                FVector3f::ZeroVector;

            Particle.PredictedPosition =
                Particle.Position;

            continue;
        }

        Particle.Velocity +=
            Gravity * DeltaTime;

        Particle.PredictedPosition =
            Particle.Position +
            Particle.Velocity * DeltaTime;
    }
}

// 使用修正后的位置修正速度
void FPBDClothSolver::UpdateVelocities(
    float DeltaTime)
{
    const float InverseDeltaTime =
        1.0f / DeltaTime;

    for (FPBDParticle& Particle : Particles)
    {
        if (Particle.InvMass <= 0.0f)
        {
            Particle.Velocity =
                FVector3f::ZeroVector;

            Particle.PredictedPosition =
                Particle.Position;

            continue;
        }

        Particle.Velocity =
            (Particle.PredictedPosition -
                Particle.Position) *
            InverseDeltaTime;

        Particle.Position =
            Particle.PredictedPosition;
    }
}

void FPBDClothSolver::Reset()
{
    Particles.Reset();
    Triangles.Reset();
    ConstraintBatches.Reset();
    DistanceConstraintBatch = nullptr;
}

const TArray<FPBDParticle>&
FPBDClothSolver::GetParticles() const
{
    return Particles;
}

const TArray<FPBDTriangle>&
FPBDClothSolver::GetTriangles() const
{
    return Triangles;
}

const TArray<FPBDDistanceConstraint>&
FPBDClothSolver::GetDistanceConstraints() const
{
    static const TArray<FPBDDistanceConstraint>
        EmptyConstraints;

    return DistanceConstraintBatch
        ? DistanceConstraintBatch->GetConstraints()
        : EmptyConstraints;
}
