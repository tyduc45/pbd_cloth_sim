#include "PBDClothSolver.h"

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

    UE_LOG(
        LogTemp,
        Display,
        TEXT(
            "PBD grid initialized: "
            "%d particles, "
            "%d constraints, "
            "%d triangles"),
        Particles.Num(),
        DistanceConstraintBatch
        ->GetConstraints()
        .Num(),
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

    Integrate(DeltaTime);

    FPBDConstraintContext Context;
    Context.Particles = MakeArrayView(Particles);
    Context.DeltaTime = DeltaTime;

    for (TUniquePtr<IPBDConstraintBatch>& Batch :
        ConstraintBatches)
    {
        Batch->PreStep(Context);
    }

    for (int32 Iteration = 0;
        Iteration < SolverIterations;
        ++Iteration)
    {
        Context.Iteration = Iteration;

        for (TUniquePtr<IPBDConstraintBatch>& Batch :
            ConstraintBatches)
        {
            Batch->Solve(Context);
        }
    }

    for (TUniquePtr<IPBDConstraintBatch>& Batch :
        ConstraintBatches)
    {
        Batch->PostStep(Context);
    }

    UpdateVelocities(DeltaTime);
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