#include "PBDBendConstraintBatch.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"

DEFINE_LOG_CATEGORY_STATIC(LogPBDBendDebug, Log, All);

static TAutoConsoleVariable<int32> CVarPBDBendDebug(
    TEXT("p.PBD.BendDebug"), 1,
    TEXT("0: off; 1: first anomaly per batch; 2: every anomaly. Diagnostic only."));
static TAutoConsoleVariable<float> CVarPBDBendDebugMaxCorrection(
    TEXT("p.PBD.BendDebugMaxCorrection"), 10.0f,
    TEXT("Report single-constraint particle corrections larger than this many local cm."));
static TAutoConsoleVariable<int32> CVarPBDBendDebugBreak(
    TEXT("p.PBD.BendDebugBreak"), 0,
    TEXT("Break before an anomalous correction if a debugger is attached."));


FName FPBDBendConstraintBatch::GetDebugName() const
{
    static const FName Name(TEXT("Bend"));
    return Name;
}

int32 FPBDBendConstraintBatch::GetSolvePriority() const
{
    return 200;
}

void FPBDBendConstraintBatch::PreStep(
    FPBDConstraintContext& Context)
{
    // Lambda 只在当前子时间步的多次迭代之间保留。
    for (auto& Constraint : Constraints)
    {
        Constraint.Lambda = 0.0f;
    }
}

void FPBDBendConstraintBatch::Solve(
    FPBDConstraintContext& Context)
{
    if (Context.DeltaTime <= UE_SMALL_NUMBER)
    {
        return;
    }
    const float DeltaTimeSquared = Context.DeltaTime * Context.DeltaTime;
    for (auto& Constraint : Constraints)
    {
        if (!Context.Particles.IsValidIndex(Constraint.Particle1) ||
            !Context.Particles.IsValidIndex(Constraint.Particle2) ||
            !Context.Particles.IsValidIndex(Constraint.Particle3) ||
            !Context.Particles.IsValidIndex(Constraint.Particle4))
        {
            continue;
        }
        FPBDParticle& P1 = Context.Particles[Constraint.Particle1];
        FPBDParticle& P2 = Context.Particles[Constraint.Particle2];
        FPBDParticle& P3 = Context.Particles[Constraint.Particle3];
        FPBDParticle& P4 = Context.Particles[Constraint.Particle4];

        const FVector3f E = P2.PredictedPosition - P1.PredictedPosition;
        const FVector3f U = P3.PredictedPosition - P1.PredictedPosition;
        const FVector3f V = P4.PredictedPosition - P1.PredictedPosition;

        const FVector3f Cross1 = FVector3f::CrossProduct(E, U);
        const FVector3f Cross2 = FVector3f::CrossProduct(E, V);

        float A = Cross1.Size();
        float B = Cross2.Size();

        if (A <= UE_SMALL_NUMBER || B <= UE_SMALL_NUMBER)
        {
            continue;
        }

        FVector3f N1 = Cross1 / A;
        FVector3f N2 = Cross2 / B;

        const float CosTheta = FMath::Clamp(FVector3f::DotProduct(N1, N2), -1.0f, 1.0f);
        const float SinTheta = FMath::Sqrt(FMath::Max(0.0f, 1.0f - CosTheta * CosTheta));
        // The unsigned acos gradient is undefined at a flat or fully folded hinge.
        if (SinTheta <= UE_SMALL_NUMBER)
        {
            continue;
        }

        // 将重复的链式求导部分提出来。
        const FVector3f T1 = (N2 - CosTheta * N1) / A;
        const FVector3f T2 = (N1 - CosTheta * N2) / B;
        // 求解约束的梯度
        const FVector3f Grad3 = -FVector3f::CrossProduct(T1, E) / SinTheta;
        const FVector3f Grad4 = -FVector3f::CrossProduct(T2, E) / SinTheta;
        const FVector3f Grad2 = -(FVector3f::CrossProduct(U, T1) + FVector3f::CrossProduct(V, T2)) / SinTheta;
        const FVector3f Grad1 = -Grad2 - Grad3 - Grad4;
        // 求分母上的二次型
        const float WeightSum =
            P1.InvMass * Grad1.SizeSquared() +
            P2.InvMass * Grad2.SizeSquared() +
            P3.InvMass * Grad3.SizeSquared() +
            P4.InvMass * Grad4.SizeSquared();
        // 计算分母和分子
        float Alpha = Constraint.Compliance / DeltaTimeSquared;
        const float Denominator = WeightSum + Alpha;
        float ConstraintError = FMath::Acos(CosTheta) - Constraint.theta;
        if (Denominator <= 0.0f)
        {
            continue;
        }
        // 计算delta lambda 并更新 x
        const float DeltaLambda = (-ConstraintError - Alpha * Constraint.Lambda) / Denominator;
        const FVector3f DeltaP1 = P1.InvMass * DeltaLambda * Grad1;
        const FVector3f DeltaP2 = P2.InvMass * DeltaLambda * Grad2;
        const FVector3f DeltaP3 = P3.InvMass * DeltaLambda * Grad3;
        const FVector3f DeltaP4 = P4.InvMass * DeltaLambda * Grad4;

        const int32 DebugMode = CVarPBDBendDebug.GetValueOnGameThread();
        if (DebugMode > 0)
        {
            const FVector3f Positions[] = { P1.PredictedPosition, P2.PredictedPosition,
                P3.PredictedPosition, P4.PredictedPosition };
            const FVector3f Gradients[] = { Grad1, Grad2, Grad3, Grad4 };
            const FVector3f Corrections[] = { DeltaP1, DeltaP2, DeltaP3, DeltaP4 };
            const float InvMasses[] = { P1.InvMass, P2.InvMass, P3.InvMass, P4.InvMass };
            const int32 Indices[] = { Constraint.Particle1, Constraint.Particle2,
                Constraint.Particle3, Constraint.Particle4 };
            const float Scalars[] = { A, B, CosTheta, SinTheta, ConstraintError,
                Constraint.theta, WeightSum, Alpha, Denominator, Constraint.Lambda,
                DeltaLambda, Constraint.Lambda + DeltaLambda };
            bool bFinite = true;
            for (float Value : Scalars)
            {
                bFinite &= FMath::IsFinite(Value);
            }
            double MaxCorrection = 0.0;
            for (int32 I = 0; I < 4; ++I)
            {
                bFinite &= FMath::IsFinite(InvMasses[I]) && !Positions[I].ContainsNaN()
                    && !Gradients[I].ContainsNaN() && !Corrections[I].ContainsNaN()
                    && !(Positions[I] + Corrections[I]).ContainsNaN();
                // Measure in double so a large finite float vector does not overflow SizeSquared.
                MaxCorrection = FMath::Max(MaxCorrection, FVector(Corrections[I]).Size());
            }
            const float Threshold = CVarPBDBendDebugMaxCorrection.GetValueOnGameThread();
            if ((!bFinite || MaxCorrection > Threshold) &&
                (DebugMode >= 2 || !bHasReportedDebugAnomaly))
            {
                bHasReportedDebugAnomaly = true;
                const int32 ConstraintIndex = static_cast<int32>(&Constraint - Constraints.GetData());
                UE_LOG(LogPBDBendDebug, Warning,
                    TEXT("BEFORE WRITE Batch=%p Iteration=%d Constraint=%d Dt=%.9g NonFinite=%d MaxDeltaP=%.9g cm Threshold=%.9g"),
                    static_cast<const void*>(this), Context.Iteration, ConstraintIndex,
                    Context.DeltaTime, !bFinite, MaxCorrection, Threshold);
                UE_LOG(LogPBDBendDebug, Warning,
                    TEXT("A=%.9g B=%.9g Cos=%.9g Sin=%.9g Angle=%.9g RestAngle=%.9g C=%.9g WeightSum=%.9g Alpha=%.9g Denom=%.9g LambdaBefore=%.9g DeltaLambda=%.9g"),
                    A, B, CosTheta, SinTheta, FMath::Acos(CosTheta), Constraint.theta,
                    ConstraintError, WeightSum, Alpha, Denominator, Constraint.Lambda, DeltaLambda);
                for (int32 I = 0; I < 4; ++I)
                {
                    UE_LOG(LogPBDBendDebug, Warning,
                        TEXT("P%d Index=%d InvMass=%.9g Before=(%.9g,%.9g,%.9g) Grad=(%.9g,%.9g,%.9g) DeltaP=(%.9g,%.9g,%.9g) Length=%.9g"),
                        I + 1, Indices[I], InvMasses[I], Positions[I].X, Positions[I].Y, Positions[I].Z,
                        Gradients[I].X, Gradients[I].Y, Gradients[I].Z,
                        Corrections[I].X, Corrections[I].Y, Corrections[I].Z, FVector(Corrections[I]).Size());
                }
                if (CVarPBDBendDebugBreak.GetValueOnGameThread() != 0 && FPlatformMisc::IsDebuggerPresent())
                {
                    UE_DEBUG_BREAK();
                }
            }
        }

        // Diagnostics do not clamp or skip corrections: preserve the failure for investigation.
        Constraint.Lambda += DeltaLambda;
        P1.PredictedPosition += DeltaP1;
        P2.PredictedPosition += DeltaP2;
        P3.PredictedPosition += DeltaP3;
        P4.PredictedPosition += DeltaP4;
    }
}

void FPBDBendConstraintBatch::Reserve(
    int32 ConstraintCount)
{
    Constraints.Reserve(ConstraintCount);
}

void FPBDBendConstraintBatch::AddConstraint(
    int32 Particle1,
    int32 Particle2,
    int32 Particle3,
    int32 Particle4,
    float theta0,
    float Compliance)
{
    FPBDBendConstraint& Constraint =
        Constraints.AddDefaulted_GetRef();

    Constraint.Particle1 = Particle1;
    Constraint.Particle2 = Particle2;
    Constraint.Particle3 = Particle3;
    Constraint.Particle4 = Particle4;
    Constraint.theta = theta0;
    Constraint.Compliance = FMath::Max(0.0f, Compliance);
}

const TArray<FPBDBendConstraint>&
FPBDBendConstraintBatch::GetConstraints() const
{
    return Constraints;
}