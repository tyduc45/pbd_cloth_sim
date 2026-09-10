#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PBDClothSolver.h"
#include "Templates/PimplPtr.h"
#include "PBDClothComparisonActor.generated.h"

class FChaosClothComparisonState;
class UCameraComponent;

// Two independent particle systems with identical inputs. The reference invokes
// official Chaos PBD distance/bending classes, not a second copy of our formulas.
UCLASS()
class PDD_CLOTH_API APBDClothComparisonActor : public AActor
{
    GENERATED_BODY()

public:
    APBDClothComparisonActor();
    virtual ~APBDClothComparisonActor() override;
    virtual void Tick(float DeltaTime) override;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Comparison")
    void ResetComparison();

    UFUNCTION(BlueprintCallable, Category = "Comparison")
    void PushBoth();

    UFUNCTION(BlueprintCallable, Category = "Comparison")
    void StepOnce();

    UFUNCTION(BlueprintCallable, Category = "Comparison")
    FString GetComparisonReport() const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Grid", meta = (ClampMin = "2", ClampMax = "64"))
    int32 NumX = 16;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Grid", meta = (ClampMin = "2", ClampMax = "64"))
    int32 NumY = 16;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Grid", meta = (ClampMin = "0.1"))
    float Spacing = 10.0f;

    // Changing grid properties requires ResetComparison. Time/force settings affect both sides immediately.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Simulation", meta = (ClampMin = "0.001", ClampMax = "0.033333"))
    float FixedDeltaTime = 1.0f / 60.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Simulation", meta = (ClampMin = "1", ClampMax = "64"))
    int32 SolverIterations = 8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Simulation")
    FVector Gravity = FVector(0.0, 0.0, -980.0);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Simulation")
    bool bGravityEnabled = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Simulation")
    bool bPaused = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Forces")
    FVector PushVelocity = FVector(0.0, 150.0, 0.0);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Forces")
    bool bOscillatingForce = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|Forces")
    FVector ForceAcceleration = FVector(0.0, 150.0, 0.0);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comparison|View")
    bool bOverlay = false;

    UPROPERTY(VisibleAnywhere, Category = "Comparison|View")
    TObjectPtr<UCameraComponent> ComparisonCamera;

protected:
    virtual void BeginPlay() override;

private:
    void DrawComparison();
    FPBDClothSolver CustomSolver;
    TPimplPtr<FChaosClothComparisonState> ChaosState;
    double Accumulator = 0.0;
    double SimulationTime = 0.0;
    int32 StepCount = 0;
    float ViewSeparation = 230.0f;
    bool bInvalidState = false;
};
