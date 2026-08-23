// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PBDClothSolver.h"
#include "DrawDebugHelpers.h"
#include "PbdClothActor.generated.h"

UCLASS()
class PDD_CLOTH_API APbdClothActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	APbdClothActor();

protected:
	//横向例子数量
	UPROPERTY(EditAnywhere, Category = "PBD | Grid",
		meta = (ClampMin = "2" , UIMin = "2" , UIMax = "128"))
	int32 NumX = 16;
	// 纵向粒子数量
	UPROPERTY(EditAnywhere, Category = "PBD | Grid",
		meta = (ClampMin = "2", UIMin = "2", UIMax = "128"))
	int32 NumY = 16;

	// 相邻粒子距离（cm）
	UPROPERTY(EditAnywhere, Category = "PBD | Grid",
		meta = (ClampMin = "0.1", UIMin = "1.0", UIMax = "100.0"))
	float Spacing = 10.0f;

	// 每个时间步中投影约束的次数,投影就是算修正的意思
	UPROPERTY(EditAnywhere, Category = "PBD | Simulation",
		meta = (ClampMin = "1", UIMin = "1", UIMax = "32"))
	int32 SolverIterations = 8;

	// 模拟使用的固定时间步
	UPROPERTY(EditAnywhere, Category = "PBD | Simulation",
		meta = (ClampMin = "0.0001"))
	float FixedDeltaTime = 1.0f / 60.0f;
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
private:
	FPBDClothSolver Solver;
};
