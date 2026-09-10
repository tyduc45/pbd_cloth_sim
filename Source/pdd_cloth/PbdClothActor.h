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
	//������������
	UPROPERTY(EditAnywhere, Category = "PBD | Grid",
		meta = (ClampMin = "2" , UIMin = "2" , UIMax = "128"))
	int32 NumX = 16;
	// ������������
	UPROPERTY(EditAnywhere, Category = "PBD | Grid",
		meta = (ClampMin = "2", UIMin = "2", UIMax = "128"))
	int32 NumY = 16;

	// �������Ӿ��루cm��
	UPROPERTY(EditAnywhere, Category = "PBD | Grid",
		meta = (ClampMin = "0.1", UIMin = "1.0", UIMax = "100.0"))
	float Spacing = 10.0f;

	// ÿ��ʱ�䲽��ͶӰԼ���Ĵ���,ͶӰ��������������˼
	UPROPERTY(EditAnywhere, Category = "PBD | Simulation",
		meta = (ClampMin = "1", UIMin = "1", UIMax = "32"))
	int32 SolverIterations = 8;

	// ģ��ʹ�õĹ̶�ʱ�䲽
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
