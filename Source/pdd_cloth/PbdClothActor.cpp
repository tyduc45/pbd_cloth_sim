// Fill out your copyright notice in the Description page of Project Settings.


#include "PbdClothActor.h"

// Sets default values
APbdClothActor::APbdClothActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void APbdClothActor::BeginPlay()
{
	Super::BeginPlay();
	Solver.InitializeGrid(NumX, NumY, Spacing);
}

// Called every frame
void APbdClothActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	const TArray<FPBDParticle>& particles = Solver.GetParticles();
	const TArray<FPBDTriangle>& triangles = Solver.GetTriangles();

	const FTransform ActorTransform = GetActorTransform();
	//粒子本地转世界坐标
	auto GenWorldPosition = [&particles, &ActorTransform](int32 particleIndex) -> FVector
		{
			const FVector3f& LocalPosition = particles[particleIndex].Position;
			return ActorTransform.TransformPosition(
				FVector(
					LocalPosition.X,
					LocalPosition.Y,
					LocalPosition.Z
				)
			);
		};
	// 对于每一个三角形，搞到他世界坐标之后，画线连起来
	for (const auto& Triangle : triangles)
	{
		// 防止错误索引导致崩溃
		if (!particles.IsValidIndex(Triangle.Index1) ||
			!particles.IsValidIndex(Triangle.Index2) ||
			!particles.IsValidIndex(Triangle.Index3))
		{
			continue;
		}
		FVector World1 = GenWorldPosition(Triangle.Index1);
		FVector World2 = GenWorldPosition(Triangle.Index2);
		FVector World3 = GenWorldPosition(Triangle.Index3);
		// false, 0.0f 表示线条不永久保存，只显示当前帧。
		DrawDebugLine(GetWorld(), World1, World2, FColor::Cyan, false, 0.0f, 0, 1.5f);
		DrawDebugLine(GetWorld(), World2, World3, FColor::Cyan, false, 0.0f, 0, 1.5f);
		DrawDebugLine(GetWorld(), World1, World3, FColor::Cyan, false, 0.0f, 0, 1.5f);
	}
	// 绘制粒子（顶点）
	for (const auto& Particle : particles)
	{
		const FVector3f& LocalPosition = Particle.Position;
		const FVector WorldPosition =
			ActorTransform.TransformPosition(
				FVector(
					LocalPosition.X,
					LocalPosition.Y,
					LocalPosition.Z
				)
			);
		// 区分固定点以及动点
		const FColor PointColor =
			Particle.InvMass == 0.0f
			? FColor::Red
			: FColor::White;
		DrawDebugPoint(GetWorld(), WorldPosition, 8.0f, PointColor, false, 0.0f, 0);
	}
}

