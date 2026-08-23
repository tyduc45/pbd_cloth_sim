#pragma once

#include "PBDClothTypes.h"

class FPBDClothSolver
{
public:
	// 初始化网格
	void InitializeGrid(int32 NumX, int32 NumY, float Spacing);
	// 迭代
	void Step(float DeltaTime);
	void Reset();

	const TArray<FPBDParticle>& GetParticles() const;
	const TArray<FPBDConstraint>& GetConstraint() const;

private:
	// 计算时间积分
	void Integreate(float DeltaTime);
	void SolveDistanceConstraints();
	void UpdateVelocities(float DeltaTime);

	TArray<FPBDParticle> Particles;
	TArray<FPBDConstraint> DistanceConstraints;
};