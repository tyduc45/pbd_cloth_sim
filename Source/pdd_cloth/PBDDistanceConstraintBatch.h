#pragma once

#include "PBDConstraintBatch.h"

struct FPBDDistanceConstraint
{
	int32 ParticleA = INDEX_NONE;
	int32 ParticleB = INDEX_NONE;
	float RestLength = 0.0f;
	// 0 表示完全刚性；数值越大，约束越柔软。
	float Compliance = 0.0f;
	// 当前子时间步累计的 XPBD 拉格朗日乘子。
	float Lambda;
};

// FPBDDistanceConstraint 只保存数据；FPBDDistanceConstraintBatch 负责批量处理数据
class FPBDDistanceConstraintBatch final : public IPBDConstraintBatch
{
public:
	virtual FName GetDebugName() const override;
	virtual int32 GetSolvePriority() const override;

	virtual void PreStep(FPBDConstraintContext& Context) override;
	virtual void Solve(FPBDConstraintContext& Context) override;

	void Reserve(int32 ConstraintCount);
	void AddConstraint(
		int32 ParticleA,
		int32 ParticleB,
		float RestLength,
		float Compliance = 0.0f);

	const TArray<FPBDDistanceConstraint>& GetConstraints() const;
private:
	TArray<FPBDDistanceConstraint> Constraints;
};

