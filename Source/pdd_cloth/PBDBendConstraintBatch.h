#pragma once

#include "PBDConstraintBatch.h"

struct FPBDBendConstraint
{
	int32 Particle1 = INDEX_NONE;
	int32 Particle2 = INDEX_NONE;
	int32 Particle3 = INDEX_NONE;
	int32 Particle4 = INDEX_NONE;
	float theta = 0.0f;
	// 0 表示完全刚性；数值越大，约束越柔软。
	float Compliance = 0.0f;
	// 当前子时间步累计的 XPBD 拉格朗日乘子。
	float Lambda = 0.0f;
};

// FPBDDistanceConstraint 只保存数据；FPBDDistanceConstraintBatch 负责批量处理数据
class FPBDBendConstraintBatch final : public IPBDConstraintBatch
{
public:
	virtual FName GetDebugName() const override;
	virtual int32 GetSolvePriority() const override;

	virtual void PreStep(FPBDConstraintContext& Context) override;
	virtual void Solve(FPBDConstraintContext& Context) override;

	void Reserve(int32 ConstraintCount);
	void AddConstraint(
		int32 Particle1,
		int32 Particle2,
		int32 Particle3,
		int32 Particle4,
		float theta0,
		float Compliance = 0.0f);

	const TArray<FPBDBendConstraint>& GetConstraints() const;
private:
    bool bHasReportedDebugAnomaly = false;
	TArray<FPBDBendConstraint> Constraints;
};

