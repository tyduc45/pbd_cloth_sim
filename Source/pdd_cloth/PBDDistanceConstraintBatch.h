#pragma once

#include "PBDConstraintBatch.h"

struct FPBDDistanceConstraint
{
	int32 ParticleA = INDEX_NONE;
	int32 ParticleB = INDEX_NONE;
	float RestLength = 0.0f;
	// 0 ��ʾ��ȫ���ԣ���ֵԽ��Լ��Խ������
	float Compliance = 0.0f;
	// ��ǰ��ʱ�䲽�ۼƵ� XPBD �������ճ��ӡ�
	float Lambda;
};

// FPBDDistanceConstraint ֻ�������ݣ�FPBDDistanceConstraintBatch ����������������
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

