#include "PBDClothSolver.h"


void FPBDClothSolver::InitializeGrid(
	int32 NumX,
	int32 NumY,
	float Spacing)
{
	Particles.Reset();
	Triangles.Reset();
	DistanceConstraints.Reset();
	Particles.Reserve(NumX * NumY);
	// 设置粒子初始位置
	for (int row = 0; row < NumY; row++)
	{
		for (int col = 0; col < NumX; col++)
		{
			float X = col * Spacing;
			float Z = -row * Spacing;
			FVector3f pos = FVector3f(X, 0, Z);
			FPBDParticle particle(pos, 1.0f);
			if (row == 0 && (col == 0 || col == NumX - 1))
			{
				particle.InvMass = 0.0f;
			}
			Particles.Add(particle);
		}
	}
	// 存储粒子索引
	const int32 ExpectedConstraintCount =
		(NumX - 1) * NumY +
		NumX * (NumY - 1);

	DistanceConstraints.Reserve(ExpectedConstraintCount);
	for (int32 row = 0; row < NumY; row++)
	{
		for (int32 col = 0; col < NumX; col++)
		{
			const int32 CurrentIndex = row * NumX + col;
			int32 NeightborIndex;
			//建立 CurrentIndex 到 CurrentIndex + 1 的水平约束
			if (col + 1 < NumX)
			{
				FPBDConstraint HorizontalConstraint;
				NeightborIndex = CurrentIndex + 1;
				HorizontalConstraint.pA = CurrentIndex;
				HorizontalConstraint.pB = NeightborIndex;
				HorizontalConstraint.restLength = Spacing;
				DistanceConstraints.Add(HorizontalConstraint);
			}
			//建立 CurrentIndex 到 CurrentIndex + NumX 的垂直约束
			if (row + 1 < NumY)
			{
				FPBDConstraint VerticalConstraint;
				NeightborIndex = CurrentIndex + NumX;
				VerticalConstraint.pA = CurrentIndex;
				VerticalConstraint.pB = NeightborIndex;
				VerticalConstraint.restLength = Spacing;
				DistanceConstraints.Add(VerticalConstraint);
			}
		}
	}
	const int32 ExpectedTriangleCount = 2 * (NumX - 1) * (NumY - 1);
	Triangles.Reserve(ExpectedTriangleCount);
	// 初始化三角形 , 上下三角形, 顺序 ，顺时针
	for (int32 row = 0; row < NumY - 1; row++)
	{
		for (int32 col = 0; col < NumX - 1; col++)
		{
			const int32 A = row * NumX + col;
			const int32 B = A + 1;
			const int32 C = A + NumX;
			const int32 D = C + 1;
			FPBDTriangle TriABC; FPBDTriangle TriBDC;
			TriABC.Index1 = A;		 TriBDC.Index1 = B;
			TriABC.Index2 = B;		 TriBDC.Index2 = D;
			TriABC.Index3 = C;		 TriBDC.Index3 = C;
			Triangles.Add(TriABC);
			Triangles.Add(TriBDC);
		}
	}
	UE_LOG(
		LogTemp,
		Display,
		TEXT("PBD grid initialized: %d particles, %d constraints, %d triangles"),
		Particles.Num(),
		DistanceConstraints.Num(),
		Triangles.Num()
	);
}

void FPBDClothSolver::Reset()
{
	Particles.Reset();
	Triangles.Reset();
	DistanceConstraints.Reset();
}

const TArray<FPBDParticle>& FPBDClothSolver::GetParticles() const
{
	return Particles;
}

const TArray<FPBDTriangle>& FPBDClothSolver::GetTriangles() const
{
	return Triangles;
}

const TArray<FPBDConstraint>& FPBDClothSolver::GetConstraint() const
{
	return DistanceConstraints;
}
