# PBD Cloth：球、有限圆柱碰撞与 Physics Asset 适配层

本章在现有距离／弯曲约束 Batch 框架上增加外部碰撞。核心算法自行实现，使用本机 Chaos 的球、圆柱、胶囊几何进行自动化对照；Physics Asset 只负责提供人体形状与骨骼绑定。

- 环境：Unreal Engine 5.8.1，Win64 Development Editor。
- 完成顺序：实现 → 实际 UE 编译与测试 → 导出 git diff → 根据差异生成本教程。
- 测试结果：4/4 成功，0 失败，0 测试警告；768 个几何对照采样，球／圆柱各运行 120 个布料模拟子步。
- 前置阅读：[如何在 PBD Cloth 中搭建可扩展的多约束批次框架](https://app.notion.com/p/3d4df403630281b3aa4fc4ae41952011)。本章基于当前已有弯曲约束的工作区，不应把前文的旧 Solver 全文重新覆盖回来。

## 1. 本次 diff 改了什么

- 新增 `PBDCollisionConstraintBatch.h/.cpp`：连续碰撞体数组、球／有限圆柱／胶囊的距离和法线查询、硬碰撞投影，优先级 300。
- 新增 `PBDPhysicsAssetAdapter.h/.cpp`：一次提取 Physics Asset，逐子步转换为求解器空间碰撞体；提供 SkeletalMeshComponent 便捷入口。
- 新增 `Tests/PBDCollisionTests.cpp`：四组 UE 自动化测试，测试与核心算法分离。
- 新增 `Scripts/test_pbd_collision.py`：独立源码副本编译并运行 UE 测试，不占用正在打开的项目 DLL。
- 修改 `pdd_cloth.Build.cs`：在本次开始时已有的 Chaos 依赖上补充 ChaosCore、PhysicsCore。

本轮未修改 Solver 的算法：现有 `RegisterConstraintBatch()` 已能接入新碰撞批次。现有距离／弯曲求解、场景资源和比较测试不属于这份补丁。

完整差异保存在 `Docs/PBDCollisionImplementation.patch`。新增文件用 `git diff --no-index -- NUL <file>` 导出；Build.cs 与本轮开始前的副本进行 `git diff --no-index`。这种做法只记录本次变化，不把用户先前未提交的改动混入教程，也不操作 Git 暂存区。

## 2. 碰撞约束：只在穿入厚度范围时推开

定义 `Phi(P)` 为预测位置到实体表面的有符号距离：外部为正，内部为负；`h` 为布料的碰撞厚度。

$$
C(P)=\phi(P)-h\geq 0
$$

当 `Phi < h` 时，沿外法线推出：

$$
P \leftarrow P+(h-\phi(P))n
$$

在单位法线下，一个可移动粒子对运动学碰撞体的硬 PBD 投影中，分子、分母的逆质量相消，所以修正量不需要再乘一次 InvMass。固定粒子 `InvMass <= 0` 直接跳过。这里不积累 XPBD Lambda，也没有软接触 Compliance。

核心实现：

```c++
if (Collider.PhiWithNormal(Particle.PredictedPosition, Phi, Normal) && Phi < Thickness)
{
    Particle.PredictedPosition += (Thickness - Phi) * Normal;
}
```

只修改 `PredictedPosition`。所有迭代结束后，现有 Solver 会用 `(PredictedPosition - Position) / Dt` 回写速度。若在这里只修改 Position，后面的速度更新会产生错误。

## 3. 三种形状的区别与公式

所有形状都先把粒子转换到以碰撞体 Center 为原点、Rotation 为朝向的局部空间。Rotation 必须是单位四元数，局部主轴为 Z。

### 球

局部位置为 p，球半径为 r：

$$
\phi(p)=\|p\|-r,\quad n=\frac{p}{\|p\|}
$$

在球心，法线没有唯一答案；实现选择稳定的局部 +X 单位方向，避免除零或返回零法线后无法推出。

### 有限圆柱：平端盖

半径 r，半高 H，两个端盖在 `z = ±H`。令：

$$
d_r=\sqrt{x^2+y^2}-r,\quad d_z=|z|-H
$$

$$
\phi=\sqrt{\max(d_r,0)^2+\max(d_z,0)^2}+\min(\max(d_r,d_z),0)
$$

- 侧面外：法线沿径向。
- 端盖外：法线沿正／负 Z。
- 同时越过侧面和端盖：最近点在圆边上，法线由径向与轴向距离加权后归一化。
- 实体内部：选择距离最近的侧面或端盖。
- 圆柱轴线上若需径向推出，使用局部 +X 作为确定的方向。

只检查 `sqrt(x*x+y*y) - Radius` 得到的是无限圆柱，不能处理端盖。只对圆柱侧面推出会让粒子在端部出现错误。

### 胶囊：用于准确接入 UE 的 Sphyl

胶囊的中心线段端点位于 `(0,0,-H)` 与 `(0,0,H)`，端部是半球。先把 p 投影到这条有限线段，再按球处理距离：

```c++
const FVector3f Closest(0, 0, FMath::Clamp(Local.Z, -HalfHeight, HalfHeight));
const FVector3f Delta = Local - Closest;
Phi = Delta.Size() - Radius;
```

胶囊总长度是 `2 * HalfHeight + 2 * Radius`；圆柱总长度是 `2 * HalfHeight`。零长度胶囊自然退化为球；零高度圆柱本版拒绝。

UE Physics Asset 的 `FKSphylElem` 是胶囊，`Length` 只代表中间线段的长度。因此适配时使用 `HalfHeight = Length / 2`，不能把它转换成平端盖圆柱。标准 AggGeom 没有与本章有限圆柱对应的原生 Cylinder 列表，圆柱通过手动输入创建。

## 4. 完整碰撞实现

按下列路径创建两个文件。核心代码没有调试控制台变量或逐粒子日志，测试放在独立文件中。

### `Source/pdd_cloth/PBDCollisionConstraintBatch.h`

```c++
#pragma once

#include "PBDConstraintBatch.h"

enum class EPBDColliderShape : uint8
{
    Sphere,
    Cylinder,
    Capsule
};

// Geometry is expressed in solver space, in cm. Rotation must be a unit quaternion.
// Cylinder has flat caps; Capsule has hemispherical caps. Local axis is +Z.
struct FPBDCollider
{
    EPBDColliderShape Shape = EPBDColliderShape::Sphere;
    FVector3f Center = FVector3f::ZeroVector;
    FQuat4f Rotation = FQuat4f::Identity;
    float Radius = 1.0f;
    // Cylinder: half cap-to-cap length. Capsule: half central segment length.
    float HalfHeight = 0.0f;

    // False means invalid geometry/input. Normal points out of the solid.
    bool PhiWithNormal(const FVector3f& Position, float& Phi, FVector3f& Normal) const;
};

// Hard, unilateral contacts against kinematic geometry: C(P) = Phi(P) - Thickness >= 0.
// Update the collider array before every solver substep. No UObject access during Solve.
class FPBDCollisionConstraintBatch final : public IPBDConstraintBatch
{
public:
    virtual FName GetDebugName() const override;
    virtual int32 GetSolvePriority() const override { return 300; }
    virtual void Solve(FPBDConstraintContext& Context) override;

    void SetColliders(TArray<FPBDCollider> InColliders) { Colliders = MoveTemp(InColliders); }
    const TArray<FPBDCollider>& GetColliders() const { return Colliders; }
    void SetThickness(float InThickness);
    float GetThickness() const { return Thickness; }

private:
    TArray<FPBDCollider> Colliders;
    float Thickness = 0.0f;
};
```

### `Source/pdd_cloth/PBDCollisionConstraintBatch.cpp`

```c++
#include "PBDCollisionConstraintBatch.h"

bool FPBDCollider::PhiWithNormal(const FVector3f& Position, float& Phi, FVector3f& Normal) const
{
    Phi = 0.0f;
    Normal = FVector3f::ZeroVector;
    if (Position.ContainsNaN() || Center.ContainsNaN() || Rotation.ContainsNaN() ||
        !Rotation.IsNormalized() || !FMath::IsFinite(Radius) || Radius <= 0.0f ||
        !FMath::IsFinite(HalfHeight) || HalfHeight < 0.0f)
    {
        return false;
    }

    const FVector3f Local = Rotation.UnrotateVector(Position - Center);
    FVector3f LocalNormal;
    switch (Shape)
    {
    case EPBDColliderShape::Sphere:
    case EPBDColliderShape::Capsule:
    {
        const float SegmentZ = Shape == EPBDColliderShape::Capsule
            ? FMath::Clamp(Local.Z, -HalfHeight, HalfHeight) : 0.0f;
        const FVector3f Delta = Local - FVector3f(0.0f, 0.0f, SegmentZ);
        const float Distance = Delta.Size();
        Phi = Distance - Radius;
        // The gradient is ambiguous at the center/segment. Choose a stable unit direction.
        LocalNormal = Distance > UE_SMALL_NUMBER ? Delta / Distance : FVector3f(1, 0, 0);
        break;
    }
    case EPBDColliderShape::Cylinder:
    {
        if (HalfHeight <= 0.0f)
        {
            return false;
        }
        const float Rho = FVector2f(Local.X, Local.Y).Size();
        const FVector3f RadialNormal = Rho > UE_SMALL_NUMBER
            ? FVector3f(Local.X / Rho, Local.Y / Rho, 0) : FVector3f(1, 0, 0);
        const FVector3f CapNormal(0, 0, Local.Z >= 0.0f ? 1.0f : -1.0f);
        const float SideDistance = Rho - Radius;
        const float CapDistance = FMath::Abs(Local.Z) - HalfHeight;
        const float OutsideSide = FMath::Max(SideDistance, 0.0f);
        const float OutsideCap = FMath::Max(CapDistance, 0.0f);
        const float OutsideDistance = FVector2f(OutsideSide, OutsideCap).Size();
        Phi = OutsideDistance + FMath::Min(FMath::Max(SideDistance, CapDistance), 0.0f);
        if (OutsideDistance > UE_SMALL_NUMBER)
        {
            // Outside a rim, the closest feature is the circular edge, not just a plane.
            LocalNormal = (OutsideSide * RadialNormal + OutsideCap * CapNormal) / OutsideDistance;
        }
        else
        {
            LocalNormal = SideDistance >= CapDistance ? RadialNormal : CapNormal;
        }
        break;
    }
    default:
        return false;
    }

    Normal = Rotation.RotateVector(LocalNormal);
    return FMath::IsFinite(Phi) && !Normal.ContainsNaN();
}

FName FPBDCollisionConstraintBatch::GetDebugName() const
{
    static const FName Name(TEXT("Collision"));
    return Name;
}

void FPBDCollisionConstraintBatch::SetThickness(float InThickness)
{
    Thickness = FMath::IsFinite(InThickness) ? FMath::Max(InThickness, 0.0f) : 0.0f;
}

void FPBDCollisionConstraintBatch::Solve(FPBDConstraintContext& Context)
{
    for (FPBDParticle& Particle : Context.Particles)
    {
        if (!FMath::IsFinite(Particle.InvMass) || Particle.InvMass <= 0.0f)
        {
            continue;
        }
        for (const FPBDCollider& Collider : Colliders)
        {
            float Phi;
            FVector3f Normal;
            if (Collider.PhiWithNormal(Particle.PredictedPosition, Phi, Normal) && Phi < Thickness)
            {
                Particle.PredictedPosition += (Thickness - Phi) * Normal;
            }
        }
    }
}
```

## 5. Physics Asset 适配：数据来源与求解器分离

适配层有两个阶段：

1. `Initialize()`：读取 SkeletalBodySetups，通过 `Body->BoneName` 在参考骨架查索引，保存骨骼局部形状。本版支持 SphereElems 与 SphylElems。
2. `Update()`：接收动画计算后的 component-space 骨骼变换，把形状转换为 solver-space 数组。

位置转换顺序：

```text
骨骼局部形状中心
    → Bone.TransformPosition
    → ComponentToWorld.TransformPosition
    → SolverToWorld.InverseTransformPosition
    → 求解器空间中心
```

旋转顺序：

```c++
SolverRotation.Inverse() * ComponentRotation * BoneRotation * ShapeRotation
```

正的均匀缩放下，半径和半高乘以：

```c++
BoneScale * ComponentScale / SolverScale
```

世界／组件／骨骼变换使用 UE 双精度 FVector 和 FTransform，进入粒子求解器时才转换为 FVector3f。这样能在转换到局部空间后再缩小精度；局部坐标仍应保持合理量级。

接口边界：

- `InitializeFromComponent()` 从组件读取 Skeletal Mesh 和实际 Physics Asset；适配器不持有 UObject 指针。
- `UpdateFromComponent()` 读取最新 component-space 骨骼姿态；骨骼名称与索引不匹配时要求重新初始化。
- 使用底层 `Update()` 时，调用方必须保证骨骼数组与初始化参考骨架一致。
- 资产更换、资产形状编辑或参考骨架更换后，要重新 Initialize。
- 缺失骨骼、无效尺寸、其他形状会在 `FPBDPhysicsAssetReport::Issues` 中报告。允许保留其余合法形状，但调用者应检查报告。
- 本版只接受有限、正的均匀缩放。非均匀缩放、镜像、零缩放会报告并跳过；没有用最大缩放悄悄近似身体形状。
- Update 每次替换输出，即使失败也会清空旧数组，避免上一帧的碰撞体残留。
- 初始化与组件读取应在游戏线程、动画姿态评估完成后执行。Solve 只读普通碰撞体数据，不访问 UObject。

### 完整适配层代码

### `Source/pdd_cloth/PBDPhysicsAssetAdapter.h`

```c++
#pragma once

#include "PBDCollisionConstraintBatch.h"

class UPhysicsAsset;
class USkeletalMeshComponent;
struct FReferenceSkeleton;

struct FPBDPhysicsAssetReport
{
    int32 NumColliders = 0;
    TArray<FString> Issues;
    bool IsComplete() const { return Issues.IsEmpty(); }
};

// Read Physics Asset once, update the resulting kinematic geometry each substep.
// Extraction/component access belongs on the game thread after animation evaluation.
// No asset pointers are retained. Reinitialize when asset or reference skeleton changes.
class FPBDPhysicsAssetAdapter
{
public:
    FPBDPhysicsAssetReport Initialize(const UPhysicsAsset* Asset, const FReferenceSkeleton& Skeleton);
    FPBDPhysicsAssetReport InitializeFromComponent(const USkeletalMeshComponent& Component);

    // Bone transforms are COMPONENT space, indexed by the initialization reference skeleton.
    // Replaces OutColliders, including on failure, so old collision bodies cannot remain active.
    // Supports positive uniform scale only. Other scales are explicitly reported and skipped.
    FPBDPhysicsAssetReport Update(TConstArrayView<FTransform> ComponentSpaceBones,
        const FTransform& ComponentToWorld, const FTransform& SolverToWorld,
        TArray<FPBDCollider>& OutColliders) const;
    FPBDPhysicsAssetReport UpdateFromComponent(const USkeletalMeshComponent& Component,
        const FTransform& SolverToWorld, TArray<FPBDCollider>& OutColliders) const;

private:
    struct FBinding
    {
        FName BoneName;
        int32 BoneIndex = INDEX_NONE;
        FPBDCollider BoneLocalCollider;
    };
    TArray<FBinding> Bindings;
};
```

### `Source/pdd_cloth/PBDPhysicsAssetAdapter.cpp`

```c++
#include "PBDPhysicsAssetAdapter.h"

#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"

namespace
{
bool HasSupportedTransform(const FTransform& Transform)
{
    const FVector Scale = Transform.GetScale3D();
    return !Transform.ContainsNaN() && Transform.GetRotation().IsNormalized() &&
        Scale.X > UE_SMALL_NUMBER &&
        FMath::IsNearlyEqual(Scale.X, Scale.Y, 1.e-6) &&
        FMath::IsNearlyEqual(Scale.X, Scale.Z, 1.e-6);
}
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::Initialize(
    const UPhysicsAsset* Asset, const FReferenceSkeleton& Skeleton)
{
    Bindings.Reset();
    FPBDPhysicsAssetReport Report;
    if (!Asset)
    {
        Report.Issues.Add(TEXT("No Physics Asset supplied."));
        return Report;
    }

    for (const USkeletalBodySetup* Body : Asset->SkeletalBodySetups)
    {
        if (!Body)
        {
            Report.Issues.Add(TEXT("Null skeletal body setup."));
            continue;
        }
        const int32 BoneIndex = Skeleton.FindBoneIndex(Body->BoneName);
        if (BoneIndex == INDEX_NONE)
        {
            Report.Issues.Add(FString::Printf(TEXT("Missing bone: %s"), *Body->BoneName.ToString()));
            continue;
        }

        const auto Add = [&](const FPBDCollider& Collider)
        {
            float Phi;
            FVector3f Normal;
            if (!Collider.PhiWithNormal(Collider.Center, Phi, Normal))
            {
                Report.Issues.Add(FString::Printf(TEXT("Invalid shape on bone: %s"), *Body->BoneName.ToString()));
                return;
            }
            Bindings.Add({Body->BoneName, BoneIndex, Collider});
        };
        const FKAggregateGeom& Geometry = Body->AggGeom;
        for (const FKSphereElem& Sphere : Geometry.SphereElems)
        {
            FPBDCollider Collider;
            Collider.Center = FVector3f(Sphere.Center);
            Collider.Radius = Sphere.Radius;
            Add(Collider);
        }
        for (const FKSphylElem& Capsule : Geometry.SphylElems)
        {
            FPBDCollider Collider;
            Collider.Shape = EPBDColliderShape::Capsule;
            Collider.Center = FVector3f(Capsule.Center);
            Collider.Rotation = FQuat4f(Capsule.Rotation.Quaternion());
            Collider.Radius = Capsule.Radius;
            Collider.HalfHeight = Capsule.Length * 0.5f;
            Add(Collider);
        }
        const int32 Unsupported = Geometry.GetElementCount() - Geometry.SphereElems.Num() - Geometry.SphylElems.Num();
        if (Unsupported > 0)
        {
            Report.Issues.Add(FString::Printf(TEXT("Skipped %d unsupported shapes on bone %s (only sphere/sphyl supported)."),
                Unsupported, *Body->BoneName.ToString()));
        }
    }
    Report.NumColliders = Bindings.Num();
    return Report;
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::InitializeFromComponent(const USkeletalMeshComponent& Component)
{
    if (const USkeletalMesh* Mesh = Component.GetSkeletalMeshAsset())
    {
        return Initialize(Component.GetPhysicsAsset(), Mesh->GetRefSkeleton());
    }
    Bindings.Reset();
    FPBDPhysicsAssetReport Report;
    Report.Issues.Add(TEXT("Component has no skeletal mesh."));
    return Report;
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::Update(
    TConstArrayView<FTransform> ComponentSpaceBones, const FTransform& ComponentToWorld,
    const FTransform& SolverToWorld, TArray<FPBDCollider>& OutColliders) const
{
    OutColliders.Reset();
    FPBDPhysicsAssetReport Report;
    if (!HasSupportedTransform(ComponentToWorld) || !HasSupportedTransform(SolverToWorld))
    {
        Report.Issues.Add(TEXT("Component and solver transforms require finite, positive uniform scale and unit rotation."));
        return Report;
    }
    OutColliders.Reserve(Bindings.Num());
    for (const FBinding& Binding : Bindings)
    {
        if (!ComponentSpaceBones.IsValidIndex(Binding.BoneIndex) ||
            !HasSupportedTransform(ComponentSpaceBones[Binding.BoneIndex]))
        {
            Report.Issues.Add(FString::Printf(TEXT("Missing pose or unsupported bone transform: %s"), *Binding.BoneName.ToString()));
            continue;
        }
        const FTransform& Bone = ComponentSpaceBones[Binding.BoneIndex];
        FPBDCollider Collider = Binding.BoneLocalCollider;
        const FVector WorldCenter = ComponentToWorld.TransformPosition(Bone.TransformPosition(FVector(Collider.Center)));
        Collider.Center = FVector3f(SolverToWorld.InverseTransformPosition(WorldCenter));
        Collider.Rotation = FQuat4f(SolverToWorld.GetRotation().Inverse() * ComponentToWorld.GetRotation() *
            Bone.GetRotation() * FQuat(Collider.Rotation));
        Collider.Rotation.Normalize();
        const double Scale = Bone.GetScale3D().X * ComponentToWorld.GetScale3D().X / SolverToWorld.GetScale3D().X;
        Collider.Radius *= Scale;
        Collider.HalfHeight *= Scale;
        float Phi;
        FVector3f Normal;
        if (!Collider.PhiWithNormal(Collider.Center, Phi, Normal))
        {
            Report.Issues.Add(FString::Printf(TEXT("Transformed collider is not finite/valid: %s"), *Binding.BoneName.ToString()));
            continue;
        }
        OutColliders.Add(Collider);
    }
    Report.NumColliders = OutColliders.Num();
    return Report;
}

FPBDPhysicsAssetReport FPBDPhysicsAssetAdapter::UpdateFromComponent(
    const USkeletalMeshComponent& Component, const FTransform& SolverToWorld,
    TArray<FPBDCollider>& OutColliders) const
{
    const USkeletalMesh* Mesh = Component.GetSkeletalMeshAsset();
    for (const FBinding& Binding : Bindings)
    {
        if (!Mesh || Mesh->GetRefSkeleton().FindBoneIndex(Binding.BoneName) != Binding.BoneIndex)
        {
            OutColliders.Reset();
            FPBDPhysicsAssetReport Report;
            Report.Issues.Add(TEXT("Reference skeleton changed; reinitialize the Physics Asset adapter."));
            return Report;
        }
    }
    return Update(Component.GetComponentSpaceTransforms(), Component.GetComponentTransform(), SolverToWorld, OutColliders);
}
```

## 6. 模块依赖

本轮 Build.cs 的实际差异：

```diff
diff --git a/Source/pdd_cloth/pdd_cloth.Build.cs b/Source/pdd_cloth/pdd_cloth.Build.cs
index 3b0a128..a8098a9 100644
--- a/Source/pdd_cloth/pdd_cloth.Build.cs
+++ b/Source/pdd_cloth/pdd_cloth.Build.cs
@@ -10,7 +10,7 @@ public class pdd_cloth : ModuleRules
 	
 		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput" });
 
-		PrivateDependencyModuleNames.AddRange(new string[] { "Chaos" });
+		PrivateDependencyModuleNames.AddRange(new string[] { "Chaos", "ChaosCore", "PhysicsCore" });
 
 		// Uncomment if you are using Slate UI
 		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

```

PhysicsCore 支持物理形状相关接口。Chaos／ChaosCore 用于本项目现有比较代码和新增的真实 Chaos 几何测试；自写距离与投影公式不调用 Chaos 求解器。

## 7. 如何接入当前 Solver

### 先手动创建球和圆柱

在 `InitializeGrid()` 之后注册一次碰撞 Batch：

```c++
FPBDCollider Sphere;
Sphere.Shape = EPBDColliderShape::Sphere;
Sphere.Center = FVector3f(0, 0, -30);
Sphere.Radius = 20;

FPBDCollider Cylinder;
Cylinder.Shape = EPBDColliderShape::Cylinder;
Cylinder.Center = FVector3f(50, 0, -30);
Cylinder.Radius = 10;
Cylinder.HalfHeight = 25; // 平端盖距离为 50 cm

auto Batch = MakeUnique<FPBDCollisionConstraintBatch>();
Batch->SetThickness(1.0f);
Batch->SetColliders({Sphere, Cylinder});
Solver.RegisterConstraintBatch(MoveTemp(Batch));
```

现有 Solver 的排序为 Distance=100、Bend=200、Collision=300。每轮迭代先解内部约束，再投影碰撞。碰撞不能只在所有迭代之前执行一次，否则拉伸或弯曲可能再次把粒子拉入身体。

### 为角色集成准备持久状态

在未来组件中持有 `FPBDPhysicsAssetAdapter BodyAdapter` 和非拥有的 `FPBDCollisionConstraintBatch* BodyCollision`。以下是接入示例，展示已测试接口的调用顺序；本轮尚未新增角色运行时组件或绑定现有角色资产。

初始化时，在 Solver.InitializeGrid 之后执行：

```c++
const FPBDPhysicsAssetReport BuildReport = BodyAdapter.InitializeFromComponent(*CharacterMesh);
// 检查 BuildReport.Issues，例如缺失骨骼或不支持的碰撞形状。
auto Batch = MakeUnique<FPBDCollisionConstraintBatch>();
BodyCollision = Batch.Get();
BodyCollision->SetThickness(1.0f);
Solver.RegisterConstraintBatch(MoveTemp(Batch));
```

每个固定子时间步、Solver.Step 之前执行：

```c++
TArray<FPBDCollider> BodyShapes;
const FPBDPhysicsAssetReport PoseReport = BodyAdapter.UpdateFromComponent(
    *CharacterMesh, SolverToWorld, BodyShapes);
// 检查 PoseReport.Issues。即使失败，也提交空数组以清掉旧形状。
BodyCollision->SetColliders(MoveTemp(BodyShapes));
Solver.Step(FixedDeltaTime, SolverIterations);
```

SolverToWorld 必须表示粒子所用模拟空间的世界变换：世界空间模拟用 Identity；局部空间模拟用该空间的真实变换。Gravity 等外力也应与粒子使用同一坐标空间。

Reset 或重新 InitializeGrid 会销毁已注册的 Batch，因此 BodyCollision 随之失效；重新创建、注册并更新指针后才能调用。不要每帧重复注册 Batch。

## 8. 如何运行测试与检查结果

在项目根目录执行：

```powershell
python Scripts/test_pbd_collision.py --engine D:/UE5/UE_5.8
```

脚本复制当前 Source 与 uproject 到 `Saved/CollisionValidation`，用真实 UBT 编译，然后启动独立的 UnrealEditor-Cmd 进程运行 `PBD.Collision`。不复制内容资产，也不修改当前关卡。

本次实际结果：

- `PBD.Collision.GeometryAgainstChaos`：成功。球、圆柱、胶囊各 256 个随机采样；包含旋转和平移；对比 Chaos 的 Phi 和 Normal，并检查投影后的表面距离。距离／法线容差为 2e-5。
- `PBD.Collision.ProjectionAndEdgeCases`：成功。验证实体内部、中心／轴线、上下端盖、圆边厚度、外部不移动、固定点不移动、重复投影、无效半径、NaN、退化胶囊。
- `PBD.Collision.PhysicsAssetAdapter`：成功。使用临时 UPhysicsAsset 和两骨骼参考骨架；验证形状类型、骨骼／组件／求解器空间、旋转端点、缩放、更新姿态、缺失骨骼、缺失姿态、不支持形状、失效数据清空。
- `PBD.Collision.ClothSolverIntegration`：成功。球和圆柱各运行 8×8 网格、120 步、每步 8 次迭代；逐步验证有限位置／速度、不穿入厚度范围、固定角点不漂移。

编译记录：`Saved/CollisionValidation/Build.log`。
实际完整测试结果：`Saved/CollisionValidation/Report/index.json`。
可提交的结果摘要与源码 SHA-256：`Docs/PBDCollisionTestResults.json`。

测试骨架的 FReferenceSkeletonModifier 必须先离开作用域，才可用最终骨骼索引表；测试代码已按此组织。

测试使用真实 C++ 实现，未用 Python 重写碰撞算法作为替代。测试副本与工作区的核心代码、测试源码及 Build.cs 已按字节比对一致。正在运行的原编辑器尚未重新加载新 DLL；要在原编辑器中使用新增代码，需要正常重新编译／重新加载模块。

### 完整测试代码

### `Source/pdd_cloth/Tests/PBDCollisionTests.cpp`

```c++
#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../PBDCollisionConstraintBatch.h"
#include "../PBDPhysicsAssetAdapter.h"
#include "../PBDClothSolver.h"
#include "Chaos/Sphere.h"
#include "Chaos/Cylinder.h"
#include "Chaos/Capsule.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "ReferenceSkeleton.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDCollisionGeometryTest, "PBD.Collision.GeometryAgainstChaos",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDCollisionGeometryTest::RunTest(const FString& Parameters)
{
    const Chaos::FSphere Sphere(Chaos::FVec3(0), 2);
    const Chaos::FCylinder Cylinder(Chaos::FVec3(0, 0, -3), Chaos::FVec3(0, 0, 3), 2);
    const Chaos::FCapsule Capsule(Chaos::FVec3(0, 0, -3), Chaos::FVec3(0, 0, 3), 2);
    const Chaos::FImplicitObject* References[] = { &Sphere, &Cylinder, &Capsule };
    FRandomStream Random(90210);
    for (int32 ShapeIndex = 0; ShapeIndex < 3; ++ShapeIndex)
    {
        FPBDCollider Collider;
        Collider.Shape = static_cast<EPBDColliderShape>(ShapeIndex);
        Collider.Radius = 2;
        Collider.HalfHeight = 3;
        Collider.Center = FVector3f(7, -4, 11);
        Collider.Rotation = FQuat4f(FVector3f(1, 2, 3).GetSafeNormal(), 0.73f);
        for (int32 Sample = 0; Sample < 256; ++Sample)
        {
            const FVector3f Local(Random.FRandRange(-5, 5), Random.FRandRange(-5, 5), Random.FRandRange(-6, 6));
            const FVector3f Point = Collider.Center + Collider.Rotation.RotateVector(Local);
            float Phi;
            FVector3f Normal;
            TestTrue(TEXT("Finite distance query"), Collider.PhiWithNormal(Point, Phi, Normal));
            Chaos::FVec3 ReferenceNormal;
            const double ReferencePhi = References[ShapeIndex]->PhiWithNormal(Chaos::FVec3(Local), ReferenceNormal);
            TestTrue(TEXT("SDF agrees with Chaos"), FMath::Abs(Phi - ReferencePhi) < 2.e-5);
            const FVector3f ExpectedNormal = Collider.Rotation.RotateVector(FVector3f(ReferenceNormal));
            TestTrue(TEXT("Normal agrees with Chaos"), Normal.Equals(ExpectedNormal, 2.e-5f));
            // Independently check that a signed-distance projection lands on the surface.
            float SurfacePhi;
            FVector3f SurfaceNormal;
            Collider.PhiWithNormal(Point - Phi * Normal, SurfacePhi, SurfaceNormal);
            TestTrue(TEXT("Projection reaches surface"), FMath::Abs(SurfacePhi) < 2.e-5f);
        }
        float Phi;
        FVector3f Normal;
        TestTrue(TEXT("Center is handled"), Collider.PhiWithNormal(Collider.Center, Phi, Normal));
        TestTrue(TEXT("Center fallback is unit"), FMath::IsNearlyEqual(Normal.Size(), 1.0f));
        TestTrue(TEXT("Center is inside"), Phi < 0);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDCollisionProjectionTest, "PBD.Collision.ProjectionAndEdgeCases",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDCollisionProjectionTest::RunTest(const FString& Parameters)
{
    for (EPBDColliderShape Shape : {EPBDColliderShape::Sphere, EPBDColliderShape::Cylinder, EPBDColliderShape::Capsule})
    {
        FPBDCollider Collider;
        Collider.Shape = Shape;
        Collider.Radius = 2;
        Collider.HalfHeight = 3;
        FPBDCollisionConstraintBatch Batch;
        Batch.SetColliders({Collider});
        Batch.SetThickness(0.5f);
        TArray<FPBDParticle> Particles;
        // Center, side, upper cap, lower cap, rounded thickness at rim, exterior, fixed point.
        for (const FVector3f P : {FVector3f(0), FVector3f(1, 0, 0), FVector3f(0, 0, 2.9f),
            FVector3f(0, 0, -2.9f), FVector3f(2.1f, 0, 3.1f), FVector3f(8, 0, 0)})
        {
            Particles.Emplace(P, 1.0f);
        }
        Particles.Emplace(FVector3f(0), 0.0f);
        FPBDConstraintContext Context;
        Context.Particles = MakeArrayView(Particles);
        Context.DeltaTime = 1.0f / 60;
        Batch.Solve(Context);
        for (int32 I = 0; I < Particles.Num() - 1; ++I)
        {
            float Phi;
            FVector3f Normal;
            Collider.PhiWithNormal(Particles[I].PredictedPosition, Phi, Normal);
            TestTrue(TEXT("All movable particles separated by thickness"), Phi >= 0.5f - 1.e-5f);
            TestEqual(TEXT("Projection does not commit old position"), Particles[I].Velocity, FVector3f::ZeroVector);
        }
        TestEqual(TEXT("Fixed point unchanged"), Particles.Last().PredictedPosition, FVector3f(0));
        TestEqual(TEXT("Exterior unchanged"), Particles[5].PredictedPosition, FVector3f(8, 0, 0));
        const FVector3f Projected = Particles[0].PredictedPosition;
        Batch.Solve(Context);
        TestTrue(TEXT("Repeated projection is idempotent"), Projected.Equals(Particles[0].PredictedPosition, 1.e-5f));
    }
    FPBDCollider Bad;
    float Phi;
    FVector3f Normal;
    Bad.Radius = -1;
    TestFalse(TEXT("Negative radius rejected"), Bad.PhiWithNormal(FVector3f(0), Phi, Normal));
    Bad.Radius = 1;
    Bad.Shape = EPBDColliderShape::Cylinder;
    TestFalse(TEXT("Zero-height cylinder rejected"), Bad.PhiWithNormal(FVector3f(0), Phi, Normal));
    Bad.Shape = EPBDColliderShape::Capsule;
    TestTrue(TEXT("Zero-length capsule is a sphere"), Bad.PhiWithNormal(FVector3f(2, 0, 0), Phi, Normal));
    TestEqual(TEXT("Zero-length capsule distance"), Phi, 1.0f);
    TestFalse(TEXT("NaN input rejected"), Bad.PhiWithNormal(FVector3f(std::numeric_limits<float>::quiet_NaN(), 0, 0), Phi, Normal));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDPhysicsAssetAdapterTest, "PBD.Collision.PhysicsAssetAdapter",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDPhysicsAssetAdapterTest::RunTest(const FString& Parameters)
{
    FReferenceSkeleton Skeleton;
    {
        // The modifier rebuilds the final bone lookup when its scope ends.
        FReferenceSkeletonModifier Modifier(Skeleton, nullptr);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("arm"), TEXT("arm"), 0), FTransform::Identity);
    }
    UPhysicsAsset* Asset = NewObject<UPhysicsAsset>();
    USkeletalBodySetup* Body = NewObject<USkeletalBodySetup>(Asset);
    Body->BoneName = TEXT("arm");
    FKSphereElem Sphere(2);
    Sphere.Center = FVector(1, 0, 0);
    Body->AggGeom.SphereElems.Add(Sphere);
    FKSphylElem Capsule(1, 6);
    Capsule.Center = FVector(0, 2, 0);
    Capsule.Rotation = FRotator(0, 0, 90);
    Body->AggGeom.SphylElems.Add(Capsule);
    Asset->SkeletalBodySetups.Add(Body);
    FPBDPhysicsAssetAdapter Adapter;
    FPBDPhysicsAssetReport Report = Adapter.Initialize(Asset, Skeleton);
    TestTrue(TEXT("Supported extraction complete"), Report.IsComplete());
    TestEqual(TEXT("Two shapes extracted"), Report.NumColliders, 2);

    const FTransform Bone(FQuat(FVector::UpVector, UE_PI / 2), FVector(10, 0, 0), FVector(2));
    TArray<FTransform> Bones = {FTransform::Identity, Bone};
    const FTransform Component(FQuat(FVector::UpVector, UE_PI / 2), FVector(100, 200, 0), FVector(3));
    const FTransform Solver(FQuat::Identity, FVector(100, 200, 0), FVector(2));
    TArray<FPBDCollider> Colliders;
    Report = Adapter.Update(Bones, Component, Solver, Colliders);
    TestTrue(TEXT("Pose conversion complete"), Report.IsComplete());
    if (!TestEqual(TEXT("Two runtime shapes"), Colliders.Num(), 2)) { return false; }
    TestTrue(TEXT("Sphere center applies bone/component/solver spaces"), Colliders[0].Center.Equals(FVector3f(-3, 15, 0), 1.e-4f));
    TestEqual(TEXT("Uniform scale radius"), Colliders[0].Radius, 6.0f);
    TestTrue(TEXT("Physics Asset sphyl stays capsule"), Colliders[1].Shape == EPBDColliderShape::Capsule);
    TestEqual(TEXT("Length excludes hemispheres"), Colliders[1].HalfHeight, 9.0f);
    const FVector ExpectedEndpoint = Solver.InverseTransformPosition(Component.TransformPosition(Bone.TransformPosition(
        Capsule.Center + Capsule.Rotation.RotateVector(FVector(0, 0, 3)))));
    const FVector3f ActualEndpoint = Colliders[1].Center + Colliders[1].Rotation.RotateVector(FVector3f(0, 0, 9));
    TestTrue(TEXT("Capsule orientation/endpoints follow bone"), ActualEndpoint.Equals(FVector3f(ExpectedEndpoint), 1.e-4f));
    Bones[1].AddToTranslation(FVector(4, 0, 0));
    Adapter.Update(Bones, Component, Solver, Colliders);
    TestTrue(TEXT("Next pose moves existing shapes"), Colliders[0].Center.Equals(FVector3f(-3, 21, 0), 1.e-4f));

    Bones[1].SetScale3D(FVector(1, 2, 1));
    Report = Adapter.Update(Bones, Component, Solver, Colliders);
    TestFalse(TEXT("Non-uniform bone scale reported"), Report.IsComplete());
    TestTrue(TEXT("Old colliders cleared on bad pose"), Colliders.IsEmpty());
    Bones.SetNum(1);
    Report = Adapter.Update(Bones, Component, Solver, Colliders);
    TestFalse(TEXT("Missing pose reported"), Report.IsComplete());
    Report = Adapter.Update(Bones, Component, FTransform(FQuat::Identity, FVector(0), FVector(-1)), Colliders);
    TestFalse(TEXT("Mirrored solver rejected"), Report.IsComplete());

    Body->AggGeom.BoxElems.Add(FKBoxElem(1, 1, 1));
    USkeletalBodySetup* Missing = NewObject<USkeletalBodySetup>(Asset);
    Missing->BoneName = TEXT("missing");
    Missing->AggGeom.SphereElems.Add(FKSphereElem(1));
    Asset->SkeletalBodySetups.Add(Missing);
    Report = Adapter.Initialize(Asset, Skeleton);
    TestEqual(TEXT("Unsupported shape and missing bone reported"), Report.Issues.Num(), 2);
    TestEqual(TEXT("Valid shapes retained"), Report.NumColliders, 2);
    Report = Adapter.Initialize(nullptr, Skeleton);
    TestFalse(TEXT("Null asset reported"), Report.IsComplete());
    Adapter.Update({}, FTransform::Identity, FTransform::Identity, Colliders);
    TestTrue(TEXT("Reinitialization clears old bindings"), Colliders.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPBDCollisionSolverTest, "PBD.Collision.ClothSolverIntegration",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPBDCollisionSolverTest::RunTest(const FString& Parameters)
{
    for (EPBDColliderShape Shape : {EPBDColliderShape::Sphere, EPBDColliderShape::Cylinder})
    {
        FPBDClothSolver Solver;
        Solver.InitializeGrid(8, 8, 2);
        FPBDCollider Collider;
        Collider.Shape = Shape;
        Collider.Center = FVector3f(0, 0, -8);
        Collider.Radius = 4;
        Collider.HalfHeight = 2;
        auto Batch = MakeUnique<FPBDCollisionConstraintBatch>();
        Batch->SetThickness(0.2f);
        Batch->SetColliders({Collider});
        Solver.RegisterConstraintBatch(MoveTemp(Batch));
        const FVector3f Pin = Solver.GetParticles()[0].Position;
        for (int32 Step = 0; Step < 120; ++Step)
        {
            Solver.Step(1.0f / 120, 8);
            for (const FPBDParticle& Particle : Solver.GetParticles())
            {
                TestFalse(TEXT("Position finite"), Particle.Position.ContainsNaN());
                TestFalse(TEXT("Velocity finite"), Particle.Velocity.ContainsNaN());
                if (Particle.InvMass > 0)
                {
                    float Phi;
                    FVector3f Normal;
                    Collider.PhiWithNormal(Particle.Position, Phi, Normal);
                    TestTrue(TEXT("Collision remains enforced after internal constraints"), Phi >= 0.2f - 1.e-4f);
                }
            }
        }
        TestEqual(TEXT("Pinned corner unchanged"), Solver.GetParticles()[0].Position, Pin);
    }
    return true;
}

#endif
```

### `Scripts/test_pbd_collision.py`

```python
"""Build an isolated source snapshot and run real UE automation tests.

The working editor can remain open. No live project DLL, content, or map is modified.
Usage: python Scripts/test_pbd_collision.py [--engine D:/UE5/UE_5.8]
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", type=Path, default=Path("D:/UE5/UE_5.8"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    validation = root / "Saved" / "CollisionValidation"
    validation.mkdir(parents=True, exist_ok=True)
    shutil.copytree(root / "Source", validation / "Source", dirs_exist_ok=True)
    project = validation / "pdd_cloth.uproject"
    shutil.copy2(root / "pdd_cloth.uproject", project)
    report = validation / "Report"
    build_log = validation / "Build.log"
    command = [str(args.engine / "Engine/Build/BatchFiles/Build.bat"),
               "pdd_clothEditor", "Win64", "Development", f"-Project={project}",
               "-WaitMutex", "-NoHotReloadFromIDE"]
    print(f"Building source snapshot; log: {build_log}", flush=True)
    with build_log.open("w", encoding="utf-8") as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=900)
    # Remove only the previous summary file, so a failed run cannot reuse an old pass.
    summary_file = report / "index.json"
    summary_file.unlink(missing_ok=True)
    run_log = validation / "Automation.log"
    command = [str(args.engine / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"), str(project),
               "-unattended", "-nop4", "-NullRHI", "-nosplash", "-nosound",
               "-ExecCmds=Automation RunTests PBD.Collision", "-TestExit=Automation Test Queue Empty",
               f"-ReportExportPath={report}", f"-abslog={run_log}"]
    print(f"Running PBD.Collision; log: {run_log}", flush=True)
    with (validation / "Process.log").open("w", encoding="utf-8") as output:
        subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=300)
    summary = json.loads(summary_file.read_text(encoding="utf-8-sig"))
    if summary.get("failed", 0) or summary.get("succeeded", 0) != 4:
        raise RuntimeError(f"Expected four passing tests: {summary_file}")
    print(f"PASS: {summary['succeeded']} tests; report: {summary_file}", flush=True)


if __name__ == "__main__":
    main()
```

## 9. 本机 Chaos 源码对应位置

根目录：`D:/UE5/UE_5.8/Engine/`。

- `Plugins/ChaosCloth/Source/ChaosCloth/Private/ChaosCloth/ChaosClothingSimulationCollider.cpp:539`：Physics Asset 提取入口；Sphyl 中央线段长度与骨骼映射。
- 同文件 `:428`：骨骼变换更新碰撞体。
- `Source/Runtime/Experimental/Chaos/Public/Chaos/Cylinder.h:104`：有限圆柱 PhiWithNormal，区分侧面、端盖、圆边。
- `Source/Runtime/Experimental/Chaos/Public/Chaos/Sphere.h` 与 `Capsule.h`：本章自动化测试直接实例化的官方几何类型。
- `Source/Runtime/Experimental/Chaos/Private/Chaos/PerParticlePBDCollisionConstraint.cpp:186`：局部空间距离查询、Thickness - Phi、沿法线修正预测位置。
- `Source/Runtime/Experimental/Chaos/Private/Chaos/PBDEvolution.cpp:586`：在迭代中调用碰撞规则与 CCD 分支。
- `Source/Runtime/Experimental/Chaos/Private/Chaos/PBDSoftBodyCollisionConstraint.cpp:547`：新 Evolution 的普通形状碰撞路径。

借鉴的是分层结构、距离查询接口、投影方法与骨骼映射；项目代码自行实现。球心／胶囊中心线／圆柱轴线上的法线不唯一，本实现明确提供稳定单位方向。随机对照避开这种不可微点，特殊测试单独验证它们可被推出。

## 10. 当前边界与后续工作

- 当前是粒子对运动学几何的离散、硬、无摩擦碰撞。厚度为非负距离；未实现软接触、摩擦、反弹系数、CCD、自碰撞和布料反推刚体。
- 高速运动可能在一个子步中跨过整个形状；仅检测最终位置不能保证防穿透。后续可按 Chaos 的路径加扫掠／CCD。
- 多碰撞体使用顺序投影。有限迭代下，最后处理的形状可能影响先前接触；不可满足的夹缝、固定点处于身体内部，需要上层处理，不能靠本次投影保证全局无穿透。
- 粒子不穿入不能保证整个三角形或边都不穿入细小障碍；需要更密网格或后续面／边碰撞。
- 适配器目前支持球和普通胶囊；Box、Convex、TaperedCapsule、LevelSet 等会报告为不支持。
- 本轮还未接入真实角色动画、子步姿态插值、移动局部空间惯性补偿或动态附着。调用 UpdateFromComponent 会读取当前姿态；需要时间插值时应先计算中间姿态，再传给底层 Update。
- 不把“球／圆柱几何与 Chaos 一致”解释为“整个自写布料求解器与 Chaos 完全一致”。测试保证的是本章列出的具体行为与输入范围。

下一步可以从真实角色 Physics Asset 的球／胶囊数据出发，先完成动画评估后的更新时序与附着点，再添加接触摩擦及 CCD。
