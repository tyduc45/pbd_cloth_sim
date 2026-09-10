# PBD / Chaos 双布料对拍

打开 `/Game/Tests/PBDChaosComparison`，点击 **Play**，再点击游戏视口获取键盘焦点。
地图中的 `PBDClothComparisonActor` 会自动切换到自己的观察相机；布料线框在运行时绘制。
首次使用新增 C++ 类前请保存原场景并重启编辑器。原场景和项目默认地图没有改动。

## 两侧比较的对象

- 左侧青色：项目现有 `FPBDClothSolver`，Distance + Bend，包含 BC 对角线。
- 右侧橙色：直接调用引擎 `Chaos::Softs::FPBDSpringConstraints` 和 `FPBDBendingConstraints`。
- 红点：固定粒子。`O` 可以取消视觉平移，将两套结果叠在一起。

这是官方约束类的对拍，不是完整 Chaos Cloth Asset / FEvolution 流程的对拍。
右侧没有注册碰撞、自碰撞、系绳、动画驱动、空气动力、阻尼或其他约束。
穿插仍可能发生，因为测试有意关闭碰撞。

## 控制变量

| 参数 | 两侧设置 |
| --- | --- |
| 粒子 | 16 × 16，间距 10 cm，XZ 平面 |
| 固定点 | 第一行的两个角点 |
| 逆质量 | 自由点 1，固定点 0 |
| 距离约束 | 同一份 705 条边，包括 225 条 BC 对角线 |
| 弯曲约束 | 同一份 645 条四点共享边约束 |
| 初始速度 | 0 |
| 重力 | 局部 `(0,0,-980)` cm/s² |
| 时间步 | 1/60 秒，每步 8 次迭代 |
| 求解顺序 | 每轮 Distance 后 Bend |
| 刚性目标 | 自定义 Compliance=0；Chaos PBD Stiffness=1 |
| Bend 角度 | 有符号 atan2，平直静止角 0，误差修正项限幅 ±45° |

Chaos 使用不同的单位法线计算顺序、Bend 图着色排序及可能的 ISPC 执行路径，
所以有限次迭代的结果不承诺逐位一致。对非零 Compliance，这个硬约束基准不再等价，
不能直接照搬 Stiffness=1 作比较。

输入的重力和推力均为布料局部坐标；两侧的左右分离仅用于绘制，不参与物理计算。
两侧共用固定步长累加器：不是每个渲染帧各走一步。编辑器卡顿时最多追赶 8 步，
然后共同放弃多余的墙钟时间；比较应以屏幕中的模拟时间 `t` 为准。

## 操作

| 按键 | 两侧同步操作 |
| --- | --- |
| `Space` | 给全部非固定粒子增加 `(0,150,0)` cm/s 的速度 |
| `F` | 开关 0.5 Hz 周期加速度，幅度 `(0,150,0)` cm/s² |
| `G` | 开关重力 |
| `P` | 暂停 / 继续 |
| `N` | 暂停时推进一个完整物理时间步 |
| `R` | 重建两侧网格，清零速度和模拟时间，保留当前力与暂停开关 |
| `O` | 并排 / 重叠显示 |

Details 中可以修改网格、步长、迭代次数和外力。修改网格后必须按 `R`；
步长和外力参数下一步对两侧同时生效。可在 Play 前勾选 `Paused`，避免来不及观察初始状态。

建议先观察纯重力，再按 `P` 暂停、`Space` 推动、`N` 单步，最后用 `O` 比较偏差。

## 指标与已执行检查

屏幕持续显示：逐粒子位置 RMS / 最大差值、两侧最大速度、最大相对边长误差、固定点漂移、
实际步长与迭代次数。出现非有限位置或速度会暂停两侧，保留现场，`R` 可重新开始。
现有 `p.PBD.StageDebug`、`p.PBD.BendDebug` 及相关阈值仍然可用，它们记录的是自定义一侧。

`Scripts/create_cloth_comparison.py` 在真实 UE 运行时调用两个求解器，检查：

1. 无重力静止 60 步：无运动，位置 RMS 为 0。
2. 纯重力 300 步：位置差在本次报告的 0.0001 cm 精度附近。
3. 无重力、离面速度 120 步：末尾 RMS 约 2.2407 cm。
4. 重力 + 周期外力 180 步：末尾 RMS 约 0.6038 cm。

这四项的有限值、固定点保持和拓扑数量检查通过。后两项不是数值等价性通过证明。
当前默认 8 次迭代下，两侧纯重力最大边长误差在 5 秒时约 33%，
说明两侧都存在明显的有限迭代残差；一致并不表示已经达到了理想刚性约束。
可以同时提高迭代次数或减小步长进行进一步比较。

详细报告：`Saved/Tests/PBDChaosComparison.json`。

## 重新生成地图及运行检查

先编译工程，然后运行以下 PowerShell 命令。脚本只新建或更新专用测试地图。

```powershell
& 'D:\UE5\UE_5.8\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' `
    'E:\UEPJ\pdd_cloth\pdd_cloth.uproject' `
    -run=pythonscript `
    '-script=E:\UEPJ\pdd_cloth\Scripts\create_cloth_comparison.py' `
    '-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities' `
    '-DisablePlugins=ModelContextProtocol,MCPClientToolset' `
    -unattended -nop4 -nosplash -NullRHI `
    '-abslog=E:\UEPJ\pdd_cloth\Saved\Logs\ComparisonSetup.log'
```

这次构建使用独立模块文件名 `UnrealEditor-pdd_cloth-9081.dll`，避开原编辑器占用的 DLL；
构建成功后模块清单已经指向新文件。项目下次正常构建仍可使用普通 Build 命令。
