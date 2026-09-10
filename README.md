# pdd_cloth

项目的最终设计方向与阶段性验收标准见 [PBD Cloth Design Goals](Docs/PBDClothDesignGoals.md)。

新增、删除或重命名 C++ 源文件后，运行以下命令刷新 Visual Studio 工程文件：

```powershell
powershell -ExecutionPolicy Bypass -File Scripts/refresh_vs_project.ps1
```

默认引擎路径为 `D:/UE5/UE_5.8`，可通过 `-Engine` 指定其他路径。脚本显式生成 VS 2022 格式，避免默认配置生成 VS Code 工程。VS 提示外部工程发生变化时选择重新加载；若未提示，关闭并重新打开 `pdd_cloth.sln`。这一步更新文件列表及 IntelliSense 配置，不编译 C++。
