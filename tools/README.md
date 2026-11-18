# 从本地 STEP 生成组合仿真

准备 2026-11-16 的 `V2R1_END_EFFECTOR_COMPLETE_F_FRAME.stp`，或者交接包中的去除 STEP 头部个人元数据版本。脚本会校验 STEP 的 DATA 段。Windows PowerShell 中在仓库根目录执行：

```powershell
py -3.12 -m pip install -r .\tools\requirements.txt
py -3.12 .\tools\build_model.py 'D:\CAD\V2R1_END_EFFECTOR_COMPLETE_F_FRAME.stp' --output .\generated\v2r1
py -3.12 .\tools\build_scene.py .\generated\v2r1
py -3.12 .\generated\v2r1\preview.py --gui --jaw-mm 8
```

把示例 STEP 路径换成实际文件路径。输出含组合 URDF、网格、MuJoCo XML 和网格来源记录。左右夹指连同各自滑块和齿条移动。参数 `--jaw-mm` 是每侧相对 CAD 初始位置的仿真位移。两个 X3 视角是透视预览，镜头畸变和光学外参需另行测量。

组合模型使用名义法兰安装变换。夹爪的仿真行程和质量参数为离线预览值，实际行程、TCP 与开口宽度使用对应现场测量值。源码及生成的文件均用于离线仿真。[生成物来源和许可](NOTICE_GENERATED.md)
