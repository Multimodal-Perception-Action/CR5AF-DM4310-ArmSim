# V2R1 末端零件

六个单件从 2026-11-16 的 V2R1 Inventor 总装按装配件名称逐个导出，保留毫米单位和装配坐标。

| 零件 | STEP | STL |
| --- | --- | --- |
| 直连底座 | [STEP](parts/01_Direct_Base.step) | [STL](parts/01_Direct_Base.stl) |
| D8 保险件 | [STEP](parts/02_Fuse_Cartridge_D8.step) | [STL](parts/02_Fuse_Cartridge_D8.stl) |
| 定位销 | [STEP](parts/03_Cartridge_Retaining_Pin.step) | [STL](parts/03_Cartridge_Retaining_Pin.stl) |
| Short45 相机支架 | [STEP](parts/04_Camera_Module_Short45.step) | [STL](parts/04_Camera_Module_Short45.stl) |
| 左垫片 | [STEP](parts/05_Side_Pad_L.step) | [STL](parts/05_Side_Pad_L.stl) |
| 右垫片 | [STEP](parts/05_Side_Pad_R.step) | [STL](parts/05_Side_Pad_R.stl) |

六个 STL 均通过网格封闭检查。对应 STEP 文件可回读为有效几何，见 [导出记录](part_export_checks.json)。这是 CAD 交接，实际装机仍要核对孔位、紧固件和材料。11 月 14 日 SA-V2 报告的 D8 保险件属于更早的冻结几何。

六件为项目设计。Dobot 本体的 MIT 许可范围见 [本体模型说明](../models/README.md)。
