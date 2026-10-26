# CAD display files / CAD 显示文件

**License: CC BY-NC 4.0 — non-commercial use only** (see [LICENSE](LICENSE)). The rest of this repository is Apache-2.0. These files are here only to make simulation and training convenient. Nothing in the training pipeline needs them: delete this folder if you want a tree that is Apache-2.0 only (only the fall / get-up probes use them, and two unit tests skip themselves when the folder is missing).

| File | What |
| --- | --- |
| `meshes/*.stl` | binary STL (metres) converted from the XGO-Rider2 CAD assembly v2026.09.18 |
| `rider_measured_cad.xml`, `rider_cad.xml` | the models `../rider_measured.xml` and `../rider.xml` drawn with those meshes |
| `cad_inspection.json` | the component list of the assembly with bounding boxes, and the SHA-256 of the STEP file (the 138 MB STEP itself is not part of this repository) |

The triangulation is coarse (linear deflection 0.3 mm, angular deflection 0.4 rad) and a few STEP faces could not be triangulated, so use the meshes to **draw** the robot, not to measure it.

| Mesh | Part | STEP components | Triangles |
| --- | --- | --- | --- |
| `base.stl` | body, battery, compute module, boards | 255 | 100,000 |
| `leg12.stl`, `leg22.stl` | the two legs | 17 each | 12,000 / 11,999 |
| `wheel11.stl`, `wheel21.stl` | the two wheels | 10 each | 24,958 / 30,000 |

Who uses them:

* `rider_measured_cad.xml` and `rider_cad.xml` — the same bodies, masses, inertias, joints and (primitive) collision geometry as `../rider_measured.xml` and `../rider.xml`, plus these meshes as contact-free visual geoms (`contype=0`, group 2); the placeholder visuals of the primitive model are dropped and its collision geoms are made transparent. `rider_sim/env2.py` and `env.py` load the CAD version when they render (`render_mode`, e.g. `viewer2.py`) and the primitive model otherwise (also when this folder is deleted), so **training and evaluation never see the meshes**. `tests/test_measured_calibration.py` and `tests/test_simulation.py` check that the two models have identical physical parameters (they skip themselves without this folder).
* `../../tools/shell_model.py` (used by `getup_probe.py` and `fall_angle_probe.py`) — the convex hulls of `base`, `leg12` and `leg22` as the collision shell for falls and get-up experiments. These two probes need this folder.

`meshes/provenance.json` records the source file name and how the components were grouped.

---

**许可证：CC BY-NC 4.0，仅限非商业使用**（见 [LICENSE](LICENSE)）。仓库其余部分是 Apache-2.0。这些文件只是为了方便大家仿真和训练。训练流程不需要它们：想要一棵纯 Apache-2.0 的目录树，直接删掉这个文件夹就行（只有摔倒/起身探针会用到它们，两个单元测试在没有这个文件夹时会自动跳过）。

* `meshes/*.stl`：二进制 STL（单位：米），由 XGO-Rider2 的 CAD 装配体 v2026.09.18 转出。三角化比较粗（线性偏差 0.3 mm，角度偏差 0.4 rad），少数 STEP 面无法三角化，所以网格用来**画**机器人，不要拿来测量。
* `rider_measured_cad.xml`、`rider_cad.xml`：刚体、质量、惯量、关节和（简单的）碰撞几何与 `../rider_measured.xml`、`../rider.xml` 完全相同，只是多了这些不参与接触的显示网格（`contype=0`，group 2），简单模型里的占位外观被去掉，碰撞几何被设为透明。`rider_sim/env2.py`、`env.py` 在渲染（`render_mode`，比如 `viewer2.py`）时加载 CAD 版，其它时候（包括删掉这个文件夹之后）用简单模型，所以**训练和评估从来不会看到这些网格**。`tests/test_measured_calibration.py` 和 `tests/test_simulation.py` 检查两个模型的物理参数一致。
* `cad_inspection.json`：装配体的组件清单（含包围盒）和 STEP 文件的 SHA-256；138 MB 的 STEP 本身不在本仓库里。
* `../../tools/shell_model.py`（`getup_probe.py` 和 `fall_angle_probe.py` 用它）：把 `base`、`leg12`、`leg22` 的凸包当作摔倒和起身试验的碰撞外壳。这两个探针需要这个文件夹。
