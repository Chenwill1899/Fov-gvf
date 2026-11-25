## GeoMonoPlan 第一组

更新：2026-10-24 14:21。按用户“全部分开、一组组来”，本轮完成官方发布包的三类场景；厂房/办公楼、公园/村落作为后续组，尚未开始。这里只适配地图，沿用现有ARL-Robot-1与c36控制链。

| 独立名称 | 飞行域（米，含边界留量） | 内容 | 碰撞栅格 | 当前实际证据 |
|---|---|---|---|---|
| `forest`（新入口） | 164×164×29 | 原分布794树、原地形/贴图 | 0.15m | 157.79m / 18.667仿真秒安全到达 |
| `clutter` | 218×218×39 | 原990组件、487树及混合/倾斜障碍 | 0.20m | 179.69m / 21.033仿真秒安全到达 |
| `garage` | 94×58×10 | 原3042组件、立柱、车道、标志、管线 | 0.15m | 41.90m / 7.050仿真秒安全到达 |

各图独立启动，任选一条，不并行运行：

```bash
cd /home/starry/isaac-data/EGO2P6
bash scripts/run_geomonoplan_scene.sh forest --mode manual
bash scripts/run_geomonoplan_scene.sh clutter --mode manual
bash scripts/run_geomonoplan_scene.sh garage --mode manual
```

键盘将`manual`换为`keyboard`；`1`第一人称、`3`第三人称、`V`切换。原`run_reference_scene.sh forest`仍是之前360×80×24m的林地，新`run_geomonoplan_scene.sh forest`加载GeoMonoPlan林地；原Cloud/lab/hangar/forest四图字节保持。默认请求10m/s、垂向1m/s、四相机320×240/90°/20m、0.48m物理球、c36参数保持。

车库总览仅作预览剖切，实际运行保留顶棚和外墙，物理不变。

[HTML实际预览](http://127.0.0.1:8768/geomonoplan/)提供三图各地面视角/总览/无人机近景和对应启动指令。默认完整材质RaytracedLighting/FXAA；为替代未导出的Unity烘焙环境光，加入0.55倍原贴图比例环境项，保留纹理、透明裁切、阴影和度量深度，不作真实光照标定。MinimalRendering的模式2/3静态深度一致，但RGB质量下降，因此未采用默认。RTX5070高画质林地/杂障实测约13墙钟帧/秒：10m/s指仿真速度，不能据此承诺等速实时播放。新入口可重建缓存放`/tmp/ego2p6_geomonoplan_cache`，永久地图/纹理仍在项目中，旧入口默认缓存路径不变。

资料来自[GeoMonoPlan官方AeroScene v1.0.0](https://github.com/TJU-Aerial-Robotics/GeoMonoPlan/releases/tag/v1.0.0)，490823468字节ZIP的SHA256摘要`08f3f3170b03fd4a2af41a9febae8fa5aea33ee4d2fa43afaef94523c3e0d42a`已核验。只读取Unity2022.3.62f3序列化层，不执行其二进制/MonoBehaviour；用LOD1、512px纹理与原组件/树木变换转换为米制Z-up居中坐标。车库原层高约3m，Z×2适配为约6m，XY布局保持；其它地图尺度1。Unity动画/风和原网络控制器不导入。仓库MIT声明不等于所有第三方场景素材均MIT，来源/许可边界保留在`scenes/geomonoplan_assets/NOTICES.txt`和provenance，未上传或发布。

实体碰撞采用组件凸包、树冠按1m高度分组的保守包围盒，以及完整地形顶点的分块包围盒，逐三角形/变换和占据检查通过；小枝叶间隙可能被保守封闭，非精确网格面物理。所有可见几何与四相机已分别核验；在线控制器不读取这些离线物体/连通路径，只接受原深度与初始认证球。真实smoke只给最终目标，不给离线A*路径。

当前3包、5新图检查、12原图回归、15原起飞、15新起飞、7图ARL外观、6组入口检查通过；三图9张原生RGB和12路深度可见/隐藏A/B均无机身遮挡。3次headless穿越均峰值约10m/s、独立审计全部通过，原件路径和全量指标见[VALIDATION.json](../../../EGO2P6/docs/performance/geomonoplan_scenes_20261024_124740/VALIDATION.json)。没有执行完整course、任意路线、长期真人手柄或重跑63核心/174工具套件，不转移旧图成绩。开发期转换/预览失败、暗光与简化模式未采用结果都保留在报告和归档中。

重新生成：`OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1 ../isaacsim/python.sh scripts/build_geomonoplan_scenes.py`，然后重建运行程序。源NPZ/JSON/51纹理随`scenes/geomonoplan_assets`保留，不依赖/tmp原ZIP。原四图生成器与外观更新器均保留新三图的起飞白名单，不会在重建旧图时丢失新图身份。



2026-10-23 12:47 — **四张地图已换用完整ARL-Robot-1外观。** Cloud、lab、hangar、forest共用官方模型，保留10m/s请求、0.48m物理机体与四相机/原控制器。3包、外观/几何/起飞检查通过，实际四图渲染及16路深度A/B无机身遮挡。新增林地飞行73.241m后触发COLLISION_GUARD、未到达，不能继承旧349.255m成功；完整巡航/长期手柄未验收。[实际效果](../../../EGO2P6/docs/performance/arl_visual_20261023_122132/preview/forest_robot.png) · [完整验证与失败记录](../../../EGO2P6/docs/performance/arl_visual_20261023_122132/VALIDATION.json)。

# 三类参考场景与无人机运行

2026-10-22。已阅读当前 c36 算法、四相机感知、起飞认证、仿真执行器、目标/手柄入口、独立审计及项目历史。本次在 EGO2P6 增加三套可运行场景，原 Cloud 地图及原手柄/目标入口继续保留。EGO3P0、其它历史版本和服务器未修改。

## 项目进展的当前口径

主链为四路水平深度 → 二维深度方向候选/角域后备 → 三维运动及完全制动认证 → 世界系速度执行；有限未知探索、局部恢复、真实发布历史保留。项目里的 EGO2P6 是当前工程版本名，不表示直接运行下面参考资料的 EGO-Planner 算法。

原地图有效 50 次成功为 47 次 RTX5090 与 3 次本机补跑，原始 55 次飞行、5 例排除保留。卡停、计算长尾、任意路线和真人长期手柄操纵仍未验收。这些成绩不能转移给新地图。新地图结果单列于 `performance/reference_scenes_20261022/VALIDATION.json`。

## 资料来源与还原精度

三张用户原图原样保存在 [reference_scenes](../reference_scenes)，完整检索清单和访问限制见 [sources.json](../reference_scenes/sources.json)。目前没有原始测绘、相机位姿、尺度标定或完整点云，因此下面是**参考场景重建**，不是一比一复原。所有长度、障碍坐标和线路均为可调整的工程估计。发光曲线按飞行轨迹叠影理解，不作为空中实体障碍。

| 参考图 | 更完整资料 | 本次使用与确定程度 |
|---|---|---|
| 室内箱体训练场 | [UZH 原始实验视频](https://www.youtube.com/watch?v=ZPI8U1uSJUs)；[大学官方说明与高清图片](https://news.uzh.ch/en/articles/2021/Drone-Race.html)；[BMSWEB 完整现场报道](https://www.youtube.com/watch?v=qEVx5t5l6Zw) | 补充室内训练场、门框和箱体结构；尚未确认用户 18:08 缩略图的准确原视频。BMSWEB 标题和现场报道说明已核对，广告播放限制了逐帧检查。 |
| 绿色门框机库赛道 | [UZH 原始视频](https://www.youtube.com/watch?v=fBiataDpGIo)；[Nature 原始论文](https://www.nature.com/articles/s41586-023-06419-4)；[实验室资料汇总](https://rpg.ifi.uzh.ch/research_drone_racing.html) | 用作大型机库竞速场景参考；保留绿色门框、立柱、桁架与上下层穿门。具体门位与图中场馆是否完全相同未证实。 |
| 带支撑杆林地 | [EGO-Planner 原始视频](https://www.youtube.com/watch?v=UKoaGW7t7Dk)；[官方仓库提供的 Bilibili 链接](https://www.bilibili.com/video/BV1VC4y1t7F4/)；[官方仓库](https://github.com/ZJU-FAST-Lab/ego-planner) | 同类林间飞行的补充资料，不能认定为“Flight 2”的出处；树干、白色树基与三角斜撑主要依据用户原图。Bilibili 直接读取返回 412，未声称已看完整视频。 |

不下载或重分发第三方视频。lab/hangar由本项目生成器创建；forest使用随项目保存的Poly Haven CC0树木和地面纹理，运行时无需联网。

## 模型与碰撞

当前四图的无人机外观采用[NVIDIA官方ARL-Robot-1](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_robots_aerial.html#ntnu)，来源NTNU Autonomous Robots Lab，BSD-3-Clause许可随`scenes/robots/arl_robot_1/LICENSE`保存。原始base几何统一缩放1.1632582303并下移0.2163112768m，使全部渲染顶点位于0.46m球内、最高点低于四水平相机平面0.04m；既有0.48m物理球、四相机内外参、运动控制和10m/s上限保持。外观完整内嵌各USD，材质使用Isaac自带OmniPBR.mdl，运行无需联网；`visual.usdc`与`provenance.json`供重新生成使用。没有导入ARL原动力学或关节，当前仍为既有运动学仿真。

新外观的实际渲染、16路深度A/B及CPU射线检查通过。新增forest smoke在73.241m、8.25仿真秒后触发1次安全保护并停止，未到达；源/安装/相机/冻结身份匹配，失败根因尚未确定。下文旧单条成功保留为当时外观的历史证据，不能视为新外观穿越通过。

| 名称 | 尺寸（估计，米） | 主要结构 | 碰撞实体数 |
|---|---|---|---:|
| `lab`（v3） | 140 × 100 × 7 | 150 组叠箱、100 门、外围围网、墙体、顶棚、横梁、场馆照明 | 1238 |
| `hangar`（v2，无顶棚） | 40 × 28 × 9 | 7 门（含上下层组合）、立柱、屋架、护栏、墙体与照明 | 106 |
| `forest`（v3，放大并延长） | 360 × 80 × 24 | 288 棵乔木、144 丛低矮植被、66 岩石、24 倒木、纹理地面 | 19551（含19460个保守植被凸包） |

第1张图的长、宽仍为原来的5倍：140×100×7m，障碍尺寸保持。当前 v3 改用固定种子 `20261022` 的分区随机布置和邻近间距调整，替代 v2 的5×5重复模板。25个28×20m区域各含6组叠箱、4个门框，区域之间没有隔墙，位置各不相同；箱体高度及门框高度组合保持，门框朝向和箱体配色随机分配。障碍中心最小间距6.200m、中位6.465m，最近邻间距变异系数8.10%；按每组障碍水平外包围盒计算的最小组间净距2.504m。起飞点和目标附近保留净空，但没有为测试清空整条走廊。

起点仍为 `(-67,-45,3)`，短程目标仍为 `(66,-45,3)`。现在两点直线会经过障碍，`smoke` 仅将最终目标交给原控制器，由深度感知闭环避障。生成器另用离线解析几何检查连通性，并为 `course` 编排31个停稳式航点，覆盖场地的下、中、上部；这些静态航段已检查净空，完整course尚未实际飞行。连通性辅助路径保存在manifest，仅作离线证据，不输入在线控制器。旧v2模型及其133m测试原件保留在HISTORY快照与原实验目录。

机库 `hangar` 已按用户要求移除实体天花板 `Roof`，让天空光和太阳光进入场内；碰撞栅格同步移除顶部占据，门框、立柱、桁架和其它106个实体保持。仍为40×28×9m，原工作高度范围保持，去顶棚不等于扩大飞行域。

每张地图位于 `scenes/reference_<名称>/`，包含可编辑的 `scene.usdc`、`manifest.json` 和 `occupancy.bin`；forest还需一同保留 `assets/` 本地纹理与资源。树木网格已内嵌USD并以实例复用，外部资源SHA也参与入口校验与运行冻结。米制、Z 向上，四路 320×240/90°水平深度相机从原 Cloud 模板复制。`manifest.json`保存全部物体参数、起终点、航点、门框开口、网格尺寸和 SHA-256。

lab/hangar视觉和碰撞使用相同的箱体、球体与胶囊体定义。forest树木为带树皮与叶片透明贴图的真实网格，碰撞另用空间分组的26方向保守凸包，完整覆盖每个渲染三角形，并与0.15m保守占据同步。凸包包含少量透明叶卡区域和叶隙，不应理解为逐叶空隙精确碰撞；树木不再使用实心球冠。地面仍为平整实体加土壤纹理，未建模风、物理地形起伏、动态物体或柔性植被。

原程序只接受固定 Cloud 校验值的起飞认证，新增三图时已扩展**完整文件校验白名单**。新入口使用原有半径 2.5m 的初始自由球，且仍逐体素检查；三图起飞高度设为 3m，确保初始球不穿地面。全图碰撞数据用于仿真保护和离线审计，不作为飞行中控制器的全图规划输入。较低起飞点配 1.2m 初始球曾导致控制器在起点附近循环，失败原始记录保留。

## forest v3：两倍缩放与三倍延长（2026-10-23，当前版本）

按用户要求，先把原60×40×12m的长宽高同时×2，得到120×80×24m，再沿X长边延长至360×80×24m。中央原布局完整保留，全部树木、低矮植被、岩石和倒木的位置、尺寸、高度均准确×2，朝向和树种保持；土壤纹理的物理尺度也×2。无人机、相机、速度与安全参数没有缩放。

西/东各新增120m长区域，沿用68%林团+32%散生树的采样、最小间距、空隙和林下植被规则，分别使用独立种子20261024/20261025，没有重复复制中部布局。最终288树、144丛低矮植被、66岩石、24倒木；树高约13.62–22.86m，树干中心最小间距6.101m。起点(-172,0,6)、目标(172,0,6)，默认请求10m/s。边界、碰撞、起飞身份与巡航航点同步。

保持0.15m占据精度，网格2407×541×167，共217465229格；地图扩大使启动时距离场生成和内存开销上升。几何连通路线只用于离线检查，在线控制器仍只接收目标和四相机深度。12组几何/原布局缩放/独立新区/资源回归、15起飞认证、3包构建、Isaac原生RGB及四路320×240深度通过。单条实际穿越通过：349.255m、39.900仿真秒，峰值10.000m/s，保护介入0、扫掠重叠0，全部独立审计通过。完整course、任意路线和长期手柄尚未验收。

[新版全景](../../../EGO2P6/docs/performance/reference_forest_2x_extend3x_20261023_103953/preview/forest_overview.png) · [林内视角](../../../EGO2P6/docs/performance/reference_forest_2x_extend3x_20261023_103953/preview/forest_eye_level.png) · [完整验证](../../../EGO2P6/docs/performance/reference_forest_2x_extend3x_20261023_103953/VALIDATION.json)。旧v2及其短程成绩、原始模型快照保留，不能当作v3成绩。原手柄启动命令保持不变。

## forest v2：自然林地（2026-10-23，历史版本）

用户要求去掉过于规则的种植布局后，参考 [FlightBench森林场景](https://thu-uav.github.io/FlightBench/#scenarios)、[Infinigen](https://github.com/princeton-vl/infinigen) 和 [ForestGenerator](https://github.com/callumnewlands/ForestGenerator) 的自然植被组织方式，改为固定种子20261023的林团与散生树混合分布，并留出自然空隙。没有导入或运行上述项目的完整地图/生成器，也没有为测试清空直线走廊。

实际资产为 [Tree Small 02](https://polyhaven.com/a/tree_small_02)、[Island Tree 01](https://polyhaven.com/a/island_tree_01) 和 [Forest Ground 04](https://polyhaven.com/a/forest_ground_04)，遵循 [Poly Haven CC0许可](https://polyhaven.com/license)。作者、原始下载SHA和转换记录位于 `scenes/reference_forest/assets/LICENSE.txt` 与各 `provenance.json`。原始.blend、下载清单和旧forest模型保留在HISTORY快照。

96棵乔木最小树干中心间距3.059m、中位3.475m；树形、360°朝向和缩放随机，另有48丛低矮植被、22块岩石和8根倒木。原白色树基、斜撑、球冠和人工远墙移除。两个共享树木模板分别78967/63576三角形，以USD实例复用；保守碰撞共6460个小凸包。起终点、工作域和10m/s默认请求保持。离线连通路径54.907m、7航段，26点course航线仅作停稳巡航；smoke运行仍只发送最终目标，不向在线控制器提供全图或离线路径。

已完成3包构建、11项几何/材质/资源回归、15项实际起飞认证、全部渲染顶点与碰撞来源逐点对齐（误差0），以及CPU离线画面目视检查。[完整验证](../../../EGO2P6/docs/performance/reference_natural_forest_20261022_212047/VALIDATION.json)。[总览](../../../EGO2P6/docs/performance/reference_natural_forest_20261022_212047/preview_isaac/forest_overview.png)和[林内视角](../../../EGO2P6/docs/performance/reference_natural_forest_20261022_212047/preview_isaac/forest_eye_level.png)为最终USD的Isaac原生渲染，四路320×240深度均有有效输出；较早的preview_cpu仅为CPU文档预览。最初另一EGO3P0仿真占用共享通道，期间只做CPU检查；10:15:21确认通道释放后独占非阻塞锁预览，再通过原无排队入口执行一次穿越，未打断其它进程。**新版单条实际穿越成功：53.837m、8.500仿真秒、峰值9.99934m/s，保护介入与连续扫掠重叠均0，独立审计全部通过，保守机体净空下界0.104995m。** [原始飞行审计](../../../EGO2P6/docs/performance/reference_scene_runs/forest_smoke_20261023_101620_183830/audit.json)。短程headless统计Isaac墙钟47.01帧/s，完整避障均值1.068ms、最大19.962ms，不是固定60Hz或稳定1ms验收。完整course、任意路线和长期真人手柄仍未验证，旧v1成绩保持独立。

资产复现：用Blender运行 `scripts/prepare_forest_assets.py` 并指定快照中的 `polyhaven_sources`；再用Isaac Python执行 `scripts/build_forest_collision_assets.py` 和 `scripts/build_reference_scenes.py --scene forest`，最后重建运行包以更新精确文件白名单。必须保留assets中地面贴图。CPU预览入口为 `scripts/preview_reference_forest_cpu.py`，不会启动ROS或占用GPU控制。

## 运行

在项目目录执行，三图分别用 `lab`、`hangar`、`forest`。默认请求上限 **10m/s**，与 EGO2P6 保存配置一致；可用 `--speed 3` 显式降速。起步、近障、转弯和制动时，控制器会按原有约束降低实际速度。此前三图验证采用 3m/s；默认值恢复为 10m/s 不代表已完成三图的 10m/s 飞行验收。

```bash
cd /home/starry/isaac-data/EGO2P6

# 手柄模式：Isaac + RViz，右杆平移，左杆升降/偏航，回中解锁
bash scripts/run_reference_scene.sh lab --mode manual
bash scripts/run_reference_scene.sh hangar --mode manual
bash scripts/run_reference_scene.sh forest --mode manual

# 无手柄时使用键盘模式，按原仿真窗口提示操作
bash scripts/run_reference_scene.sh lab --mode keyboard

# 短程自动目标：同一手柄避障策略，目标意图替代摇杆
bash scripts/run_reference_scene.sh hangar --mode smoke --headless

# 多航点场景巡航：逐点停稳，再继续下一段
bash scripts/run_reference_scene.sh forest --mode course --timeout 900

# 只显示地图和协议，不启动仿真
bash scripts/run_reference_scene.sh lab --mode smoke --dry-run
```

短程路线连接各图预设起终点，随机lab需要自主绕开沿途障碍；不是全图覆盖。`course` 是停稳式多航点巡航，不是连续高速竞速或穿门正确性验收。静态线路净空检查不等于闭环执行成功；若控制器停滞，失败和全部轨迹保留。

共享锁 `/tmp/fov_gvf_user_navigation.lock`、ROS 域 42、单控制发布者检查保持。一次只运行一个仿真，关闭窗口或 Ctrl+C 退出。输出位于 `docs/performance/reference_scene_runs/<地图>_<模式>_<时间>_<进程>/`，包含实际参数、深度相机标定、控制器身份、完整轨迹、控制计算统计、结果及独立审计。失败目录不自动删除。手柄/键盘需真人操作，未把自动目标测试当作真人手柄验收。

## 重建与修改

模型生成源是 `scripts/build_reference_scenes.py`；几何规则在 `scripts/reference_scene_geometry.py`，lab随机采样与离线路线编排在 `scripts/reference_lab_layout.py`。修改物体位置、尺度、门位或树木随机种子后需重新生成地图、起飞认证白名单并重新构建。不要仅修改 manifest 中的尺寸或校验值。

```bash
cd /home/starry/isaac-data
PYTHONNOUSERSITE=1 isaacsim/python.sh EGO2P6/scripts/build_reference_scenes.py --scene lab
EGO2P6_BUILD_TESTING=ON bash EGO2P6/scripts/build_isaac_ros_workspace.sh
PYTHONNOUSERSITE=1 isaacsim/python.sh EGO2P6/scripts/test_reference_scenes.py
```

原图与场景对应关系及每个坐标都可继续用实测资料修正；重新生成会使旧图 SHA 与旧运行结果保留各自的历史含义。

## 实际预览

以下为 Isaac 从交付模型渲染的图片。总览为方便看布局，临时隐藏前墙与顶棚，lab还隐藏横梁和顶部灯具；该剖视修改不保存进场景。第一人称图片使用完整环境。

![随机且均匀布置的箱体训练场剖视总览](../../../EGO2P6/docs/performance/reference_lab_random_20261022/preview/lab_overview.png)

![去掉天花板后的机库场内视角](../../../EGO2P6/docs/performance/reference_hangar_open_roof_20261022_210717/preview/hangar_eye_level.png)

![林地第一人称视角](../../../EGO2P6/docs/performance/reference_scenes_20261022/preview/forest_eye_level.png)

## 当前随机布局验证（v3）

新图以默认10m/s请求上限完成一条133m名义穿越：16.917仿真秒到达，实际路程133.522m，峰值10.000m/s，独立解析几何机体扫掠净距下界0.397m；碰撞保护/扫掠重叠均0，全部29项独立审计通过。三包构建、8组几何/分布/可复现性检查、15项起飞认证、63项核心检查均通过。Isaac实际模型预览和四路深度已验证。见[随机布局验证](../../../EGO2P6/docs/performance/reference_lab_random_20261022/VALIDATION.json)。完整course和真人手柄操作未验收，旧图成绩不转移至新图。

## 扩大后的第1张地图历史验证（v2）

扩展图 `reference_lab_v2` 已以默认10m/s请求上限完成一条133m名义穿越：16.117仿真秒到达，实际路程132.659m，峰值10.000m/s，外部碰撞保护和独立扫掠重叠均0；全部独立审计项通过。三包构建、6组几何、15项起飞检查通过；63项核心检查中，首次61通过/2项因Python环境冲突导入失败，按项目环境重跑这2项后通过。详见[扩展图验证记录](../../../EGO2P6/docs/performance/reference_lab_5x_20261022/VALIDATION.json)。完整蛇形巡航和真人手柄操作未验收。

## 去顶棚机库验证（hangar v2）

106个保留实体、全部门框/航点/工作域以及267项机器人/相机属性与旧版一致；删除149,174个仅属于顶棚的占据单元，变更层中心高度9.0–9.3m。8组场景几何、15项起飞认证、三包构建通过；实际Isaac预览显示采光改善，四路320×240深度有效。地图元数据使用自身身份，并去掉从Cloud模板继承的高度拉伸标签。见[验证记录](../../../EGO2P6/docs/performance/reference_hangar_open_roof_20261022_210717/VALIDATION.json)。本次没有新增闭环飞行，初版hangar的3m/s结果继续作为旧地图历史。

## 初版三图历史验证（室内图为原28×20m）

| 地图 | 到达时间（仿真秒） | 实际路程（米） | 机体扫掠保守净距下界（米） | 外部碰撞保护 |
|---|---:|---:|---:|---:|
| lab | 8.550 | 20.657 | 0.420 | 0 |
| hangar | 11.883 | 30.655 | 0.320 | 0 |
| forest | 18.883 | 51.655 | 0.640 | 0 |

下表历史结果均为请求上限3m/s、保持原c36策略的单段短程测试，不作为扩大后室内图的验收结果。63项核心、174项平台/工具、6组场景几何与15项起飞认证检查通过。7次尝试包含1次无飞行初始化错误、2次未到达、4次到达（其中lab额外复测一次）；全部记录保留。中间三次到达的报告序列化错误另存并重算，不改原轨迹。详见[完整验证](../../../EGO2P6/docs/performance/reference_scenes_20261022/VALIDATION.json)。
