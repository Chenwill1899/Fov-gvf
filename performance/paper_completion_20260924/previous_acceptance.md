# EGO1P1 论文方法验收 — 2026-09-24

**结论：完成本轮实现与验收；完整论文复现/导航验收未通过。**
用户明确选择“未观测空间一律不可通行；观测不足时停车，完整运行验收暂不通过”。
当前主入口已启用严格规则，保留失败测试，不把停车、零碰撞或旧版本成功运动当作导航成功。

## 分阶段结果

| 阶段 | 已实现内容 | 验收结论 |
|---|---|---|
| 1 保守观测与自由域 | 深度块最小值、未知拒绝、深度采集位姿、机体包络及方向扫掠证书 | 数值正/反例和机载拒绝契约通过；单视图证书保守，未做多视图/历史观测并集 |
| 2 锚定局部截面反馈 | 规则域、等势截面返回、chi、有限差分J、论文角域反馈、单一三维输出 | 解析直场/曲场及J u、跨片导数测试通过；不能替代连续模型证明 |
| 3 跨帧参考延续 | 场区间冻结、世界方向参考转图、可行延续、改向/失效重置 | 数值冻结与小角度相机旋转换图通过；未证明公共域全局跳变上界 |
| 4 粗细调和场 | 粗场源/目标条件、障碍无通量、细场边界耦合、源连通分量 | 数值残差和耦合约束通过；实际规则域可能不存在，必须停车 |
| 5 动态验证和消融 | 一阶响应、扰动输入、独立碰撞真值、ROS和Isaac实际主链 | 严格停车通过；障碍闭环到达失败，完整运行不通过 |

实现公式、离散近似与传感器假设见 [PAPER_IMPLEMENTATION.md](PAPER_IMPLEMENTATION.md)。

## 最终执行结果

证据目录：[performance/paper_acceptance_20260924](performance/paper_acceptance_20260924)。

| 检查 | 实际结果 |
|---|---|
| 主构建脚本 | 3个ROS包成功；build_final.log |
| CTest | **11/12通过，退出8**；paper_closed_loop_check失败，未改为预期失败或跳过 |
| 平台pytest | 14/14通过 |
| Shell/Python/差异 | 两入口bash -n、修改的运行Python AST、git diff --check通过 |
| 论文数值测试 | 直场横向误差3→1.0981；曲场3.08273→1.12845（1秒）；无反馈对照保持初值的至少98%；J u和场交界导数断言通过 |
| 真实ROS节点合成输入 | --paper探针退出0；前上、左上、后下、右下、斜向均返回UNOBSERVED_BODY_ENVELOPE、零矢量；视野外、深度/输入过期、松杆分别正确停车 |
| 最终Isaac Cloud | 六秒仿真、363帧、301条命令、200次有输入的算法调用；位置始终(-64,-11.2,2.8)，只有一种施加命令（零），ESDF阻断0次，主入口退出0 |

Isaac返回MANUAL_TIMEOUT表示按时退出，**不是导航成功**。其算法耗时约0.002ms是机体观测检查
提前拒绝的耗时，不能作为完整场求解50Hz运行性能。退出时仍有rosout context invalid告警；
控制器与命令桥均正常退出。原始日志完整保留。

## 障碍闭环为何失败

closed_loop_final.csv保留八个场景各500步（10秒）的结果。测试使用机体后方2m的外部相机，
仅用于给数值算法提供可观测起点；机载相机的位置没有改动。测试运动模型为tau=0.22s、
加速度上限1.2m/s²，碰撞真值独立，不使用ESDF保护。

- 空旷场景前进到x=9.56688m；改向场景到(4.73692,4.32901,0)m。
- 立柱场景停在约(2.0402,1.19078,0.0779)m，未达到原验收条件x>4.5m。
- 扰动场景停在约(2.11201,1.23559,0.2310)m；该位置的少量扰动位移属于测试注入。
- 八场景碰撞计数均为0；有障碍场景记录的最小净间距约0.727m。但停车也能零碰撞，不能据此判通过。
- 该严格版本的这些障碍场景未有效进入TRACKING规则反馈状态，消融结果不能支持“反馈性能优于无反馈”的结论；目前正面证据仅来自解析场测试。
- 立柱停车位置只剩少量角域边缘方向能满足严格扫掠前缀，粗网格保守降采样及规则截面要求下无法生成可用场，最终状态为NO_REGULAR_FIELD。日志还保留STATE_OUTSIDE_FREE_DOMAIN、OUTSIDE_FLOW_BOX等拒绝状态。

同版本独立闭环运行中，各场景计算均值约3–10ms量级；此处保存的最大值20.3624ms。
与Isaac并行跑CTest时耗时更高，不能以单次无负载耗时宣称硬实时保证。

## 实施中的失败与修正

初期遇到Eigen矩阵运算链接问题、低速换相机方向状态错误、目标附近扰动漂移、源区零梯度
停顿及前视距离过短，均在后续实现中修正。严格检查后的新失败另外保留，不能继承此前的
12/12或Isaac运动通过记录。曾使用过的单个长扫掠包围盒过于保守，现改为连续短段包围盒；
方向自由域与最终rollout使用一致余量。规则势场进一步采用C2重建，使规则区域内向量场C1。

第一次最终CTest有两项Python测试因用户目录NumPy二进制冲突而收集失败；设置
PYTHONNOUSERSITE=1后两项通过。没有改变依赖版本。当前剩余失败是闭环行为失败，不是依赖问题。

## 重现

在项目根目录执行：

```bash
bash scripts/build_isaac_ros_workspace.sh
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source /tmp/fov_gvf_ego1p1_isaac_install/setup.bash
ctest --test-dir /tmp/fov_gvf_ego1p1_isaac_build/pc_gvf --output-on-failure
/usr/bin/python3 -m pytest -q src/pc_gvf_platforms/test
ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tools/ros_vector_guidance_probe.py --paper
ISAAC_MANUAL_INPUT_MODE=trace \
ISAAC_INTENT_TRACE="$PWD/src/pc_gvf/test/fixtures/paper/isaac_strict_observation_trace.json" \
ISAAC_MANUAL_TIMEOUT=6 ISAAC_HEADLESS=1 FOV_GVF_RVIZ=false ROS_DOMAIN_ID=84 \
bash scripts/run_isaac_fov_gvf_navigation.sh
```

## 尚未满足的最终验收条件

1. 必须有实际观测证据覆盖机体包络和所需运动空间；目前四水平相机无法给出这种证据。未增加相机，未引入自由空间先验。
2. 在严格自由域中形成可用规则区域，并通过立柱/扰动/改向等闭环到达测试。不能靠降低安全余量、关闭规则或降低验收距离解决。
3. 需要足够有效的规则反馈运行样本，才可验收动态扰动恢复、跨帧延续和粗细场消融性能。
4. 论文式25–31的全域假设、误差上界及真实平台实验未验证，不能声称“完全符合论文全部内容”。

EGO1P0和其他项目未修改。此次代码未提交或推送GitHub；修改前完整备份为
`/home/starry/isaac-data/备份/EGO1P1_before_paper_20260923_205045/EGO1P1`。
