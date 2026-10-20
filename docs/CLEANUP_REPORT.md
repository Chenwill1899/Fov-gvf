# EGO2P6 本机精简记录

更新时间：2026-10-19 20:19。用户明确授权删除与运行无关冗余，范围只在本机P6。清理前实际占用约32.64 GB。最终容量与验证以本文末尾“完成结果”为准。

## 保留

62份原有生产src文件字节不变，核心C++/头文件、Cloud/平地场景和保存profile身份保持。保留四水平深度/历史观测、完整运动与制动认证、有界主动探索、手柄与目标入口、ROS消息/桥接/显示、当前本机安装、必要算法回归与夹具、当前5090 c36部署工具及少量同步元数据、现有Git提交/分支和WORK_LOG。

## 删除与简化

- 删除继承的performance大批量CSV、JSON、原始二进制重放、冻结源码/ELF、图片、旧复制备份和旧实验脚本。本机P5仍保留原始成功/失败实验与录像，未修改P5。
- 删除runtime中的旧服务器产物、imported_local_ubuntu_from_ego2p5、旧重放/缓存/kit、profile/source_backups、旧构建/日志；活动本机install保留。完成回归后移除可重建build与验证缓存。
- 删除旧deployment/rtx5090候选与过往EGO版本说明、历史分析/消融/探针工具；保留当前deployment/local_ubuntu与rtx5090_c36。
- Git整理不改提交/分支，清理5878个无引用对象；对象在P5中逐ID存在核验通过，P6全部refs前后一致，git fsck连通性检查通过。Git约505 MiB降至15 MiB。
- 运行路线由performance迁至config/routes.json，基础协议迁至config/navigation_protocol.json；回归地图及其颜色依赖迁至src/pc_gvf/test/fixtures/verified_maps，证据汇总模块迁至tools/summarize_trial_evidence.py。避免运行和回归依赖已删除的实验路径。
- 大型二进制诊断重放默认关闭，EGO2P6_RECORD_REPLAYS=1显式开启；保留全量CSV与独立审计，未修改几何算法、风险预算或速度参数。默认构建BUILD_TESTING=OFF，EGO2P6_BUILD_TESTING=ON可恢复全量回归构建。
- 修复speed_benchmark.py既有argparse帮助字符串95%的百分号转义；当前远端适配工具兼容已经修复的源码。未修改服务器。

## 实际验证与初次失败

- 测试模式3包重新构建成功；核心完整CTest63/63、工具/平台pytest174/174通过。
- 首次pytest未设置本机包路径而收集失败；第二次只设置PYTHONPATH却漏加载ROS环境也收集失败；完整加载本机environment、Humble和安装后174通过，修复后再次174通过。
- 初次CTest61/63，两个observed_space回归因迁移时遗漏derived map的height_colors.usdc失败；从P5只读复制完整颜色夹具后全量63/63通过，生产地图认证代码未放松。
- 8组源码/安装实参相等；RViz构造4组、显示选项6组通过；真实R07 Shell最终exec截获、手柄重放关闭/开启2组截获均通过。没有启动ROS节点或Isaac，没有新导航飞行。
- 3个CLI帮助、5条路线配置、4个ELF依赖检查通过。首次CLI发现既有百分号转义错误，修复后通过；首次ldd未加载ROS环境触发缺库，加载实际运行环境后通过。
- Git初检解析器只匹配英文而漏读中文“不可达”输出，断言失败；Shell继续执行了本任务已授权的无引用对象清理。随后修正解析，对5878个原对象ID逐一核对全部存在P5、全部refs保持且fsck通过，未删除提交或暂存区引用对象。初次日志和修正结果均保留。
- 精简前后31项核心/场景/profile SHA不变；额外从精简前小源码压缩包逐文件核对62份生产src字节全部一致。
- 默认BUILD_TESTING=OFF的全新干净构建3包成功（1分59秒）；安装消息与平台模块、4个ELF依赖、安装链接以及8参数/4 RViz/6选项再次核验通过。删除build后实际source安装并导入消息/桥接/显示模块仍通过。未执行新飞行。

## 恢复与证据

精简前小源码/配置/文档压缩包、原Git状态/refs、每个删除路径的类型/大小/inode/时间清单、各阶段日志和SHA证据位于：

`/home/starry/isaac-data/HISTORY/snapshots/EGO2P6_cleanup_20261019_201138`

其中current_source_before.tar.gz不包含30GB历史原始数据；大数据来源仍为../EGO2P5，不能认为每个P6适配历史字节都有独立整树备份。源P5与既有HISTORY保存点保留。不会为本次精简把大数据移到同一分区来假装释放空间。

原复制/性能结论保存在WORK_LOG、HISTORY和来源P5；来源正式20/20仍属P5指定范围，稳定1ms/无停顿/任意路线/真人长期手柄未验收。本机本次精简没有同步修改服务器现有c36部署。

## 完成结果

完成时间：2026-10-19 20:20。本机P6实际占用由 **32.640 GB** 降至约 **0.138 GB（131.2 MiB）**，项目净减少约 **32.502 GB，99.58%**。分区普通用户可用约 **34.99 GB**。日志及记录写入后可能有数十KiB差异。

清理前的小源码/配置/文档和清单/日志共约8 MiB，存于HISTORY保存点；没有把30GB大数据挪到同一分区。当前src约49 MiB，其中约41 MiB是必要回归地图夹具；场景约35 MiB、独立安装约31 MiB、Git约15 MiB。后续图形缓存按需再生。

## 2026-10-19 20:46 继续简化：恢复五目录结构

用户要求继续简化并解释新增目录。本机根目录现在仅performance、scenes、scripts、src、tools五个主要文件夹（隐藏.git保留版本记录）。本轮主要合并分类和重复入口，未改变核心算法；当前约137.4MB。

| 原顶层目录 | 原作用 | 本次处理 |
|---|---|---|
| config | 路线/基础协议 | 并入src/pc_gvf/config |
| profiles | 当前手柄/目标参数 | 并入scripts/manual_profile.env |
| deployment | 本机/服务器入口、适配与备份 | 本机入口并入scripts；服务器工具与旧入口备份归档HISTORY保存点 |
| runtime | 生成的安装程序、构建、缓存/日志 | 统一改为performance/runtime；按新前缀全新构建，验证后删除旧安装及新build/测试输出 |

新入口：bash scripts/run_manual.sh；目标bash scripts/run_goal.sh，EGO2P6_GOAL_ROUTE指定r01–r10或route300。构建命令仍为scripts/build_isaac_ros_workspace.sh；新参数源scripts/manual_profile.env，运行环境scripts/environment.sh。CMake同步安装三份配置文件，不留重复兼容目录。README/算法状态/手柄说明已去除当前入口的旧路径。

验证：新前缀从空目录3包构建成功（测试模式），核心63/63、工具平台174/174、8参数、4 RViz/6选项、手柄二进制重放开/关2组及R07真实Shell最终exec截获通过；11路线/语法、30项核心/地图SHA和profile纯路径差分检查通过；4 ELF依赖、3安装配置、消息/平台模块导入和链接检查通过。删除build后再次source安装、导入消息与桥接通过。未启动ROS节点或Isaac、无本次新飞行，不改服务器/P5/P4，其它任务performance结果和资料保留。

保存点：/home/starry/isaac-data/HISTORY/snapshots/EGO2P6_structure_20261019_204101。包含原布局源码/文档tar、服务器工具/旧入口备份归档、迁移清单和全部验证日志。旧安装只临时保留到新安装验证成功，随后删除；恢复旧布局需解压原小源码并按旧构建脚本重建。上文上轮清理中的路径为其当时状态，以本节和README当前结构为准。
