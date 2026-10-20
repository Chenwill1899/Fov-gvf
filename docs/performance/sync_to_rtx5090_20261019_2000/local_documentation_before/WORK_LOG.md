### 2026-10-19 16:10 — [EGO2P5] c30第6次R07预算卡停后，c31修复低速缓存饥饿与验证返回优先级

- 状态：c30实际6次5安全到达、1失败，后14槽未执行；c31主构建/最终定向重建与63/63回归通过，尚无c31飞行。
- 目的：按用户持续迭代要求，处理已离开旧停点但后续低速候选/恢复循环再次耗尽时间预算的问题。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/include/pc_gvf/paper_guidance.hpp、navigation_progress.hpp、src/active_navigation.cpp、depth_angular_controller_node.cpp、paper_replay.cpp、test/cpp/native_history_recovery_check.cpp、navigation_progress_check.cpp；本轮c30_validation完整6条原轨迹/样本/审计/录像/失败MP4/完整解码，candidates/c30_failure_analysis*、c31源码/ELF/SHA/构建/测试/冻结反事实；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：进展按参考目标投影>.20m或实际空间离开2m重置，2s无有效进展触发既有1Hz恢复，已存在pending低速候选不再否决搜索。停滞时从仍有.635m完整证明的已到达足迹中选距当前位置.35..3m的返回点，保留已发布延迟指令、固定事件原点和全部累计债务；显式验证返回优先于旧前进候选，恢复期间重置进展监视防止反复抢占返回。空事件创建时补充最多16个真实未证明且避开已测障碍的小壳层见证，空间分散选择，后续真实观测才能标记进展，不制造自由体积。新增W重放仅在verified_retreat状态需要保存时使用，保存新返回优先标记及完整V工作域；普通仍Q/U/V，读取兼容旧3—V。
- 验证：c30 R07/2于246.467s停在(81.009,32.584,7.172)，距终点49.245m，最终机体.595已知/重启.615未证明，时间11.4833s/距离仅.301m/体积1.218m³，末一分钟876内部预算拒绝；原风险/冻结/样本审计通过且扫掠/外部保护/限值违规0。c31 63/63通过（包括返回点不被当前内部点覆盖、风险债务保留、W重放一致、缓慢/侧向/后退进展检测）。同一旧失败过程中83.767/116.933s快照，请求验证返回后分别输出相反方向的完整认证返回，核心8.94/3.70ms，账本1.3167/4.5s保持；这是单次离线诊断，不冒充新闭环验收。构建中原节点已完成后，再用cmake目标重建/安装最终监视参数，确保最终源和实际安装一致。
- 影响与边界：10m/s请求、.20s延迟、.595+.02硬包络、3.5m/12s/5m³预算及80ms期限不改。没有为已耗尽预算的终末快照清账放行；真实已知返回和新测进展是许可条件。四次新增失败分别c27/c28/c29/c30保留，所有未执行槽位单列。
- 回滚/备份：原24完整基线、26—31逐版源码/实际ELF/SHA/所有尝试保留，未改P4/服务器，无提交推送。
- 后续事项：c31优先R07后R06/R09/R01，各五次串行可见实验；若再失败继续根据原证据优化。

### 2026-10-19 15:42 — [EGO2P5] c29第13次R06再失败，c30修复已知内部自锚点取消内退并完成冻结状态反事实

- 状态：c29实飞仿真13次中12安全到达、1卡停，后7槽未执行；c30构建3包和63/63回归通过，准备重新五轮比较。
- 目的：继续落实用户失败后归因优化，处理边界恢复接管条件本身的逻辑缺口。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/src/active_navigation.cpp、depth_angular_controller_node.cpp、test/cpp/native_history_recovery_check.cpp；本轮c29_validation完整13条轨迹/全量CSV/冻结执行/原审计/13原现场录像解码及失败MP4；candidates/c29_failure_analysis.json、failure_audit、c29_new_boundary_regression*、c30源码/实际ELF/SHA/构建/测试/c30_c29_failure_counterfactual.json；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：c29第四轮R06112.267s停于(100.525,2.553,19.08499)，距终点157.602m，时间账本仅.2667s。冻结审计机体+.03仍已知、锚点等于自身、recovering=0；旧逻辑因内部证明和自锚点提前结束恢复，普通旧方向却触碰新增预留线，循环失败。c30在通用锚点完成判定前强制接管临界垂向边界，保持内退直至退出边界带，不因已有内部证明取消；认证内退按1/.5/.25三档速度(.24/.12/.06m/s垂向)尝试，保留原硬包络和紧急路径。内退成功后仅清除调和场/旧提名与节点提名缓存/方向迟滞，不清除真实几何、风险预算或探索原点，实际发布历史仍由finish确认。
- 验证：旧c29静态核心执行新增“已知内部+自锚点仍需边界内退”用例确实失败；c30全部63/63通过。对完全相同c29末期冻结快照顺序各重复3次，c29核心中位274.07ms且PROBE_NO_USEFUL_PATH；c30中位2.34475ms、ACTIVE_CERTIFIED_INWARD、指令(0,0,-.24)，完整认证通过。该.24为已认证速度，未知许可仍限.2垂向。c30最终速度阶梯源码完成后另重建一次确认安装一致，未将构建中间文件当已验证飞行。c29所有13原录像及失败兼容MP4完整解码通过、原机位保留。
- 影响与边界：c29未完成20条，不冒称全通过；三次新增失败分别来自c27/c28/c29均完整保留。C30未知预算与硬工作域/半径/.20延迟/80ms期限保持，未修改地图或在线加入真值；冻结反事实支持该状态接管修复，不冒称闭环路线已通过。
- 回滚/备份：c24完整基线，c26—c30逐版源码/实际ELF/SHA，c29静态库用于负回归保留；不提交推送，不改P4/服务器。
- 后续事项：c30重新四路线各5次串行可见验证，仍在自然失败后继续分析优化。

### 2026-10-19 15:16 — [EGO2P5] c28高位R06卡停后，c29加入边界提名余量与三维内退

- 状态：c28实际1次R06失败，158.9s停于(121.928,17.690,19.181)，距目标140.012m，其余19槽未执行；c29重建与63/63回归通过，尚无新飞行。
- 目的：处理计算加速仍未解决的高位局部路径与返回方向不足，继续按新失败迭代。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/src/active_navigation.cpp、paper_guidance.cpp、test/cpp/native_history_recovery_check.cpp；本轮c28_validation完整原数据/现场MKV与兼容MP4/两格式解码/SHA，candidates/c28_failure_analysis.json、c29源码/实际ELF/SHA、构建/测试/重放；WORK_LOG与HISTORY/CHANGELOG。
- 修改内容：c29对普通向外的方向提名及完整预测路径加入0.10m垂向恢复余量（只删候选，不当自由证据），保留内退与紧急停止；机体靠地/顶边界0.15m范围且内部证明不足时进入恢复，优先尝试含0.24m/s垂向的完整认证内退、以及直接/斜向内退风险候选。未知执行仍被原0.2m/s垂向/.3总速、完整硬包络/返航/累计预算约束，不修改目标高度或硬工作域。
- 验证：c28完整风险/冻结执行/计算样本审计通过且扫掠/保护/限值违规0；最后一分钟内部56次风险返回与344次预算拒绝，说明高位局部通道和恢复问题保留。c29新增近顶边界回归验证认证下降、向外提名拒绝且向内提名允许；主63/63通过。首次c29编译因Eigen条件表达式两侧类型不同失败，显式eval修复后重建通过，初始失败日志未覆盖。已对旧c28首/末快照重放；末期预算仍不足、拒绝保持，未用放宽或清账伪造恢复。
- 影响与边界：10m/s请求上限、.20s延迟、3.5m/12s/5m³、.595+.02硬包络与80ms期限保持；前两新失败完整保留，不把停止候选的未执行槽算完成。源码行为候选只在P5，地图/服务器不改。
- 回滚/备份：baseline24、c26/c27/c28/c29逐版源码与实际控制ELF/SHA均保留，无提交推送。
- 后续事项：c29从R06开始四路线各5次，失败则保留并继续归因迭代。

### 2026-10-19 15:04 — [EGO2P5] c27复测R06近终点再次失败，归因后实施c28恢复评估加速

- 状态：c27两次实际运行后按既定规则停止后续槽位：R01安全到达，R06在136.4s卡停，距终点4.555m；另18槽未执行，不能计入完成。
- 目的：按用户“失败则继续总结优化”要求，解决常规认证改快后仍存在的未知空间返回评估超时。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/include/pc_gvf/paper_guidance.hpp、proof_query_memo.hpp、src/paper_guidance.cpp、active_navigation.cpp、test/cpp/proof_query_memo_check.cpp；本轮c27_validation完整原数据/两现场录像/解码/失败MP4导出及candidates/c27_failure_analysis.json；后续c28构建/重放/试验；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：c28将精确胶囊查询缓存扩大至当前不可变求解范围，按是否允许pending证明及4个线程分离、遇观测/新求解清空，阴性证明随本线程正证据版本变化失效，动态/共享时间检查仍逐段执行。未知费用改用包住完整曲线的单个连续胶囊，不能证明整条已知时将整条路程记作未知上界；只会更保守记账，不缩小风险预算。继续保留c27已修复的空事件与恢复速度提名。
- 验证：c27失败未隐瞒；70个内部EXPLORATORY_RECOVER结果因计算期限改发零速，最早核心711.8ms；末期加入摄入后仍超80ms，最后时间账本11.3667s但下一步与返回超12s。所有原审计扫掠/保护/限值违规0，轨迹/样本/冻结身份与风险审计通过；两录像完整解码。c28目前构建中，未宣称通过或提升。
- 影响与边界：保持.595+.02机体、3.5m/12s/5m³、.3m/s/.2垂向探索与80ms期限；未放行旧预算末期帧。缓存为求解范围内已有精确几何结果，不跨新观测，不将未知写成自由证据。
- 回滚/备份：c24/c26/c27完整活动源或修改源码/实际ELF/SHA均保留；c27全部失败及18未执行槽位保留，原P4/服务器不变。
- 后续事项：完成c28回归和新失败冻结快照计时，再重新执行同源四路线各五次；继续记录并分析新失败。

### 2026-10-19 14:56 — [EGO2P5] c26四路线首轮4/4安全到达，继续c27消除恢复速度反复细化

- 状态：c26实现/回归/4次实际飞行完成；c27构建和63项回归通过，正在准备完整比较。
- 目的：不仅依靠少量成功轮，继续处理旧失败快照仍存在的持续计算期限问题。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/include/pc_gvf/paper_guidance.hpp、src/pc_gvf/src/active_navigation.cpp、paper_guidance.cpp、test/cpp/native_history_recovery_check.cpp；performance/iterative_recovery_20261019_1425/candidates/c26、c27及构建/回归/离线重放/分析；c26_pilot/全部4轨迹/全量样本/原审计/现场录像/完整解码；work/run_trials.py、summarize_trials.py、verify_videos.py、x11_layout.py、record_windows.py与独立计时诊断；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：c26允许空观测目标事件在真实位移>.2m且仅创建事件后的真实深度完整证明.635m内部机体后关闭，旧种子/旧观测/时间变化或松杆不清账；加精确静止胶囊快捷和路径内同查询复用。c26试用把前缀球改为胶囊内接球，但未改善旧困难帧耗时，c27已恢复原独立证明的大前缀球。c27将恢复候选速度提名限制到max(.3m/s,1.25×实际已发布/测量速度)，仍需完整认证；大速度失败后优先邻近已执行速度，找到第一个完全认证速度即返回，取消额外5轮二分求极限，防止有安全指令却耗到超时清零。风险、硬包络、工作域、.20s延迟与80ms期限均不放宽。
- 验证：旧核心运行新增空目标回归确实失败，新版两次主构建3包和全部63/63回归通过。c26四条各1次全部安全到达、配置/冻结执行/样本/风险审计全通过、扫掠/保护/限值违规0，4录像完整解码，均值R06 1.610/R07 3.066/R09 2.788/R01 3.985ms，仍有844ms长尾。独立计时确认旧R06/3冻结帧c24/c26约109/107ms，而c27约3.6ms可认证非零指令；完整离线明细保留。并行CTest期间的最初重放进程耗时不作为性能结论，后续在无仿真/构建负载时顺序重复3次测核心。
- 影响与边界：c26首轮4/4不能等同四路线各5次稳定通过；c27未新增飞行，预算型旧末期帧仍会正确拒绝耗尽预算，不能靠清账放行。新原机位录像均保留，未使用在线真值。候选26/27和原24实际ELF、对应源码与SHA单独保留。
- 回滚/备份：baseline_candidate24/为完整原始活动树与实际安装；candidates/c26、c27为逐版源码/ELF/SHA。原P4和服务器不改，无提交推送。
- 后续事项：c27同源同参四路线各五次串行可见实验；若自然失败，保留全过程并暂停该候选后续槽位，分析失败再迭代；不将未执行槽位冒称完成。

### 2026-10-19 14:40 — [EGO2P5] 按六次失败归因开始算法与参数迭代优化

- 状态：进行中；用户明确授权优化、实际仿真、失败后继续归因优化。
- 目的：提升本机四路线候选24基线14/20到达率，针对空观测目标事件、认证长尾、高位恢复与局部往返。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：performance/iterative_recovery_20261019_1425/plan.json、baseline_candidate24/完整活动源码/配置/脚本与四个实际构建产物/SHA/Git状态；后续src/pc_gvf及本机试验记录；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：先完整备份活动候选24及未提交状态，不重置既有修改。计划修复真实新观测进展关闭空目标事件，消除重复几何验证并保留完整扫掠与紧急停止检查；失败后再迭代恢复策略。完整机体/工作域、无全图真值输入、累计风险预算与80ms停车期限保持。
- 验证：开始前读取HISTORY规则和项目AGENTS；无活动导航，磁盘约50GB可用。尚未构建或飞行，不预报提升。
- 影响与边界：默认只改P5，P4/服务器/地图与历史证据保留；实际实验串行并保留Isaac/RViz原角度现场录像。
- 回滚/备份：本轮baseline_candidate24/含完整来源和实际安装二进制、静态库及SHA；无提交推送。
- 后续事项：实现、回归、离线重放、可见仿真并记录各候选所有失败后，按实测结果选择保留版。

### 2026-10-19 14:17 — [EGO2P5] 六次本机未到达的逐次原因对比与冻结快照诊断完成

- 状态：完成；只分析既有20次实验，没有新增飞行或修改部署算法。
- 目的：按用户要求，对六次失败逐一比较同路线成功运行，判断具体未到达原因。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：performance/local_failed_routes_5x_20261019_1310/failure_analysis_20261019/FAILURE_ANALYSIS.md、six_failures.csv、trace_analysis.json、solver_timing_analysis.json、terminal_metrics.json、replay_results.json、diagnostic_results.json、diagnostic_validation.json、input_sha256.json、analysis_verification.json、analyze_traces.py、make_diagnostic.py、plot_comparison.py、diagnostic/、两张比较图及六张原视频抽帧；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：新增离线分析和仅增加日志的独立重放器，不替换安装。R06/3、R09/5末一分钟所有内部可认证/保持候选分别334/357次被80ms期限清零；核心中位131.31/88.66ms。R07/2、R06/4、R09/4为0.595m已知/0.615m重启壳未证明后的预算死锁：时间账本12.233s；或加候选与返航后12.032/12.317s超12s。R06/5从58.98s后一直在末点25cm内，却再走20.46m，最长连续低速26.47s，反复恢复/制动/期限导致600s超时。六失败快照均active事件开放但critical_targets空/observed_targets0，源码关闭条件要求非零标记，已定位事件无法关闭逻辑缺口；成功快照也可存在该状态，非必败充分条件。
- 验证：全部20条60Hz轨迹、全量计算CSV和协议核对；6末期原快照各普通/审计重放共12次exit0；独立诊断20快照含4成功对照，状态/accepted/指令误差<1e-10全部匹配。211项冻结活动源/入口/安装ELF/场景/占据SHA零变化，3新增脚本AST通过。两图与六张原录屏约80%抽帧已查看，保持遮挡/原机位。首个OpenCV导入因用户NumPy2与系统扩展ABI不兼容失败，PYTHONNOUSERSITE=1重试6张成功，未改依赖；绘图缓存使用/tmp。
- 影响与边界：归因区分求解器内部状态与最终发布状态、卡停直接条件与前因，不把诊断保存造成的失败后期限误归为起因。旧6失败均保留；扫掠/保护/限值违规均0，不能称碰撞失败。未修改或验证修复方案，未做传感器逐帧锁定的闭环反事实；不宣称录屏/GPU单独造成失败。
- 回滚/备份：所有新增物仅在本批分析子目录，原始数据不改；独立诊断源码与构建命令/SHA保留，不覆盖主安装；旧日志只追加，无提交推送。
- 交付：报告/对比图/诊断证据共94文件已复制至/home/starry/Documents/Codex/2026-10-18/ego2p5/outputs/四条路线各五次实验_20261019/失败原因分析_20261019，复制后逐文件SHA一致；本批分析目录保存delivery_receipt.json。
- 后续事项：建议优先修复空观测目标事件关闭、有界认证计算、高位三维返回和目标进展循环识别；本任务仅归因，无自动续跑。

### 2026-10-19 13:53 — [EGO2P5] 本机四条历史失败路线各5次复测、失败现场双窗口录像与完整核验完成

- 状态：完成；20/20次本机实际飞行，14次到达、14次安全到达、6次未到达；R01 5/5到达；R06 2/5到达；R07 4/5到达；R09 3/5到达。无自动续跑。
- 目的：落实用户本机更新算法R01/R06/R07/R09各五组实验、记录结果并保留失败无人机与RViz原角度录屏要求。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：performance/local_failed_routes_5x_20261019_1310/下plan.json、state.json、RESULTS.md、results.csv、completion_summary.json、environment.json、frozen_batch_identity.json、failed_recordings.txt、work脚本/逐轮握手/布局/截图/进程资源/日志、videos各轮原始MKV/兼容MP4/采集元数据、video_checks各轮完整解码/原始及MP4 SHA、posters；performance/p5_speed_20261018/local_ubuntu/local5x_20261019_1310_*每次协议/冻结实际ELF/参数/传感器/全量轨迹与计算样本/原独立审计/风险账本审计；项目README/EGO2P5_VERSION/CURRENT_CHECKPOINT/WORK_LOG及HISTORY README/FOV_GVF_FAMILY/CHANGELOG。
- 修改内容：候选24本机Humble实际控制ELF SHA668ec16bcab482ef1b2dc9a2e4bb2a18677571d960ad96c4f01f6fa3e29c891d；20次全新进程、四路线交替五轮，10m/s上限/垂向1/depth20/H.5/cache.3/.75/history16、0.20s模型延迟、a16/tau.10、完整机体工作域/原累计预算、默认不绑核均保持。每次先等待可选录屏握手再开始仿真时间，Isaac/RViz并排现场X11录制2560x800/15fps，默认第三人称跟随视角遇遮挡不调整。全部成功/失败原始H264 high444 MKV保留；交付MP4只做I420兼容转码，不改机位/不裁剪时序/不重建轨迹。控制算法、地图、相机未改，未使用服务器Jazzy安装。
- 验证：20次原始审计与独立冻结执行身份匹配，运行期间活动源/实际安装未变；除路线目标和日志路径外有效控制参数一致；全部全量计算样本数与guidance计数一致。扫掠重叠总0、外部保护总0、限值违规{'speed': 0, 'vertical': 0, 'acceleration': 0, 'applied_command': 0}，风险日志审计全通过=True。20个视频逐帧完整解码、双画面变化、录屏就绪先于仿真、原始/MP4 SHA核对通过。原协议600s仿真超时/既有60s零速卡停正常终止，未按循环另行提前截断。没有把失败轮从均值/计数排除，没有重复选成功轮替代失败。
- 失败与修正：准备阶段首个编码检查缺h264parse组件失败，改用x264原生AVC格式直连Matroska后15帧编码/解码通过；无需安装依赖。临时防锁屏首个gnome-session-inhibit调用不支持--命令分隔符而退出1，按本机语法重试成功，本轮结束自动释放，不改系统设置。早期一次聊天把R07/2“结束”误说成“到达”，读原审计后立即更正；原始账本始终保留ABORTED_EXPLORATION_STALL，最终数据不受该口头误报影响。所有原失败、日志、被遮挡画面保留。
- 影响与边界：本轮为本机显示与录屏条件下仿真实测，不等同5090无画面或实飞；存在卡停与计算长尾，不能宣称任意路线无卡停或严格每帧1ms。全部逐次均值/P95/P99/最大值由完整流式样本计算，常驻8192窗口不冒充全程分位数。
- 回滚/备份：可选录屏握手前原运行器保存在本轮work/run_fov_gvf_navigation.before_recording_gate.py；未设置ISAAC_RECORDING_HANDSHAKE_DIR时原流程不等待。原P4、来源快照、地图和历史失败未改，未提交推送。
- 后续事项：无；所有授权实验和失败现场录像交付完成，无后台导航或排队。

### 2026-10-19 13:11 — [EGO2P5] 开始本机四条历史失败路线各5次实验与Isaac/RViz现场录屏

- 状态：实验准备完成，尚未启动飞行，后续逐次记录；用户明确授权20次本机实验及失败录像。
- 目的：使用更新后的候选24，在本机对R01/R06/R07/R09各5次复测，保留未成功到达全过程屏幕录像。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：performance/local_failed_routes_5x_20261019_1310/（work/x11_layout.py、record_windows.py、run_batch.py、原运行器备份、编码检查视频及后续plan/state/记录/录像）；scripts/isaac/run_fov_gvf_navigation.py；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：新增串行实验编排，四路线交替5轮，沿用本机主run_goal/主运行Shell，当前本机候选24控制ELF SHA668ec16bcab482ef1b2dc9a2e4bb2a18677571d960ad96c4f01f6fa3e29c891d、10m/s/垂向1、history16、0.20s延迟、默认不绑核。原协议600秒仿真上限、现有60秒零速卡停终止，不新增循环判定提前结束；进程异常有1800秒wall/轨迹180秒不更新独立中断记账。所有尝试先录，失败视频完整保留，成功视频也保留。只将本轮Isaac/RViz窗口并排固定，保持默认跟随视角不因遮挡调整；真实X11双窗口画面15fps/2560x800/H264，非轨迹重建。仿真开始前增加可选录屏握手，录屏准备好才启动模拟时间/意图；不设环境变量时沿用原流程，控制算法/地图/相机未改。
- 验证：真实设备检查本机RTX5070、3840x2160桌面、无其它导航、磁盘约56GB空闲；所有新增Python与运行器AST通过。首次编码检查因缺h264parse组件失败，保留事实；改用x264原生AVC格式直连Matroska后15帧编码/完整解码通过，无安装依赖或驱动更改。尚未运行本轮飞行，不预报成功。
- 影响与边界：现场显示/录屏带来额外开销，本轮性能不能冒充5090无画面对照。所有20次使用冻结同源/同安装/同参数，运行间检查身份，不因失败调算法或角度。原P4及既有失败证据保留。
- 回滚/备份：运行器原文位于本轮work/run_fov_gvf_navigation.before_recording_gate.py；新增批处理脚本独立；可选握手默认关闭，无Git提交推送。
- 后续事项：执行20次并核对全量轨迹/计算样本/原审计/现场录像完整性，完成后追加实际结论。

### 2026-10-19 12:57 — [EGO2P5] 为当前候选24本机入口加入R07路线选择并交付可见运行命令

- 状态：完成；用户自行启动，本轮未执行导航。
- 目的：按用户“算法更新了，用新的算法给我RO7路线的运行指令”要求，读取最新候选24本机安装与真实R07起终点，保留Isaac/RViz。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：deployment/local_ubuntu/run_route300.py、deployment/local_ubuntu/README.txt、deployment/local_ubuntu/backups/route_selector_20261019_125713/run_route300.before.py、runtime/local_ubuntu/route_selector_validation_20261019/summary.json、WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：本机包装器原固定旧300m起终点，新增EGO2P5_GOAL_ROUTE路线选择，未设仍route300；r07/R07从已保存的source_5090_comparison/routes.json选择真实起点[321.6,-37.6,10]、终点[33.6,45.6,10]，距离299.777m。身份记录增加requested_route_id及route_source，保留路线文件SHA。非法路线在运行前拒绝；不覆盖旧route300.json。当前候选24本机控制器、手柄策略、10m/s、0.20s延迟、工作域、显示开关和原共用锁均沿用。
- 验证：Python AST与Shell语法通过；独立加载包装器构造协议，默认旧路线、小写r07、大写R07及非法值拒绝共4例通过，实际协议速度10/延迟.20/工作域保持。检查未调用driver.run、ROS或Isaac，不是新飞行/画面/性能验收；仅Python入口变更无需重新编译控制器。
- 影响与边界：原P4/算法核心/相机/地图/旧证据无改动；最新来源R07仍存在卡停失败，提供命令不代表该路线已稳定通过，本机新飞行未验收。
- 回滚/备份：修改前包装器保存在deployment/local_ubuntu/backups/route_selector_20261019_125713/run_route300.before.py；不设EGO2P5_GOAL_ROUTE即可继续原路线。
- 后续事项：用户自行运行；无后台续跑或自动排队。

### 2026-10-19 12:54 — [EGO2P5] 绘制新版30+20次中全部未到达路线组的实测高度

- 状态：完成；只读重绘既有实验，没有新飞行或算法修改。
- 目的：按用户附图样式绘制新版50次实际飞行中出现未抵达终点的路线组，保留同组成功曲线供对照。
- 项目路径：/home/starry/isaac-data/EGO2P5；只读来源EGO2P4/performance/multiroute_20261018_2208与compare_before_after_20261019_1007。
- 涉及文件：performance/failure_heights_50runs_20261019/下逐项：plot_failure_heights.py; validate_delivery.py; record_history.py; manifest.json; all_50_flights.csv; six_failed_flights.csv; RESULTS.md; generate.log; generate_initial.log; delivery_check.json; height_figures_and_data.zip; figures/r01_measured_height.png; figures/r01_measured_height.svg; figures/r06_measured_height.png; figures/r06_measured_height.svg; figures/r07_measured_height.png; figures/r07_measured_height.svg; figures/r09_measured_height.png; figures/r09_measured_height.svg; figures/failed_route_groups_overview.png; figures/failed_route_groups_overview.svg; figures/six_failed_flights.png; figures/six_failed_flights.svg; heights_csv/r01_A1_height.csv; heights_csv/r01_A2_height.csv; heights_csv/r01_A3_height.csv; heights_csv/r06_A1_height.csv; heights_csv/r06_A2_height.csv; heights_csv/r06_A3_height.csv; heights_csv/r07_A1_height.csv; heights_csv/r07_A2_height.csv; heights_csv/r07_A3_height.csv; heights_csv/r09_A1_height.csv; heights_csv/r09_A2_height.csv; heights_csv/r09_A3_height.csv; heights_csv/r01_B1_height.csv; heights_csv/r01_B2_height.csv; heights_csv/r06_B1_height.csv; heights_csv/r06_B2_height.csv; heights_csv/r07_B1_height.csv; heights_csv/r07_B2_height.csv; heights_csv/r09_B1_height.csv; heights_csv/r09_B2_height.csv；P5 WORK_LOG.md、HISTORY/CHANGELOG.md。
- 修改内容：筛选首批validation5及仅启动重试的30飞行，加对照批version=after的20飞行，排除旧版20；同一candidate24控制ELF d6092715...、10m/s请求上限和相同10组Z10起终点。44到达、6未到达，涉及R01/R06/R07/R09（1/1/3/1次）；每组画A首批3次+B对照新版2次，共20条80561完整样本。生成4单组PNG/SVG、4组总览PNG/SVG、6次失败独立面板PNG/SVG（共12图）、20高度CSV、全50及6失败清单、完整来源SHA/路径与ZIP。坐标为原始仿真t减预热2s、实测世界Z；无平滑/插值/抽稀。×仅是未到达中断终点，不是碰撞点或首次卡停时刻；虚点线10m目标高度与19.8m工作域上界不是障碍顶面。3次无轨迹启动R08(A)/R05(B)/R10(B)单列，补跑按原槽计50，不能画不存在的高度。
- 验证：50原CSV SHA逐项匹配原报告；协议/实际参数/ELF、相同起终、速度10、预热2、域上界19.8核对通过；开始/结束SHA一致。20导出CSV80561个t/XYZ与原CSV逐值完全一致；50唯一标签、6失败、每组A3/B2、3启动无轨迹断言通过。6PNG解码、6SVG解析、ZIP完整性/图与数据字节、Python语法和报告链接通过；4独立图及6失败面板已目视检查，图例避开R01/R09曲线、修正预热灰区坐标轴及六面板页脚间距。原始数据未改；没有运行ROS/Isaac/手柄/新导航、未重新跑核心回归。
- 影响与边界：图表属于5090已记录实验，不能当本机新飞行、性能改进或卡停原因证明。6次原状态均INTERRUPTED_CYCLIC_RECOVERY，是循环恢复卡停后中断，不能伪称自然TIMEOUT/碰撞。算法/配置/本机安装/地图/来源历史保持，无服务器连接、无续跑、未提交推送。
- 回滚/备份：本轮仅新增performance/failure_heights_50runs_20261019与两处追加日志；来源50轨迹/协议/参数和两批总结保持原路径与SHA，重绘脚本和包可复现。无现有源码覆盖或删除。
- 后续事项：无；用户查看4组高清图/6失败单独面板或下载PNG/SVG/数据包。HISTORY规则R-001保持。

### 2026-10-19 12:44 — [EGO2P5] 同步5090候选24到本机Humble并完成构建回归

- 状态：完成用户本次本机算法同步；无新导航飞行，本机5070性能/平均稳定1ms与无卡停未验收。
- 目的：将此次5090优化后的candidate24_published_command合并进本机EGO2P5，保留已有手柄、可见Isaac及RViz入口。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件（逐项）：ACTIVE_NAVIGATION.md; AGENTS.md; EGO2P5_VERSION.md; MANUAL_CONTROL_PROFILE.md; README.md; deployment/local_ubuntu/README.txt; deployment/local_ubuntu/run_goal.sh; deployment/local_ubuntu/run_manual.sh; deployment/local_ubuntu/run_route300.py; profiles/manual_10mps_depth20.env; scripts/isaac/manual_control_math.py; scripts/isaac/navigation_benchmark.py; scripts/isaac/run_fov_gvf_navigation.py; src/pc_gvf/CMakeLists.txt; src/pc_gvf/include/pc_gvf/depth_angular_core.hpp; src/pc_gvf/include/pc_gvf/navigation_progress.hpp; src/pc_gvf/include/pc_gvf/paper_guidance.hpp; src/pc_gvf/include/pc_gvf/proof_query_memo.hpp; src/pc_gvf/include/pc_gvf/timing_series.hpp; src/pc_gvf/launch/isaac_cloud_navigation.launch.py; src/pc_gvf/src/active_navigation.cpp; src/pc_gvf/src/depth_angular_controller_node.cpp; src/pc_gvf/src/depth_angular_core.cpp; src/pc_gvf/src/paper_guidance.cpp; src/pc_gvf/src/paper_replay.cpp; src/pc_gvf/test/cpp/active_navigation_check.cpp; src/pc_gvf/test/cpp/camera_evidence_cache_check.cpp; src/pc_gvf/test/cpp/certified_recovery_check.cpp; src/pc_gvf/test/cpp/exhausted_budget_check.cpp; src/pc_gvf/test/cpp/free_space_ceiling_check.cpp; src/pc_gvf/test/cpp/navigation_progress_check.cpp; src/pc_gvf/test/cpp/proof_query_memo_check.cpp; src/pc_gvf/test/cpp/published_command_search_check.cpp; src/pc_gvf/test/cpp/recovery_comfort_check.cpp; src/pc_gvf/test/cpp/timing_series_check.cpp; src/pc_gvf/test/cpp/working_bounds_check.cpp; src/pc_gvf/test/test_long_runtime.py; src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py; src/pc_gvf_platforms/pc_gvf_platforms/observed_map_visualizer.py; src/pc_gvf_platforms/test/test_display_storage.py; tools/sim4_benchmark.py; tools/test_sim4_benchmark.py；WORK_LOG.md；performance/sync_candidate24_20261019_1240中的原备份/487文件清单、合并脚本/计划/补丁、191来源SHA、42生产/文档变更与日志清单、测试/参数/GUI/Shell截获记录、源码取证报告副本、RESULTS/completion_summary、审计/文档/记录脚本；HISTORY/README.md、FOV_GVF_FAMILY.md、CHANGELOG.md。生成缓存/常规构建文件不逐一列举。
- 修改内容：33项服务器增量三方合并；真实发布命令只提交一次，恢复搜索1Hz、精确记忆4096/几何复用、计时常驻8192与CSV流式、显示100万+25万有界、完整机体工作域/ceiling、V重放。恢复可选内侧舒适带4cm→1cm，硬.595/起步.615及可靠锚点.635保持。profile10/垂1、16历史/1s、depth20/H.5/cache.3/.75、a16/tau.10/world保持，新.20工程延迟和X[-80,399.8]/Y[-60,59.8]/Z[0,19.8]生效。服务器绑核0..15改为本机显式EGO2P5_CONTROLLER_CPU_AFFINITY、默认不绑核；goal包装增加域审计声明及显式--pipeline-delay .20，防止测试器默认0覆盖；manual日志版本标识与可选本机亲和适配。
- 验证：正式代码包SHA36f89733...、191活动来源SHA全匹配；142项原生/平台源除P5身份替换外与服务器逐字一致。主构建3包成功，63/63核心（7 pytest入口）、174/174平台工具通过。source/install保存10/默认6/目标手柄/固定目标8/8，delay/bounds/P5路径/ELF核对；RViz条件4例、显示选项6例；真实goal Shell末尾exec截获验证CLI.20、10/1与域/策略/默认不绑核，通过且未执行路线。Shell/Python语法通过。24项source/install一致、178项P4活动源/地图无变化；本机共用导航锁空闲。没有Node.execute、新Isaac/RViz运行、真人手柄或飞行。
- 失败与修正：初三方合并仅profile安装路径冲突，尚未装源即退出；解决为本机路径/可选CPU后应用。初参数验证在goal发现delay=0非.20，first_validation_attempt完整保留；修复真实goal Shell并重跑受影响8参数/GUI/语法/实际Shell截获通过，C++与平台已过测试未重复。构建有禁字节码/串行LTO警告、退出0。额外只读SSH核对因自动审批服务容量不足未执行（非不安全裁定）；采用先前完整取证且此次重新SHA校验快照，未绕过审核/改服务器。
- 影响与边界：本机P5源/独立Humble安装已更新，原P4和地图/历史失败/来源备份不变；不执行Jazzy二进制，ROS42/共用锁/单发布者检查保持。最新5090来源对照40轨迹到达9/20→17/20、逐次均值再平均7.424→3.238ms、中位3.666→.984、max914.469→394.229；期限拒绝率1.195%→1.210%未改善，3次新版卡停，R07均9.068→22.713退化。上述均非本机新成绩；辅助升降尚非完整自适应螺旋、恢复非完整历史走廊规划、80ms不可抢占局限保持。
- 回滚/备份：performance/sync_candidate24_20261019_1240/before_sync.tar.gz，8,033,432字节、SHA34fee8e8...；源码/脚本/profile/tools/本机入口/文档与实际旧本机install487普通项回读全匹配。新文档前版另存documentation_before；原始来源包/P4快照和旧10月18日导入保留；all_active_changes.patch/final_changed_files.json用于逐项恢复。当前本机控制ELF SHA668ec16b...，构建自同源Humble；与服务器d6092715...环境不同，不要求ELF相同。
- 后续事项：本机新飞行/真人手柄/5070性能尚未执行；当前同步请求已交付，无自动续跑、未提交推送。HISTORY规则版本R-001不变。

### 2026-10-18 19:59 — [EGO2P5] 300米Isaac与RViz入口构建和source/install显示条件核验完成

- 状态：完成；未启动导航，交付用户自行运行。
- 目的：验证本次RViz入口改动已安装且不会重复启动控制器。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：runtime/local_ubuntu/goal_rviz_validation_20261018/（build.log、validate_launch.py、summary.json）；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：本条补记前条显示功能验证结果，无新增算法变化。
- 验证：原主构建脚本3包成功，仅Python禁止字节码提示；源码/安装launch各rviz=true/false共4例构造通过，true为1控制器+1桥+2显示节点+1RViz，false仅前2节点；手柄策略use_fixed_goal=false和max_speed10/max_depth20均保持。显示开关默认/显式及非法值拒绝6例通过；Node.execute调用0、飞行0。首次检查未设ROS_LOG_DIR而在只读~/.ros/log报错，改工作目录日志；第二次测试访问不存在的Node私有属性报错，改公开node_executable后全部通过；仅测试脚本修正，未影响生产启动。
- 影响与边界：已完成构建/静态launch构造，未现场验证本次Isaac+RViz画面/点云或新的到达，不把检查当新飞行验收。
- 回滚/备份：前条goal_rviz备份保留；无显示时默认行为保持。
- 后续事项：用户运行EGO2P5_GOAL_HEADLESS=0 EGO2P5_GOAL_RVIZ=true bash deployment/local_ubuntu/run_goal.sh 唯一标签；无自动续跑。

### 2026-10-18 19:58 — [EGO2P5] 300米可见路线同时启动RViz和观测/轨迹节点

- 状态：实现完成；构建与启动构造核验待追加，未启动导航。
- 目的：满足用户要求300米自动路线同时打开RViz。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/launch/navigation_benchmark.launch.py、tools/sim4_benchmark.py、deployment/local_ubuntu/README.txt、deployment/local_ubuntu/backups/goal_rviz_20261018_195848、WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：benchmark原来只有控制器与命令桥，新增rviz条件下的navigation_visualizer、observed_map_visualizer及rviz2，参数/配置复用人工入口原值。新增EGO2P5_GOAL_RVIZ=true/false；有画面默认true、无画面默认false。保持单控制器、全部算法/路线/速度/累计预算不变。
- 验证：两修改Python文件AST通过；无ROS/Isaac启动，后续补记构建和实际launch构造结果。
- 影响与边界：显示真实在线观测而不提供全图真值；可视化有开销，非本机新性能或飞行验收。
- 回滚/备份：修改前文件保存在deployment/local_ubuntu/backups/goal_rviz_20261018_195848。
- 后续事项：主构建安装launch，核验source/install节点与rviz条件，交付用户自行启动命令。

### 2026-10-18 19:56 — [EGO2P5] 为300米自动路线增加可见Isaac窗口开关

- 状态：完成；未启动导航，交由用户自行运行。
- 目的：满足用户询问300米起终点有画面运行指令。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：tools/sim4_benchmark.py、deployment/local_ubuntu/README.txt、deployment/local_ubuntu/backups/goal_visible_20261018/sim4_benchmark.before.py、WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：驱动此前固定ISAAC_HEADLESS=1，新增EGO2P5_GOAL_HEADLESS=0/1读取和校验，未设仍1；设0显示Isaac窗口。其余协议、300m起终点、速度10、手柄策略、冻结/审计、共用锁不变。benchmark原本不含RViz节点，此开关只显示Isaac。
- 验证：Python AST语法通过；独立执行新增选项表达式与校验，默认1/显式0/显式1三例及非法值拒绝通过；未执行ROS/Isaac、未新增飞行，不宣称画面或到达已验收。
- 影响与边界：只变更显示选择；可见窗口有额外渲染开销，不沿用5090无画面性能作为本次成绩。
- 回滚/备份：修改前完整驱动保存于deployment/local_ubuntu/backups/goal_visible_20261018/sim4_benchmark.before.py；未设置新变量保留原无画面行为。
- 后续事项：用户自行启动，无自动续跑。

### 2026-10-18 19:46 — [EGO2P5] 补充停止结果：本轮导航锁已释放

- 状态：完成；仅本轮进程全部停止，共用锁实测可独占获取。
- 目的：保障用户自行启动不受刚才联调占用。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：本轮session_summary.json、WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：前条首次检查shared_lock_free=false；ROS组已退出，但Isaac PID33960在着色器编译等待中未响应SIGINT/SIGTERM，严格校验其命令身份后仅对该进程SIGKILL，父Shell随后正常清理退出。
- 验证：fcntl LOCK_EX|LOCK_NB成功获取原共用锁。保留此前失败及强制停止事实。
- 影响与边界：无算法/地图修改、无飞行完成结论，不影响其它任务。
- 回滚/备份：本轮原日志保留；下次正常启动将继续创建独立日志。
- 后续事项：用户自行启动，无后台续跑。

### 2026-10-18 19:46 — [EGO2P5] 用户改为自行启动，结束本轮手柄启动联调

- 状态：已结束本轮启动联调；共用导航锁释放=False。用户将自行启动。
- 目的：落实用户“直接给运行控制代码，我来启动”的最新要求。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：runtime/local_ubuntu/manual_runs/manual_20261018_194343_33786/（console.log、actual_parameters.yaml、runtime_manifest.json、session_summary.json）；WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：按精确PID与命令身份停止本轮Isaac及其ROS启动组；不改算法、数值、地图。新增run_manual.sh保留供自行运行，启动代码亦可直接source既有profile后执行原主Shell。
- 验证：实际控制器为本机depth_angular_controller，SHA88dc5e963726859c647b551e150d8b148f649fbb7e2817a75b9384121ae9d5f0，use_fixed_goal=false、speed/max_speed10、垂向1、depth20/H.5/cache.3/.75/history16、三主动策略及累计预算读回一致，/position_cmd单发布者。Isaac/GPU/四相机初始化完成至首次RtPso着色器编译等待，用户要求自行启动时主动结束；未到视口捕获或回中解锁，不写成完整手柄/飞行通过。初始化曾有radar_points临时QoS告警与系统rclpy导入回退，保留日志，未改配置/依赖。
- 影响与边界：无自动目标或代替用户推杆；本次仅实际启动及参数联调，不新增300m到达、本机1ms或真人手柄验收。
- 回滚/备份：既有活动源176项SHA零差异，新增入口及本次全部日志保留。
- 后续事项：用户自行启动并现场操作，未安排自动续跑。

### 2026-10-18 19:43 — [EGO2P5] 启动本机物理手柄控制，核对最终三轮验收版本

- 状态：启动准备完成；真实GUI/ROS联调结果待后续追加，不预先声明通过。
- 目的：按用户要求阅读项目、确认最新连续三次成功验收同版，并供用户用物理手柄运行。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：新增deployment/local_ubuntu/run_manual.sh；runtime/local_ubuntu/manual_runs/及latest_manual_run.txt；项目WORK_LOG与HISTORY/CHANGELOG；聊天outputs手柄启动脚本。
- 修改内容：新入口source既有manual_10mps_depth20.env与本机environment.sh，经原主运行Shell启动；固定域42、提名位移.75、本机不绑核，10m/s/垂向1与三主动策略保持，物理手柄无超时，GUI/RViz开启；每轮独立控制台/性能/manifest/视口/重放目录。控制算法、相机、地图、既有主脚本未改。
- 验证：活动源SHA清单176/176一致；Shell语法通过；本机真实设备北通A2P3A BFM /dev/input/js0、8轴16按钮映射0/1/2/3，GPU RTX5070/580.178.04，DISPLAY=:0；检查时无现存导航。沙箱设备/GPU不可见已改用只读非沙箱核实，非驱动故障。沿用19:38构建44/168/8回归证据，未重复构建、未把来源5090三轮.547/.542/.538ms当本机成绩。
- 影响与边界：本次供用户操作仿真；松杆/断连与回中门沿用。尚无本次用户推杆验收或本机1ms成绩，不自动执行目标导航。
- 回滚/备份：新增入口可独立移除；现有源码176项SHA无差异，原验收及P4保留。
- 后续事项：启动GUI并核实单发布者、实际控制器/参数及手柄在线状态，按实测追加结果。

### 2026-10-18 19:38 — [EGO2P5] 保存5090当前版并完整复制到本机，环境适配与3包/44/168/8核验通过

- 状态：完成；新目录在本机Ubuntu，不在服务器。本次复制不新增导航或本机性能验收。
- 目的：落实用户保存当前版本、全部内容复制到本机EGO2P5并说明代码算法/进展/状态的要求。
- 项目路径：来源ubuntu5090@192.168.31.112:/home/ubuntu5090/dhm/EGO2P4；目标/home/starry/isaac-data/EGO2P5；本机旧/home/starry/isaac-data/EGO2P4保留。
- 涉及文件：EGO2P5全部来源内容；performance/saved_current_20261018_1905/{savepoint.json,active_source_and_executables.tar.gz,source_snapshot.json}；performance/import_from_rtx5090_20261018/（原样/来源/合并/最终SHA核验、15文件冲突归档、13文件适配前备份与补丁、构建/CTest/pytest/8参数日志和completion_summary）；deployment/local_ubuntu/；README、EGO2P5_VERSION、AGENTS、ACTIVE_NAVIGATION、MANUAL_CONTROL_PROFILE、CURRENT_CHECKPOINT、WORK_LOG；HISTORY的README/PROJECT_INDEX/FOV_GVF_FAMILY/ENVIRONMENT_AND_ASSETS/CHANGELOG。
- 修改内容：先在指定服务器P4内保存当前源与执行产物、完整清单，补齐前轮视频证据，确认无导航；rsync完整复制含隐藏文件、运行/构建/缓存、失败冻结、备份和视频，不排除任何服务器项目文件。17,975普通文件/1,770目录/71链接（另含目标根目录）与约11.28GB已传回，所有源SHA/类型/权限/纳秒mtime/链接目标通过；自清单文件单独SHA核验。补齐本机旧P4的Git和历史/诊断逻辑路径33,619条，物理新增33,215条/31,741文件，404条经保留remote_trials链接指向同一数据；不同内容15文件只归档，不覆盖服务器活动源。仅13文件做P5路径/身份及Humble适配，新增本机入口、独立runtime/local_ubuntu；原服务器runtime和deployment/rtx5090保留来源身份。ROS域42、共用锁与单发布者检查保持，本机不绑5090 CPU0..15。旧P4文档保存EGO2P4_*，新文档明确算法构成、参数、4次来源成绩与未解问题。
- 验证：原样复制差异0；服务器复制后19,816条重新全SHA/元数据核验变化0，清单SHA e0ff7ab3a3887a4e69d0e0f09ac23238f7b1aa4aa767edea187c22784595f350；服务器与本机来源都保留。合并后所有来源/本机补充/15冲突内容通过，本机P4全34,505条内容未变、共享文件inode0；其190原活动源SHA亦零变化。最终再核验17,974普通源文件（自清单另核验）与71链接，只有13适配文件/当前说明属预期改动，核心认证/主动算法/CMake与地图SHA不变，Node仅shared_robot_id变更。本机主构建3包成功（仅禁用bytecompile提示）、CTest44/44含6组pytest、独立平台/工具168/168、源/安装saved10/default6/手柄目标/固定目标8/8参数构造且Node.execute=0；Shell语法通过。测试域203，未启动新飞行。Git HEAD与P4相同50f7c5500af86e31d0d9af03de8ec9a29ffb10e1，未提交推送。当前源/执行/安装与本机部署来源区分，不宣称二进制跨发行版相同。
- 最新飞行/视频：继承的5090可视化轮同版40.167s到达、avoid均.976/P95 3.554/max172.137ms，2次COMPUTE_DEADLINE后恢复；扫掠/保护/限值违规0。634帧逐一与原CSV位置一致、完整解码通过，NAV偏航重建最大误差.046度，所有5条显示视线可见；场景前后SHA一致。首版字幕下沿裁切后只改离线布局再渲染，实际导航仍1轮；4倍速动图与完整1倍速视频全部保存。前三轮.547/.542/.538ms与本轮均为5090同路线来源成绩；不当作P5本机或严格每帧1ms通过。
- 影响与边界：本机Ubuntu22.04/Humble/RTX5070/9600X与5090的24.04/Jazzy/i9不同，独立本机构建，不执行导入的服务器安装/脚本。无算法/数值调参、无地图重生成、无新增垂直相机、无在线真值授权，不改变累计未知预算或把探索写安全。长尾、非抢占80ms期限、计时/显示长期增长、非全局锚点恢复等仍保留说明；无新增真人/其它路线/实飞/长期验收。沙箱内GPU查询失败已用只读非沙箱确认，不改驱动/全局环境；传输完整约24分钟，未因耗时省略失败或缓存。
- 回滚/备份：原服务器P4与本机P4来源均保留；P5内有服务器保存点、全SHA清单、13适配前文件与补丁、15本机冲突归档。原P4/P3及其它项目未自动同步；不自动删除或回滚已存证据。
- 后续事项：无；本次新P5已就绪，后续开发默认在P5，当前无导航续跑安排。

### 2026-10-18 19:04 — [EGO2P4→EGO2P5] 完成单轮可视化，保存5090当前版本并开始本机完整复制

- 状态：进行中；用户确认可视化效果后要求保存当前版本，将全部内容复制到本机Ubuntu新目录EGO2P5。
- 目的：保留5090已优化代码、所有历史/失败与本轮视频，为本机独立开发建立清晰版本。
- 项目路径：来源ubuntu5090@192.168.31.112:/home/ubuntu5090/dhm/EGO2P4；本机/home/starry/isaac-data/EGO2P4及新/home/starry/isaac-data/EGO2P5。
- 涉及文件：EGO2P4/performance/visualization_20261018_1856/（脚本、结果、视频、动图及逐帧核验），保存检查点；EGO2P5完整副本和适配说明；本项目及HISTORY记录。
- 修改内容：前一轮只新增1次实际导航，随后离线按CSV渲染，字幕修订不重跑导航。现在以5090当前源码作为新P5活动源，并保留本机P4已有历史/诊断中服务器没有的内容；先完整逐项核对，再只适配本机路径/环境与P5身份，不优化算法或调参。
- 验证：新增可视化轮ARRIVED/accepted/safe_arrival真，40.167仿真秒、avoid均.976/P95 3.554/max172.137ms，callback均3.049ms；峰值10/路径均速7.687m/s、308.745m；扫掠/外部保护/限值违规0，但有2个COMPUTE_DEADLINE（t7.333/18.217s）后恢复。1个EXPLORATORY_RECOVER记录、2事件，固定原点/累计预算/命令日志审计通过。与前三轮同源/ELF/有效参数，实际全部线程0..15通过。完整视频634帧/1280x720/15fps/42.267s含起步等待，4倍速动图；不是现场录屏，偏航按GoalTrial规则重建，NAV最大偏差.046度，无新物理推进。视频解码/逐帧核验正在补齐；当前复制尚未开始，不能提前报完整校验或本机运行通过。
- 影响与边界：本机Ryzen9600X只有CPU0..11，不能沿用5090 i9的0..15亲和性；本机Humble/Isaac环境与服务器Jazzy不同，导入的二进制/缓存作为来源证据，须独立构建本地版。性能数字属于5090该路线，不是本机或每帧1ms承诺。保存全部旧失败；不再执行导航。
- 回滚/备份：P4本机与服务器来源目录保留；将建立保存点、全量SHA/类型/元数据清单及适配前备份，无Git提交推送。
- 后续事项：完成保存、全部内容复制/源未变核验、本机环境适配与说明，再按实际本地检查结果收尾。

### 2026-10-18 14:53 — [EGO2P4] 5090同版手柄300m三轮安全到达，完整避障均值稳定.538–.547ms；归档完成

- 状态：完成（限定该路线三轮、10m/s请求上限和完整避障每轮均值<=1ms）；每帧1ms、其它路线、真人手柄/实飞与长期内存稳定性未通过新验收。
- 目的：完成用户多轮卡停分析/优化并保存可复现的5090版本，避免把均值或单次到达冒称普遍安全和严格实时。
- 项目路径：远端`/home/ubuntu5090/dhm/EGO2P4`；本机`/home/starry/isaac-data/EGO2P4`及HISTORY。
- 涉及文件：deployment/rtx5090/{run_validated_goal.sh,RUN_GUIDE.txt,record_affinity.py,audit_probe_trace.py,summarize_trials.py,plot_trials.py,server_source_20261018_verified.tar.gz,server_source_20261018_verified.json,server_snapshot_20261018/}；performance/rtx5090_deploy_20261016/{remote_trials/,validation_logs/,FINAL_ANALYSIS.txt,RESULTS.txt,TRIALS.csv,summary.json,completion_summary.json,CURRENT_CHECKPOINT.json,manual_comparison.png,manual_baseline_comparison_20261018.png,server_solver_optimization_20261018_verified.patch}；README、EGO2P4_VERSION、ACTIVE_NAVIGATION、WORK_LOG与HISTORY/{README,FOV_GVF_FAMILY,ENVIRONMENT_AND_ASSETS,CHANGELOG}。
- 修改内容：最终保留T事件原点/锚点分离、认证接管、原始命中/几何证明缓存等此前已记录修复，正常手柄搜索、history16/1s、cache.75/TTL.3及完整硬约束/风险预算保持；运行入口显式固定manual与CPU0..15，只有本控制器及线程受亲和性约束，无系统设置/其它进程改动。增加独立复现入口、全29条10速成功/失败账本、六条T版风险日志不变量审计、CPU同源对照图、正式构建测试日志、176文件远端源码包和相对本机16文件补丁。本机190份原始源/脚本/profile等SHA复核零差异，远端算法未回灌本机活动树。
- 验证：同一源75f1a16d6f996545c18960215101c7a149be7ea7a7ae95337cc489269b1d90f0、同一ELF de35462afb1a8bb9321a73c3b8b0ec0c704b071541b094024edee9af6bf2b753、同控制参数c7dbd5b40b12f562be57409fc57898aaed1b243148371b38f20019e41e151efc三轮均ARRIVED/safe_arrival/accepted真，仿真45.467/42.417/44.133s；avoid均.547/.542/.538、P95 2.479/1.987/1.813、max33.020/35.401/16.650ms，完整callback均2.736/2.874/3.055ms。<=1ms样本93.51/93.95/94.23%，最差非重叠5仿真秒分箱均.915/.887/.784ms；COMPUTE_DEADLINE0。三轮扫掠/保护/速度/加速度/命令/垂向违规均0，峰值10m/s，路径均速6.733/7.328/7.085m/s，实际Z范围2.905..11.131、5.266..12.846、6.411..13.715m，起终z10不代表恒高。全部线程掩码0..15、实际/冻结产物身份及逐轮源/安装无改动复核通过。主构建3包，ctest44/44、pytest168/168原日志取回；最终壳脚本bash -n通过。六条T版CSV事件起点/预算单调/风险命令不变量全通过；P核三轮无风险动作，不能据此宣称风险/辅助全部分支新飞行验收。当前/proc核对无本项目导航进程，排队已清空。
- 历史纠正与失败：早期“原手柄核心”文件名措辞过宽，原profile本就可用goal几何核接人工意图；策略以use_fixed_goal/提名顺序判定，两当前核均有几何复用，不按basename判定。第一次源码收尾脚本断言完整参数SHA一致失败；核对仅paper_replay_directory/performance_log_path/performance_run_id不同后，使用驱动现有排除记账字段的stable settings SHA，原参数与差异不改。图表首次系统Python命中用户NumPy2/系统Matplotlib ABI冲突失败，改PYTHONNOUSERSITE=1 /usr/bin/python3生成并目视核验成功；无安装/降级。旧6速中断、29条10速账本内13到达（含1身份拒绝）、13卡停、2主动循环中断、1完整600s超时均保留；没有执行的排队轮次不算通过。未绑核同源T版2/3、1.787/2.368/13.592ms与P核三轮并列，时序/路径不同，不把全部收益独归因GPU/CPU。
- 影响与边界：无新增垂直相机或在线地图真值；.595完整物理跟踪半径+restart.02、累计3.5m/12活跃秒/5m³保持。恢复额外舒适带.08中的内半带.04沿用本次已记录标定，不能宣称无安全策略变化或碰撞概率保证。更大history32和2m提名位移均未采用。无后台自动续跑、无Git提交推送、其它项目未改。
- 回滚/备份：before_adaptation、runtime/source_backups各阶段tar和每轮frozen保留。最终源码压缩包SHA256 42b0da7fcc5f45d116616369b298351b7874c411f6752238f51664a819ae59ff，取回后176文件逐一SHA通过。更改亲和性只需另用run_goal.sh并取消CPU环境变量；算法回滚须从选定备份恢复并重新构建，不能把旧性能当当前验收。
- 后续事项：本请求范围已结束；其它路线、实物手柄、长时和严格每帧期限另行验收，本轮无续跑安排。

### 2026-10-18 14:28 — [EGO2P4] T版回归44/168，发现P/E迁移；仅P核首轮安全到达且均值.547ms

- 状态：部分完成；仅P核首轮已到达且完整避障均值<=1ms，稳定重复仍待两轮，不能提前宣布稳定通过。
- 目的：针对代码优化后仍出现的长尾，核验5090异构CPU运行环境对实时计算的影响。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本机EGO2P4/performance/rtx5090_deploy_20261016。
- 涉及文件：deployment/rtx5090/{record_affinity.py,audit_probe_trace.py}、reentry_origin_candidate、remote_trials、CURRENT_CHECKPOINT与双重记录；当前三轮期间不再改源/安装。
- 修改内容：T版源已通过正式主构建3包、44核心、168工具；仅新增独立运行配置EGO5090_CONTROLLER_CPU_AFFINITY=0..15，仍原手柄base策略、10/垂速1、depth20/H.5/cache.3/.75、history16/1s、原预算与硬安全约束。只对本控制器及其线程执行taskset，不更改其它进程/系统调度/CPU频率。亲和性工具从实际参数读取请求CPU集合并核验实际ELF及全部线程。新增CSV账本审计仅检查活动事件起点固定、预算单调和EXPLORATORY命令/预算限值，不冒称观测或碰撞概率审计。
- 验证：T版未绑核三轮：r01安全到达54.367s、avoid1.787/P95 6.808/max249.435ms，callback6.083；r02安全到达42.600s、2.368/7.936/441.800ms，callback5.723；r03卡停109.850s，终点距61.949m，13.592/43.347/803.267ms，callback25.619，主因PROBE_NO_USEFUL_PATH53.817s；三轮扫掠/保护/限值违规0，仅2/3到达，不能混作3/3。原点账本r01 3262行/34个风险记录/1事件，日志不变量违规0。只读采样本控制器主线程20次/约.5秒，实际CPU分布跨0,1,2,4,8,10,11,15,16,17,18,19,26,30，证实跨P/E迁移。仅P核独立首轮ARRIVED45.467s、safe_arrival/accepted真，avoid均.547/P50 .288/P95 2.479/max33.020ms，完整callback均2.736/P95 11.253/max46.911ms，扫掠/保护/限值违规0、峰值10；实际线程掩码/身份报告已产生，最终逐轮复核待归档。该轮没有改算法、缓存位移或风险额度，GPU缓存已在此前多轮暖机。现在仅按相同配置重复r02/r03，尚未执行完成。
- 影响与边界：此对照区别于已拒绝的4线程CPU集合+探测优先+2m缓存组合；本次允许0..15全部P核逻辑线程，保持正常绕障搜索。均值.547不等于每帧/全回调1ms；P95仍2.479。没有新增垂直相机或人为把未知写成安全。首轮不能证明其它路线/真人手柄/实飞稳定。
- 回滚/备份：T版构建前before_reentry_origin_20261018.tar.gz与所有旧版本/失败冻结保留；去掉CPU环境变量即回到T版未绑核参数，不需修改系统。前三轮通过的早期base版本before_probe_pcore tar继续保留。
- 后续事项：完成同版同参数另外两轮，到达/求解/帧长尾/CPU/风险账本全审计；再保留复现入口和源码快照，按实际结果判定1ms目标。

### 2026-10-18 14:20 — [EGO2P4] 修复可靠锚点与事件基准混用，加入完整认证接管恢复和T版审计

- 状态：进行中；1ms与稳定到达联合目标仍未通过，未将成功单轮或部分轨迹当完整验收。
- 目的：继续分析多轮真实卡停，避免恢复目标退化和近终点反复返回，同时降低重复查证成本。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本机EGO2P4/performance/rtx5090_deploy_20261016。
- 涉及文件：远端paper_guidance.hpp、{active_navigation,paper_guidance,paper_replay,depth_angular_controller_node}.cpp、active_navigation_check.cpp、native_history_recovery_check.cpp、scripts/isaac/run_fov_gvf_navigation.py；本机execution_admission_candidate、reentry_origin_candidate、native_hash_candidate、remote_trials与双重记录。
- 修改内容：先分开当前完整执行起步(.595+.02)和可靠返回点内侧(.595+.04)条件，可靠锚点必须仍有4cm内侧证明，无效时从真实已到达位置选取，移动锚点不刷新预算。真实209m卡停快照机体最近原始命中.647072m，当前位置有restart证明但无4cm内侧证明，旧锚点却等于当前位置；修复后同快照由PROBE_NO_USEFUL_PATH转EXPLORATORY_RECOVER。进一步把可移动返回锚点与固定探测事件起点/时刻分离，预算恢复的真实位移与新鲜关键观测从事件起点计算，不能因锚点随已知飞行移动而丢失位移基准。恢复中允许一个已完整认证的低速前向短段接管，风险额度保留；已有完整起步证明时未关闭事件也能进入正常绕障，不跳过正常绕障搜索。新增T版重放保存事件起点/有效性/时刻，兼容3—S，旧S缺起点不能据此补充额度；诊断JSON和CSV增加事件活动/起点字段。前端非求解缓存作用域内优先测试刚已认证的实际位置球，使用与原扫描相同包含式，冲突撤销/地面/完整认证都保持。
- 验证：几何缓存对照r01卡停97.417s/距终点86.021m，avoid均39.689/P95 426.269/max1008.507ms；r02安全到达48.317s、均.977/P95 4.732/max86.629ms，仅1/2到达，r03未执行。第一版可靠锚点修复主构建/44核心/168工具通过；r01 ARRIVED46.667s，均1.133/P95 4.469/max232.789ms、callback4.748；r02 ARRIVED54.267s，均3.781/P95 10.161/max677.612ms、callback7.957；r03在约(291.325,-36.400,11.298)近终点循环，t394.9经60s位移范围[.19412,.05808,.04319]m判据后中断，单列INTERRUPTED_CYCLIC_RECOVERY。原驱动RUNTIME_ERROR/protocol缺失未改，部分轨迹扫掠/保护/限值0、峰值10、avoid均6.667/P95 27.480/max301.338ms，不算完整到达。可靠锚点版2/3完成到达，不能只看前两轮。新事件原点/认证接管本机全核心静态库编译、主动导航与扩展回归通过（隐藏顶面/松杆/无新信息耗尽/精确命中/预算/重放）；旧S真实快照可读且转风险恢复。新增过期观测拒绝恢复额度和最新足迹快速正证明之后，5090正式44/168及三轮正在构建，尚未飞行。独立native hash表候选同冻结帧5次输出一致、局部solve中位34.263→29.524ms约14%，但未推广，不能写成整轮收益。一次CSV插入锚点字符串断言失败，修正括号锚点后AST通过；一次未等rsync完成读取快照失败，传输完成后重放成功，无生产崩溃。
- 影响与边界：正常10m/s请求/原相机/地图/完整硬包络/累计3.5m、12s、5m³保持；恢复舒适带标定沿用既有已记录4cm内半带。新T版只是记录完整状态，不把未知风险写成认证。没有采用退化的探测优先/绑4核/2m缓存组合。当前只有本任务进程信号，暂停排队均已清理。
- 回滚/备份：runtime/source_backups/before_anchorproof_20261018.tar.gz和before_reentry_origin_20261018.tar.gz；每轮源/安装冻结保留。190份本地原始活动源码继续保持，远端优化仍在指定根目录。
- 后续事项：完成新T版44核心/168平台及三轮同路线/手柄策略审计，实际完整平均1ms不提前宣称；更新最终源码快照、部署说明与HISTORY项目索引状态。

### 2026-10-18 13:47 — [EGO2P4] 丢弃探测优先/绑核/缓存增大退化组合，恢复三轮通过逻辑并单测证明缓存

- 状态：部分完成；真实手柄基线3/3到达已保留，新组合0/2到达且退化，不作为当前验收版本。
- 目的：不以最大耗时缩短换取卡停或高平均计算开销，继续尝试保持1ms与到达。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本地EGO2P4/performance/rtx5090_deploy_20261016。
- 涉及文件：远端src/pc_gvf/CMakeLists.txt、active_navigation.cpp、paper_guidance.cpp、native_history_recovery_check.cpp；本机geometry_cache_candidate、plot_trials.py、remote_trials、双重记录。
- 修改内容：恢复前3轮手柄通过的两算法源文件，已在本机按其冻结manifest逐字节SHA验证一致；取消CPU亲和性与2m提名位移，恢复.75m/TTL.3。仅将已有PC_GVF_GOAL_TUBE_REUSE几何证明缓存也编译进手柄基础核心，复用当前观测/当前步内已完整证明球和固定管段几何；不缓存执行许可、不跳过全路径/制动、不减少1024节点/7层证明深度。Node仍use_fixed_goal=false。CPU前缀/环境记录能力保留但本组三轮不启用。
- 验证：退化组合主构建3包、44核心/168平台通过；两个实际CPU核验均确认base ELF及所有线程8,10,12,14。r01在92.383s卡停、终点距86.520m，avoid均17.301/P95 55.001/max155.853ms、callback29.804；r02在92.217s卡停、距106.167m，均17.754/P95 37.838/max159.332ms、callback21.628；两轮扫掠/外部保护/限值违规0但未到达。r03未执行，排队父进程已清理。减少全角域回退会把需要正常绕障搜索的场景过早交给低速探测/恢复，组合出现长期无路；因同时调整三项，不能独立归因于绑核或缓存位移。已有手柄原基线仍3/3到达，避障均1.103/1.889/1.068ms。纯几何缓存候选开始主构建，回归/新三轮尚未运行，不提前报成功。
- 影响与边界：仍是300m、起终z10、10m/s请求上限、无垂直相机、.595完整包络/执行含.02restart、累计预算3.5m/12s/5m³。卡停轮未因均值或max更好而算成功，全部失败保留。首次Matplotlib默认配置目录不可写后自动改/tmp临时缓存，图已生成/视觉检查；脚本现显式缓存到本项目性能目录。
- 回滚/备份：runtime/source_backups/before_probe_pcore_20261018.tar.gz保留通过基线，before_geometry_cache_20261018.tar.gz保留退化组合；逐轮冻结源/产物与SHA不改。本机190份原始活动源复核均未改，所有算法部署在指定5090根目录。
- 后续事项：完成纯几何缓存44核心/168平台与三轮手柄闭环；比较三轮平均/长尾/到达，未同时改其它策略。

### 2026-10-18 13:40 — [EGO2P4] 实际手柄核心3/3安全到达，均值1.068–1.889ms；启动求解与CPU优化对照

- 状态：部分完成；300m/10m/s实际手柄核心已连续三轮通过到达审计，稳定1ms仍未通过。
- 目的：保持真实手柄策略并压低完整避障均值与1.085s长尾。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本机/home/starry/isaac-data/EGO2P4。
- 涉及文件：deployment/rtx5090/{environment.sh,run_goal.sh,run_route300.py,record_affinity.py,summarize_trials.py}；隔离probe_first_candidate、pcore_candidate；远端src/{active_navigation,paper_guidance}.cpp、native_history_recovery_check.cpp、navigation_benchmark.launch.py、tools/sim4_benchmark.py；remote_trials与双重记录。
- 修改内容：修复environment.sh重复source强制覆盖控制器的问题，显式选择在嵌套Shell中保持，默认手柄核心，固定目标对照可选goal核心。三轮基线用真实base控制器、use_fixed_goal=false、原手柄候选顺序。新增候选在无方向提名/全运动认证失败时先评估主动认证与有界风险探测，避免反复全角域回退；水平未知探测加入±30/60度低速视差候选，完整认证/占据硬约束/累计预算仍有效。CPU入口只对本控制器执行taskset8,10,12,14，记录/校验实际ELF和所有线程掩码；没有改其它进程亲和性或系统策略。方向提名位移.75→2m，TTL.3不变、不缓存执行许可。manifest补记CPU/policy/cache环境。
- 验证：最初手柄policy轮ARRIVED45.467s、avoid均.970/P95 3.654/max97.117ms、扫掠/保护0，但嵌套环境把实际ELF覆盖为goal而冻结预期base，frozen_execution_matches失败，safe_arrival/accepted均假，完整原始失败保留，不能算手柄核心验收。修复后两显式内核的重复source2/2通过；真实base三轮均ARRIVED、safe_arrival/accepted均真：r01 50.683s，avoid1.103/P95 4.512/max205.129ms、callback5.397；r02 40.567s，1.889/4.878/1084.617ms、callback5.712；r03 48.683s，1.068/4.638/93.553ms、callback5.360。三轮扫掠/保护/限值违规0、达10m/s、终点距均约.492m。r02前端均.353/core1.535ms，最大帧t7.55前端5.324/core1079.291ms、快照.000097ms，证实长尾在核心而非文件快照；r01前端.338/core.765ms。探测优先本地新增回归通过，无提名构造返回ACTIVE_CERTIFIED .5m/s；旧冻结快照proposal=false不覆盖新分支，不能拿其耗时冒充改进。CPU前缀实际ROS参数构造通过，非法CPU99999拒绝、Node.execute未调用。新组合正在5090构建/回归，未跑三轮，不提前声称1ms通过。
- 影响与边界：当前完整硬包络与风险预算保持，未知动作仍独立EXPLORATORY、不得写自由空间。测试以自动目标方向替代物理手柄，未做真人手柄/实物飞行；起终高度10m，实际绕障高度变化另由轨迹报告。每轮冻结期间代码/安装无改动，旧退化/身份失败均保留。
- 回滚/备份：runtime/source_backups/before_probe_pcore_20261018.tar.gz保存前三轮通过版本src/tools，旧版本tar/逐轮内容寻址冻结继续保留；未提交推送。
- 后续事项：新组合44核心/168工具后再三轮完整手柄验证，对比到达、均值、长尾及预算；未达标则保留已验证基线并如实记录。

### 2026-10-18 13:33 — [EGO2P4] 记录恢复循环中断、固定目标退化，纠正完整手柄测试入口

- 状态：进行中；全部联合验收目标仍未通过。
- 目的：落实用户“用手柄参数算法”的要求，明确固定目标排序与完整手柄链路差别，并继续降低无效求解。
- 项目路径：本机EGO2P4及远端/home/ubuntu5090/dhm/EGO2P4。
- 涉及文件：远端navigation_benchmark.launch.py；deployment/rtx5090/{run_goal.sh,run_route300.py,audit_interrupted.py}，manual_policy_candidate、probe_first_candidate、CURRENT_CHECKPOINT、remote_trials与双重记录。
- 修改内容：新增EGO5090_MANUAL_POLICY，运行封装默认1，use_fixed_goal=False并选原手柄depth_angular_controller；GoalTrial仍只发布/human_intent并独立审计到达，10m/s/垂速1/depth20/H.5/C.3及三主动策略不变。0保留固定目标专用排序供对照。wrapper记录policy、缓存位移、history容量和实际profile/入口SHA。此前测试仅用了手柄参数，固定目标专用候选优先级与goal几何复用核心仍在，不能称完整手柄链路；旧结果均明确归固定目标对照。
- 验证：strict_first075首轮在x230.9m确认连续60s位置范围[.09664,.09366,.03580]m，t276.533中断，标记INTERRUPTED_CYCLIC_RECOVERY；原驱动因中断写RUNTIME_ERROR/audit_error protocol，原记录不改。新增部分轨迹独立扫掠审计调用相同几何函数，0重叠/保护/限值违规、峰值10；该前缀avoid均8.536/P95 37.675/max2194.897ms，不能作为完整到达。旧strict第2/3轮未运行。随后恢复舒适带+.02交接/制动必要前检/4线程版主构建、44核心、168平台通过；固定目标r01卡停77.567s、终点距246.632m，avoid均88.281/P95 896.122/max2322.908ms，r02卡停83.167s、距126.951m，均60.301/P95 523.445/max958.525ms，两轮扫掠/保护/限值违规0，未到达。r03未执行。4线程本地冻结帧3次中位62.622ms对2线程104.774ms、输出一致，仅支持该帧，不是5090整轮性能收益。手柄入口实际ROS参数构造通过，Node.execute未调用，原手柄核心、use_fixed_goal=false及上述参数核实；完整手柄控制对照正在启动。
- 影响与边界：当前主动风险完整硬包络(.595，执行含restart .615)保持；恢复只对可调额外.08舒适带采用内半带.04下界，已经更近的点只能远离，仍需预算/制动/返回。早先“半径不改”针对完整硬约束，不能把软缓冲标定写成无任何安全策略变化。没有概率保证；未新增传感器。排队父进程暂存SIGSTOP仅阻止旧版本后续排队，当前仿真不受影响；结束后已TERM/CONT清理，最初清理断言因僵尸子进程失败，核对Z状态后完成；没有其它项目进程信号。一次停止循环判据失败未发信号，后续满足才中断。
- 回滚/备份：全部旧源码tar、逐轮冻结原始产物与interruption.json；手柄入口前launch备份在runtime/source_backups/navigation_benchmark.before_manual_20261018.py。本机权威src未改。
- 后续事项：完整手柄至少一轮对照后再测probe-first候选；该隔离候选优先评估有界探测而非反复全角域求解，新增±30/60度低速水平视差候选；尚未推广/5090回归/飞行。稳定到达与1ms均未通过。

### 2026-10-18 13:22 — [EGO2P4] 两次交接失败及制动软缓冲根因；完整认证优先版运行中

- 状态：部分完成；尚未通过稳定到达与完整避障均值1ms。
- 目的：持续按冻结失败状态修复，不省略失败或用预算刷新绕过恢复。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本地EGO2P4/performance/rtx5090_deploy_20261016。
- 涉及文件：远端active_navigation.cpp、depth_angular_controller_node.cpp、native_history_recovery_check.cpp；本地handoff_candidate、strict_first_candidate、alternate_return_candidate、clearance_diagnostic、recovery_clearance_candidate、emergency_precheck_candidate、三份diagnostic_*.bin、remote_trials与双重记录。
- 修改内容：第二版在完整内侧锚点返回完成时先退出恢复，正常分支日志保留持续探测预算；avoid计时起点前移到activeNavigationRequested之前。第三版将有完整新鲜运动/制动认证的水平候选置于风险占据网格与可见性评分之前，仍有原始点当前机体硬否决；原始点先按相机局部距离保守预筛减少远点变换。正常已知运动持续更新可证明锚点但不刷新事件额度；正常飞行也收集关键目标新观测。制动路径的额外舒适缓冲也可触发原有单调退回规则，完整半径/预算不改。前三项已部署且44核心通过。
- 验证：handoff075首轮ABORTED_EXPLORATION_STALL106.15s、终点距49.525m，avoid均13.525/P95 31.641/max1860.235ms；handoff2首轮70.633s、终点距256.886m，均21.771/P95 35.061/max366.810ms。两轮扫掠重叠/外部保护0；第二版44核心与168平台工具通过。第一轮制动快照真实点最近.681806m>完整rollout半径.615m，却进入额外.08缓冲；锚点已落后8.14059m。第三版重放由PROBE_UNSAFE_BRAKE转EXPLORATORY_RECOVER。第二轮无路快照真实点最近.668206m；返回线接近新点由.702273降到.694155m，被额外缓冲拒绝，未穿过.615硬半径；原.04交接条件失败而实际restart .02证明存在。离线候选把交接条件对齐实际restart余量，恢复时外层可调舒适带由.08采用内半带.04下界，原本已更近的点仍只能远离；完整硬半径不变。该候选仅本地合成窄通道/原有原始历史/危险障碍/松手预算等回归通过，两个真实失败快照转为EXPLORATORY_RECOVER或完整认证运动，尚未部署。替换备用锚点候选对无路快照无效，未推广。独立测试最初误用本地旧4参数缓存头文件编译失败，改隔离include并链接实际原始历史修复对象后通过；失败输出保留于工具记录。
- 影响与边界：strict_first首轮自然运行中，t199.3已至x230.87，曾在132m反复恢复但后续继续推进；尝试的严格60s/1m循环判据未满足，脚本未发信号。没有将这轮当作到达、没有减少实际机体/跟踪余量、没有新垂直相机；外层缓冲候选是工程标定，非概率保证。新增必要制动证明前置与4线程角域候选仅本地诊断准备，尚未生产验证。
- 回滚/备份：runtime/source_backups/before_handoff{,2}_20261018.tar.gz、before_strict_first_20261018.tar.gz和逐轮冻结产物；本机权威src未动。
- 后续事项：完成本轮自然运行，测试恢复舒适带/实际restart交接候选并重复；量化前端/核心/快照/回调长尾与1ms差距。

### 2026-10-18 13:07 — [EGO2P4] 继续5090多轮卡停分析，修复恢复后正常绕障交接

- 状态：进行中；上一轮锚点/缓存两对照均失败，1ms与稳定到达未通过。
- 目的：继续用户授权的10m/s、约300m、中间高度路线测试，定位超时与恢复循环。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本机/home/starry/isaac-data/EGO2P4。
- 涉及文件：performance/rtx5090_deploy_20261016/handoff_candidate/{active_navigation.cpp,native_history_recovery_check.cpp,diagnostic_replay.cpp}，新收集两轮remote_trials和diagnostic_budget.bin，双重记录。生产替换前留独立备份。
- 修改内容：候选允许仍有累计探测事件、但当前位置完整内侧已获证明且不在恢复/辅助升降的帧返回正常绕障前端；跨交接继续累计测得未知距离、时间、体积，重复predicate不重复计费，不刷新额度。恢复未完成仍强制主动分支，原半径/风险预算/相机/速度不改。
- 验证：重新SSH连接并确认无本项目导航运行；旧两组完整结果已收集。cache.75：120.733s/238.637m、停(217.420,-42.453,10.301)，PROBE_BUDGET_EXHAUSTED；avoid均15.536/P95 31.341/max1955.813ms，callback均20.700。cache2：600s完整TIMEOUT/447.750m、终点距178.623m，ACTIVE_CERTIFIED_RECOVER546.117s；avoid均26.022/P95 42.094/max217.197ms，callback均31.073。两轮扫掠重叠/保护/限值违规均0、峰值10，但均未到达。轨迹表明探测事件保持开启时正常绕障前端被长期跳过，回锚点后直冲目标再返回。新候选尚未构建/测试/飞行。初次只读SSH内联引号错误导致Python语法失败，改独立stdin脚本后读取成功；远端无rg，进程查询改Python过滤，未安装依赖。
- 影响与边界：本机权威src暂不改；候选仅部署指定远端根目录。返回已知空间不自动清除探测事件；不宣称概率安全或平均1ms通过。
- 回滚/备份：baseline/native-history tar及每轮冻结源/产物仍保留；本轮先备份当前src为before_handoff_20261018.tar.gz。
- 后续事项：候选回归、失败快照离线诊断、实际多轮与耗时/安全审计。

### 2026-10-16 19:33 — [EGO2P4] 原始历史修复2轮1到达；继续退化锚点与方向缓存优化

- 状态：部分完成；300m原始历史版1/2安全到达，稳定到达与1ms未通过。
- 目的：按用户多轮卡停分析与优化要求，保留失败、对新原因逐项修复。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本机隔离候选在performance/rtx5090_deploy_20261016。
- 涉及文件：远端active_navigation.cpp、goal_proposal_cache.hpp、depth_angular_controller_node.cpp、isaac_cloud_navigation.launch.py、native_history_recovery_check.cpp；本机anchor_cache_candidate及双重记录。远端原始历史两文件修改已保留。
- 修改内容：新增仍有radius+.04内侧证明的邻近已到达球/起步球选择，解决recovering时锚点不足1mm且无重新起步余量的退化；选新锚点不重置预算/事件/关键观测。方向提名缓存增加可配置max_displacement，默认.75m保持，仅5090对照尝试2m；TTL.3、意图变化撤销与每帧完整新鲜运动认证不变，不缓存执行许可。
- 验证：原始历史版主构建3包、44/44核心、168/168平台工具通过。新路线r01 ARRIVED53.4667s/314.209m/峰值10，独立safe_arrival/accepted均真，保护/扫掠重叠/限值违规0；avoid均1.691/P95 3.777/max1686.457ms，完整callback均4.133ms。3025帧分解frontend均.562/core均1.128ms，最大帧t15.533 core1686.454ms，非快照开销。r02停于(188.970,-42.666,15.746)，PROBE_NO_USEFUL_PATH，锚点仅距.0008m，未知距离.0423m/活跃2.083s/体积.929m³，预算未耗尽；ABORTED_EXPLORATION_STALL89.55s/200.503m，avoid均17.627/P95 39.430/max1588.101ms，保护/扫掠重叠0。不能只报告r01。
- 影响与边界：新增候选尚待构建回归与整机；不改场景/传感器/10m/s上限/完整认证，风险额外缓冲退让规则及原始点宽窄阶段保持。1ms和稳定到达均明确未通过。
- 回滚/备份：原始历史前源码tar及本轮前快照，旧失败原始轨迹/重放和每轮冻结源/产物保留；无提交推送。
- 后续事项：先验证新增锚点不刷新预算，再原.75m缓存与2m缓存对照及重复运行，继续逐帧超时分析。

### 2026-10-16 19:27 — [EGO2P4] 5090原始历史命中与缓冲层恢复候选进入构建

- 状态：进行中；不是已通过的1ms/到达验收。
- 目的：用户要求多轮分析超时卡停后优化，保持10m/s、300m/中间高度路线与三主动策略。
- 项目路径：远端/home/ubuntu5090/dhm/EGO2P4；本地候选在EGO2P4/performance/rtx5090_deploy_20261016/native_history_candidate。
- 涉及文件：远端src/pc_gvf/src/{paper_guidance,active_navigation}.cpp、src/pc_gvf/CMakeLists.txt及新增test/cpp/native_history_recovery_check.cpp；本机仅候选/诊断和双重记录，权威src不改。
- 修改内容：有界探索开启时16份历史保留原始命中，修复保守缩图深度被投影到另一条射线后冒充真实历史占据。风险碰撞检查保存体素桶内全部原始点，代表点+对角线只做宽阶段，精确原始点做窄阶段；完整机体/跟踪余量不变。额外8cm舒适缓冲内只准不进一步靠近测得障碍的恢复，并保留完整包络、制动、返回与预算；recovering显式触发主动分支，内侧覆盖与锚点接近同时满足才退出。自由空间认证、风险预算/相机/10m/s参数不变，原始已知障碍不忽略。
- 验证：6m/s冻结卡停帧当前真实命中最近.704660m，历史代理view3/stamp19.0167为.558353m、stamp20.0167为.564983m，均落入.595m包络；诊断hits.txt保留。仅缓冲修复仍同快照ACTIVE_KNOWN_OBSTACLE；保留原始历史须新仿真，旧快照不可补造。恢复/精确点两隔离库分别通过原有主动回归与新增缓冲退让/实际包络内障碍拒绝用例；第一次临时测试把main改名后缺显式return引起段错误，修正测试工具后通过，失败日志保留。候选正式44项核心/168平台与整机多轮尚未执行。
- 影响与边界：原始历史增加主机内存与重放体积，数量仍16，未盲增32。已知真实点仍硬拒绝，探测仍不写永久自由证明；新恢复是工程策略，不宣称文献概率安全保证。此前纯几何/memo/球对候选未推广。
- 回滚/备份：远端runtime/source_backups/baseline_src_before_solver_optimization.tar.gz；适配前备份与baseline冻结产物继续保留，未提交推送。
- 后续事项：构建并修复真实失败、44核心/168平台、同新路线多轮10m/s与扫掠/风险/耗时审计；1ms目标尚远未通过。

### 2026-10-16 19:18 — [EGO2P4] 新300m/10m高度基线失败，按1ms目标进入卡停恢复优化

- 状态：持续优化中；部署成功、统一核心42/42/平台168/168，10m/s到达与1ms目标均尚未通过。
- 目的：用户追加要求多轮记录分析卡停并优化代码/算法，保留10m/s、无垂直相机、执行/证据连续性、辅助螺旋和有限风险探测恢复。
- 项目路径：本机EGO2P4及远端/home/ubuntu5090/dhm/EGO2P4。
- 涉及文件：deployment/rtx5090/{run_goal.sh,run_route300.py,choose_route300.py,route300.json,make_query_candidate.py,make_memo_candidate.py}；performance/rtx5090_deploy_20261016下query/memo/pair候选；WORK_LOG与HISTORY总账。所有候选仍是隔离文件，生产核心源码未修改。
- 修改内容：新路线(-5.2,-34.4,10)→(294.8,-34.4,10)，300m，nominal高度0..20中点10；两个端点按原保守2.5m起步球检查。wrapper保留原驱动冻结/传感器/轨迹审计，协议实际start/goal写入SHA。以完整avoid平均为主，兼看P95/max/期限与端到端计时，不把核时间单独当整体1ms。
- 验证：旧449m手柄10/history16行程116.511m，avoid均53.371/P95 481.989/max661.604ms；history32仅26.625m，均104.917/P95 322.532/max424.858ms，增容未改善。新300m基线行程88.924m，均61.763/P95 315.241/max440.022ms，距终点213.384m；三轮均ABORTED_EXPLORATION_STALL（停车60秒，不是600秒超时），保护/扫掠重叠/限值违规均0，未到达。perf被服务器perf_event_paranoid=4拒绝，未改系统设置，独立gprof表明约58%在包含扫描、29%在相机球体覆盖。远端最终统一核心42/42通过。三隔离候选同快照输出均ACTIVE_KNOWN_OBSTACLE，单本机含加载墙钟query0.661/memo0.816/pair0.672秒对基线0.834秒，均未接近1ms且未用于新飞行。
- 影响与边界：候选对照仅单快照墙钟，不等于求解平均。新路线初选粗列表无有效端点，系统Python受用户NumPy2/SciPy1不兼容失败，改PYTHONNOUSERSITE=1后成功；全部失败保留。尚未推广增容或候选；原源/安全参数/地图不变。
- 回滚/备份：远端runtime/source_backups/baseline_src_before_solver_optimization.tar.gz保存完整src/scripts/profiles/tools；候选.patch与源码单独保存，本地权威主源码未动。
- 后续事项：重新核对所引文献与三项现有实现，分析恢复初始额外净距门槛及长求解重复fallback；按用户要求继续多轮同路线对照。文献本轮已打开FaSTrack/FASTER/CPA/SCOPE/RAS/Pairet原文入口，PA-MPPI初次打开失败后搜索找到作者PDF，全文核对继续中。

### 2026-10-16 19:03 — [EGO2P4] 按新指令切换完整10m/s手柄profile，准备存储容量对照

- 状态：进行中；5090部署3包已构建，10m/s实际闭环运行中。
- 目的：用户明确要求使用原5070保存的手柄参数算法，在5090测试完整起终点；允许增加需要性能的存储部分以改善表现。
- 项目路径：/home/starry/isaac-data/EGO2P4；远端/home/ubuntu5090/dhm/EGO2P4。
- 涉及文件：deployment/rtx5090/run_goal.sh；performance/rtx5090_deploy_20261016离线快照/重放及阅读记录；WORK_LOG.md/HISTORY总账。远端实验在performance/p4_speed_20261016/rtx5090_deploy_20261016。
- 修改内容：run_goal先加载manual_10mps_depth20.env，再固定goal协议为10m/s、垂速1、depth20/H.5/cache.3、a16/tau.10、E与三主动策略开，保留原16份历史作基线；增加EGO5090_HISTORY_MAX_OBSERVATIONS用于32份对照。部署探索连续卡停60秒显式失败，EGO5090_ABORT_STALL_S=0可关闭，不伪称完整600秒超时或稳定统计验收。人工输入被目标指向替代，保存profile其余算法参数不变。
- 验证：远端3包构建33.6秒完成；首轮CTest39/42、平台166/168，缺冻结旧地图/协议/汇总脚本；补齐后平台168/168，核心40项通过、旧地图两项待传输完成后补跑2/2通过。前置输出目录保护拒绝1次，修正wrapper输出至允许范围后启动。6m/s轮停于(36.430,-22.151,4.894)，持续COMPUTE_DEADLINE、保护0；按用户新指令中止，interruption.json解释driver产生RUNTIME_ERROR/audit缺protocol，不当作自然超时。冻结同帧本机/5090纯核心重放分别0.834/1.14秒，均ACTIVE_KNOWN_OBSTACLE/零命令，证明不只ROS/渲染耗时。10m/s实际参数已读回一致，尚待结束。
- 影响与边界：未改算法源码、安全阈值、地图、风险预算或远端全局环境；更大历史只作容量实验，不预设收益。显示地图进程未在goal测试中启动，增大显示存储不能改善此轮控制链。
- 回滚/备份：主算法原样保留，5项远端适配前备份与SHA见deployment/rtx5090；6m/s失败/中止全部保留，未提交推送。
- 后续事项：10m/s基线与32份历史对照、完整轨迹审计、最终统一回归和记录同步。

### 2026-10-16 18:50 — [EGO2P4] RTX 5090独立目录部署与起终点验证进行中

- 状态：进行中；已读权威文档/核心代码并连接服务器，未宣称新导航通过。
- 目的：按用户要求在5090的/home/ubuntu5090/dhm/EGO2P4新建部署，适配环境后测试起终点。
- 项目路径：/home/starry/isaac-data/EGO2P4；ubuntu5090@192.168.31.112:/home/ubuntu5090/dhm/EGO2P4。
- 涉及文件：新增deployment/rtx5090/{adapt_remote.py,environment.sh}及performance/rtx5090_deploy_20261016；本WORK_LOG和HISTORY总账。
- 修改内容：本地算法源码不变；远端副本计划适配固定项目路径、ROS Humble→Jazzy、现有conda Isaac入口，构建/缓存/日志均放指定目录；远端适配前逐项备份并记录SHA。不改系统依赖及其他项目。导航锁因目录隔离迁至runtime/navigation.lock，ROS域42与单发布者检查保留，运行前检查现有任务。
- 验证：首次SSH在沙箱因socket权限失败，允许网络执行后连接成功；服务器Ubuntu24.04.4/Python3.12.3/ROS Jazzy、RTX5090 32GB/驱动595.84、Isaac6.0.1-rc.7现有环境已核对；指定目录原先不存在，已新建，传输进行中。构建/测试/飞行待执行。
- 影响与边界：历史P3保护287帧、36次期限拒绝仍为继承证据，不当作P4新成绩；不修改P3/P2、远端系统ROS/conda/驱动与其他项目，无提交推送，未存储密码。
- 回滚/备份：原本地源码保留；远端deployment/rtx5090/before_adaptation将保存适配前文件，恢复后需重建。
- 后续事项：完成构建、回归、完整原地图起点(-64,-11.2,2.8)至目标(384,11.2,2.8)实际运行与独立审计，追加最终实测记录。

### 2026-10-16 18:31 — [EGO2P4] 保存P3并完整复制建立P4，整理算法与当前状态

- 状态：完成；复制后独立构建/回归通过，未新增飞行。
- 目的：用户要求保存当前版本，将P3全部内容复制到P4，并说明当前代码算法、项目进展和状态。
- 项目路径：/home/starry/isaac-data/EGO2P3；/home/starry/isaac-data/EGO2P4。
- 涉及文件：P3保存点performance/saved_current_20261016_1823及README.md/EGO2P3_VERSION.md/WORK_LOG.md；P4全目录复制；P4脚本build_isaac_ros_workspace.sh/run_isaac_fov_gvf_navigation.sh，profiles/manual_10mps_depth20.env，src/pc_gvf/launch/{isaac_cloud_navigation,navigation_benchmark}.launch.py，src/pc_gvf/src/depth_angular_controller_node.cpp；tools/{sim4_benchmark,speed_benchmark,run_navigation_benchmark,verify_navigation_replays,run_omni_comparison,analyze_navigation_benchmark}.py；README.md、AGENTS.md、MANUAL_CONTROL_PROFILE.md、ACTIVE_NAVIGATION.md、EGO2P4_VERSION.md、WORK_LOG.md，新增EGO2P3_原文文档备份和performance/copy_from_ego2p3_20261016证据；HISTORY/CHANGELOG.md、PROJECT_INDEX.md、FOV_GVF_FAMILY.md、README.md。
- 修改内容：先保存活动源tar/SHA、Git现状、6项执行产物和最新181424外部日志/重放/截图；再cp -a --reflink=auto原样复制31,838文件/33,477条目、4,907,769,135字节，含.git/隐藏文件/缓存/全部失败历史。原样SHA/元数据差异0、共享文件inode0。P4仅13项运行路径/身份/说明适配及新文档：独立/tmp/fov_gvf_ego2p4_isaac_*、机器人/新实验标签ego2p4，保留旧比较算法/schema/历史标签、同域42/公共锁。当前统一三段渐变原地图、三项主动策略、10m/s保存profile及6m/s默认、所有预算/数值参数不变。
- 验证：主构建Shell3包成功；PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego2p4_isaac_build/pc_gvf --output-on-failure -j 2为42/42；ROS环境/usr/bin/python3 -m pytest -q EGO2P4/src/pc_gvf_platforms/test EGO2P4/tools/test_sim4_benchmark.py EGO2P4/tools/test_speed_benchmark.py为168/168；保存/默认/goal源安装配置6/6通过（Node.execute=0）；Q/R/S各一帧×双核心6对P3/P4重放输出一致，两重放器SHA相同；bash -n通过。最终来源全树核验与允许差异见final_inventory_check.json；逐项结果见completion_summary.json。
- 影响与边界：仅建立P4独立开发入口，无算法优化、调参、新飞行或提交/推送。P3最新真人手柄287帧外部保护、36次计算期限拒绝、1340.536ms峰值如实继承；显示内部体素/计时样本增长与整帧长计算未修复，不能宣称高速全图/严格实时/真人稳定通过。保存后P3停止改写，最终全量核验记录为准。
- 回滚/备份：完整P3保留，P4内保存来源原文；source_snapshot.json/raw_copy_summary.json为原样基线，before_adaptation/与adaptation.patch保留13项适配前版本；Git历史与未提交状态保留，无重设。
- 后续事项：性能/内存/外部保护问题仍待独立研究；本任务不自动追加飞行。HISTORY规则R-001未修改。

### 2026-10-16 18:24 — [EGO2P3] 保存当前版本，准备完整复制EGO2P4

- 状态：当前版本已保存；P4复制/独立构建待执行。
- 目的：用户要求保留P3全部内容并建立P4，说明现行算法与状态。
- 项目路径：/home/starry/isaac-data/EGO2P3。
- 涉及文件：新增performance/saved_current_20261016_1823，更新README.md、EGO2P3_VERSION.md、WORK_LOG.md及HISTORY总账。
- 修改内容：仅新增保存点/当前状态说明；源码算法、参数、地图和依赖不变。保存活动源tar/SHA、Git现状、6项执行产物、最新181424外部ROS日志/重放/截图和统计。
- 验证：6项产物复制SHA记录；最新手柄运行有287外部保护、36期限拒绝、核心峰值1340.536ms，未按零介入通过；显示历史内部无上限，尚未修复。保存不等于新提交或新飞行。
- 影响与边界：P3保存后停止改写；P4后续只适配自身运行路径/身份，不混淆继承成绩。
- 回滚/备份：当前完整P3保留，保存点见上述目录；无提交/推送。
- 后续事项：原样复制与全量核验，在P4生成当前说明并独立构建/回归。

### 2026-10-16 16:25 — [EGO2P3] 按用户辅助螺旋升降与未知探测偏好完成三项方案设计

- 状态：完成方案与文献核对；未实现、未构建、未飞行。
- 目的：按用户要求再查文献，确定执行/证据衔接、手柄辅助升降、有限风险探测与恢复三项方案；不增加垂直传感器，允许有边界的未知空间运动。
- 项目路径：/home/starry/isaac-data/EGO2P3。
- 涉及文件：新增performance/active_probe_design_20261016/FINAL_DESIGN.txt、references.json；WORK_LOG.md、HISTORY/CHANGELOG.md。
- 修改内容：固定两级运动授权CERTIFIED/EXPLORATORY，已知障碍/动力学等硬约束与未知暴露预算分开；风险动作不伪装完整认证，不把风险执行体积写入永久自由证明。辅助升降把人工高度意图转成可观察的侧移/圆弧/螺旋候选，人工接管/松杆保留；覆盖缺口转独立探测/恢复评估，预算按事件累积、不逐周期续期。执行一致性、来源证明、回退锚点与返回段风险共同处理；实现顺序一→三→二。
- 验证：通过网络核对CPA、SCOPE、FASTER、FaSTrack、RAPTOR、PA-MPPI、Pairet、RAS原文/作者版本。明确SCOPE/PA-MPPI不提供直接未知穿越的保证，Pairet的概率界需要概率模型，RAS粒子风险不等于未观测空间风险。第一版工程预算不是碰撞概率，数值待仿真定标。web首次Pairet的Cardiff PDF及两arXiv HTML/PDF返回错误，改读大学发布的正式PDF和arXiv PDF；未把未读取内容当已验证。仅写设计，没有执行构建、测试或新飞行。
- 影响与边界：用户接受更积极的未知探测已写入后续设计；当前生产仍是原严格策略，本次没有开启新模式或改动源码/参数/地图/安装产物。未知探测是项目工程扩展，其风险不能冒用论文形式保证。P2及旧成绩不变，未提交推送。
- 回滚/备份：只新增方案文档，无运行文件需回滚；既有诊断和历史资料完整保留。
- 后续事项：依定稿机制实现并标定数值，按单项/组合闭环测试验收；本轮只完成用户要求的查询与方案确定。

### 2026-10-16 15:46 — [EGO2P3] 分析空地卡停：当前位置完整机体证据不足与恢复入口锁死

- 状态：仅检查；原因分析完成，未实施修复。
- 目的：响应用户截图与“空地上卡住”的分析请求，区分几何碰撞、输入/深度过期、超时与认证拒绝，并核对合图关系。
- 项目路径：/home/starry/isaac-data/EGO2P3。
- 涉及文件：新增performance/stall_analysis_20261016/{ANALYSIS.txt,identity.json,replay_summary.json,run_statistics.json,exact_snapshot_states.json,geometry_audit.json,exact_geometry_audit.json,before_diagnostic_replay_comparison.json,analysis_scope_validation.json}；现场replays/ros_logs/截图/CSV/控制器参数备份及13帧重放输出；本WORK_LOG.md和HISTORY/CHANGELOG.md。源码、配置、地图、控制器/重放器产物均未修改。
- 修改内容：只新增诊断证据和结论记录。15:38:23轮782求解中446次UNOBSERVED_BODY_ENVELOPE，仿真9.083333至32.733333s存在；完整包络半径.58m、固定种子中心(-64,-11.2,2.8)/半径2.5m。停在(-64.10575465,-11.38421697,4.70862241)，种子包络超界约.406mm，真实最近障碍距中心3.941453m；顶部近场缺观测且历史不能补齐，当前位置核验在恢复前返回，连向下1m/s也拒绝。15:35:55轮142次同状态，但停点距障碍中心表面约.5918m、认证余量约.0118m，属于近障另一条件，不混称空地。
- 验证：只执行离线重放/哈希/CSV统计与体素几何诊断。13快照当前和诊断修改前二进制输出13/13相同（12包络拒绝、1接受），无加载失败；最新轮446包络拒绝均avoid<80ms，核心最大1.489062ms，两轮ESDF保护0，明确不属于持续超时或外部碰撞保护。当前地图SHA匹配manifest，输入、四路新鲜深度与已初始化种子均有证据。任务开始捕获的8项源码/地图/二进制身份在结束时无变化。本次未构建、未运行新仿真/真人飞行。
- 影响与边界：确认感知证据断层与当前位置包络前置门槛造成的锁死；未证明亚毫米越界仅由某一积分差异造成，连续执行轨迹尚缺。现有时延问题独立存在，但不是446次包络拒绝的直接原因。P2、其他项目及既有stall_recovery诊断改动均未触碰；不把配色/合图认作已证根因，不声称问题已修复。
- 回滚/备份：本次只增诊断档案，原始日志与快照未删除；删除新增分析目录即可移除产物，历史记录保留。生产文件无需回滚。
- 后续事项：待修复完整运动/制动证据与测得位姿衔接、垂直近场观测及有完整证明的已知空间重入；本次不自动改算法或启动飞行。

### 2026-10-16 14:14 — [EGO2P3] 完整三段地图统一为原地图，全段高度渐变

- 状态：完成；本次无导航或真人手柄飞行。
- 目的：按用户更正仅在EGO2P3工作，统一原图与扩图，以完整合并地图作为今后的原地图；EGO2P2保持原配置。
- 项目路径：/home/starry/isaac-data/EGO2P3。
- 涉及文件：scenes/ego_swarm_cloud/{ego_swarm_cloud_navigation.usd,occupancy.bin,manifest.json}；旧scenes/speed_cloud_long四文件归档移出活动树；scripts/run_isaac_fov_gvf_navigation.sh；profiles/manual_10mps_depth20.env；src/pc_gvf/src/verified_start.cpp；src/pc_gvf/test/cpp/observed_space_check.cpp；tools/{prepare_speed_maps.py,validate_unified_map.py,speed_benchmark.py,sim4_benchmark.py,test_speed_benchmark.py,validate_isaac_depth_contract.py,ros_observed_map_probe.py,analyze_navigation_benchmark.py,analyze_envelope_trace.py,summarize_speed_benchmark.py}；README.md、AGENTS.md、EGO2P3_VERSION.md、MANUAL_CONTROL_PROFILE.md、WORK_LOG.md；performance/unified_map_20261016证据；HISTORY的CHANGELOG.md、ENVIRONMENT_AND_ASSETS.md、FOV_GVF_FAMILY.md。逐项SHA与补丁见modified_files.json、changes.patch。
- 修改内容：将已有完整三段场景展平为自包含原地图，未重复拼接第一段。三段障碍共2,348,649顶点均使用51高度层的原扩图同款红→橙→黄→绿→青→蓝→紫渐变，白色地面保留。占据体素1200×300×50、.4m，原点(-80,-60,0)，起点(-64,-11.2,2.8)，新固定目标(384,11.2,2.8)，范围约480×120×20m。USD SHA cfcc22694241eb2ca5e506060c136a2f9fe5b3b5e2efd76cf0d44a1da26ed1ca；occupancy SHA e4e7e4301ae0ba6ae129bf813285ceda6d5f625f80a00b65284368682d9e68b8。Cloud各输入模式与保存profile使用同一活动地图；新speed/sim4仅original、完整地图超时600s。起步认证绑定完整USD/占据SHA，保留旧身份供归档回归；历史专用审计改读本轮before/scenes，防止旧小图成绩被新地图重新解释。6m/s默认和10m/s profile的各自速度/深度/H/C、完整名义/E认证参数保持。
- 验证：主Shell构建初轮3包3.43s，固定USD导出说明后最终3包1.85s；最终PYTHONNOUSERSITE=1 CTest40/40（12.97s），含新增统一图/短占据错配拒绝、USD篡改拒绝和三段占据点拒绝；Python216/216（.94s，原215+统一地图协议检查1）；源/安装profile、默认与goal参数构造6/6（Node.execute=0）；真实Shell场景选择6/6（手柄、键盘、轨迹、goal、保存profile、flat）；Shell/profile与tools语法、git diff --check通过。map_validation核对6网格全部拓扑/位置/变换、相机/图连接、完整占据与既有三段逐字节一致，全段色表/索引/材质一致且生成器重新生成USD/占据/manifest逐字节一致。Isaac离线渲染全图/近景两PNG成功并目视检查，无ROS、无控制器、无飞行。EGO2P2全部31,368文件SHA与保存复制快照一致，本任务前后size/mtime清单亦一致。
- 失败与修正：初轮CTest38/40，两项因未隔离用户site而载入NumPy2.2.6与系统SciPy冲突；PYTHONNOUSERSITE=1复测2/2通过，最终全40通过，初次失败日志保留。首轮生成复现因USD自动doc包含随机临时路径而SHA不同；生成器固定doc后最终逐字节复现通过，初始失败日志保留。没有修改依赖或缩减断言掩盖失败。
- 影响与边界：日后“原地图”专指统一完整三段图，旧原/扩区分仅用于历史材料；继承P2小图27.667s及其时延不属于新地图成绩。避障提名/走廊与运动认证数学没有改变，10m/s稳定到达与基本1ms目标仍未获通过证据；本次无新飞行。P2、P1及其他来源未改，无提交推送。
- 回滚/备份：本轮performance/unified_map_20261016/before保存修改前脚本/工具/配置/地图/当前说明与起步代码，旧两图相对依赖完整保留。恢复相应文件及speed_cloud_long目录后使用P3主构建脚本重建；HISTORY修改前相关文件亦备份在本轮目录。旧冻结实验与所有失败完整保留。
- 后续事项：无后台导航或自动续跑；统一图上的新飞行与真人手柄验收需另行执行。

### 2026-10-16 14:01 — [EGO2P3] 保存手柄配置并完成EGO2P2全量复制，独立验证通过

- 状态：完成；本次仅保存/复制/验证，没有新飞行。
- 目的：按用户要求保存当前手柄控制版本，完整复制EGO2P2到新EGO2P3，并在新目录说明算法、进展与状态。
- 项目路径：来源/home/starry/isaac-data/EGO2P2；新目录/home/starry/isaac-data/EGO2P3。
- 涉及文件：P2新增profiles/manual_10mps_depth20.env、MANUAL_CONTROL_PROFILE.md及源/六安装产物保存包和记录（此前保存条目）；P3完整目录，13项运行/身份适配（performance/copy_from_ego2p2_20261016/adapted_files.json）、README.md/AGENTS.md/WORK_LOG.md、EGO2P3_VERSION.md、EGO2P2_README.md/EGO2P2_AGENTS.md；复制/验证脚本、清单、构建/测试日志与completion_summary；HISTORY的CHANGELOG/README/FOV_GVF_FAMILY/PROJECT_INDEX。
- 修改内容：原样复制31,368文件/1,588目录，共32,956条目、4,557,752,738字节，含.git、隐藏缓存、全部历史、失败、地图与保存包。内容SHA、大小、模式、uid/gid、mtime一致，来源变化0、共享文件inode0；随后P3只改独立/tmp构建/安装/日志、机器人ID和新实验标签，保留P2比较入口及旧schema/历史身份。保存手柄profile仍原图10m/s/depth20/H.5/C.3、垂速1、a16/tau.10、world、E/floor0、goal几何复用核接人工意图；未加载profile的扩图6m/s/depth10/H1/C关默认保持。
- 验证：主入口独立构建3包33.5s通过；核心CTest40/40（13.08s）；平台/工具167/167=18+149（.94s）；真实ROS启动参数构造8/8（P3保存/默认/goal与P2保存，source/install，Node.execute0）；旧/Q两快照×普通/goal共4组输出一致。68个Python及活动Shell/profile语法通过。两核心静态库和两重放器SHA与当前P2完全一致；控制节点源归一化机器人ID后相同。最终P2全32,956条目保存后未变，P3所有继承条目保留，除16个已记录活动文件及普通测试缓存无其他改写。
- 失败与修正：首次Python合跑漏加载ROS环境，3项模块缺失导致收集失败，原日志保留；加载/opt/ros/humble与P3安装环境后167项通过，未改依赖。
- 影响与边界：新目录用于后续开发，P2作为保存来源保留；本次无新Isaac/真人手柄飞行。继承的原图goal27.667s与P95 2.513ms/max461.012ms仍属P2单次，10m/s双图/基本1ms联合目标未通过；两已回退算法没有恢复。当前源码和原地图算法/数值保持，不把复制回归写为性能提升。
- 回滚/备份：P2完整来源及saved_manual20_10_20261016源/产物快照；P3原样source_snapshot/raw_copy_summary、before_adaptation与补丁；来源README/AGENTS另存。旧Git元数据/未提交树保留，未提交推送，HISTORY规则R-001未改。
- 后续事项：无；后续使用P3保存profile和两个主入口，依用户新的明确指令开展工作，不自动续跑旧实验。

### 2026-10-16 13:54 — [EGO2P3] 完整复制检查点与独立路径适配

- 状态：原样复制核对通过；独立构建与验收进行中。
- 目的：用户要求保留当前手柄版本，将EGO2P2全部内容复制到EGO2P3并说明当前状态。
- 项目路径：/home/starry/isaac-data/EGO2P3（来源EGO2P2保留）。
- 涉及文件：整个原样副本31368文件/32956条目，随后13项运行/身份适配（adapted_files.json）；新增P3状态说明和复制验证证据；WORK_LOG及HISTORY。
- 修改内容：原样复制4557752738字节，含.git、隐藏缓存、所有历史/失败/保存包；内容/模式/uid/gid/mtime核对一致，source变化0、共享文件inode0。P3单独/tmp构建/安装/日志、机器人身份与新实验标签，保留P2比较入口；历史protocol版本、文档及performance内容不批量替换。手柄profile仍原图10/depth20/H.5/C.3，原入口默认仍扩图6。
- 验证：raw_copy_summary.json通过；P3构建/测试待执行，未新增仿真。
- 影响与边界：算法、数值参数、地图不变；P2已完成保存后不再改写。
- 回滚/备份：P2完整来源、P3原样source_snapshot与before_adaptation；P2_README/AGENTS保留来源文本，旧Git未提交树保留。
- 后续事项：构建独立P3、核对配置与核心重放，完成当前说明。

### 2026-10-16 13:53 — [EGO2P2] 保存当前10m/s/20m手柄版本，准备完整复制至EGO2P3

- 状态：手柄版本已保存；复制与P3验收另记P3日志。
- 目的：按用户要求保留当前手柄配置及EGO2P2完整来源。
- 项目路径：/home/starry/isaac-data/EGO2P2。
- 涉及文件：profiles/manual_10mps_depth20.env、MANUAL_CONTROL_PROFILE.md、README.md、EGO2P2_VERSION.md、WORK_LOG.md；performance/saved_manual20_10_20261016/checkpoint.json、source_checkpoint.tar.gz、installed_products.tar.gz；HISTORY/CHANGELOG.md。
- 修改内容：将上一轮提供的原图10m/s/depth20/H.5/C.3手柄配置持久保存；a16/tau.10、垂速1、E/floor0、goal几何复用核和显式人工候选缓存保持。算法源码及原扩图6m/s默认均未改。保存当前源与6个安装产物的SHA及档案，旧Git未提交树与全部历史保留。
- 验证：保存前无活动仿真控制器；上一轮构建3包、缓存9项、配置构造2/2证据保留，本次未新增飞行。保存配置将随P3实际参数构造核对，不把保存写作稳定性通过。
- 影响与边界：仅新增可加载配置、保存包与说明；不将原图goal27.667s/求解P95 2.513ms当作手柄成绩，10m/s双图/基本1ms目标未通过。
- 回滚/备份：原手柄6m/s入口保留；完整当前源/安装产物已归档。复制完成后P2作为来源保留不再改写。
- 后续事项：完整复制含隐藏文件/历史/失败至EGO2P3，适配P3独立路径并验证。

### 2026-10-16 13:32 — [EGO2P2] 20m/10m/s手柄指令准备完成，按用户最新要求未启动仿真

- 状态：完成指令准备与验证；未运行手柄飞行。
- 目的：将选中的原图27.667s参数交给用户手动启动。
- 项目路径：/home/starry/isaac-data/EGO2P2。
- 涉及文件：src/pc_gvf/src/depth_angular_controller_node.cpp、performance/manual20_10_20261016（before源码、environment、build、cache_check、launch_validation）；WORK_LOG.md、README.md、EGO2P2_VERSION.md、HISTORY/CHANGELOG.md。
- 修改内容：显式paper_goal_proposal_cache允许手柄使用，缺省仍关闭；人工缓存范围不依赖fixed_goal。指令用原图、goal几何复用核与人工意图，10m/s、depth20/H.5/C.3、a16/tau.10、垂速1、E开、floor0。默认手柄6m/s未变。
- 验证：主构建3包通过；候选缓存9项检查通过（松杆、改向、反向、过期、位移、速度变化、时间回退与不续期等）；真实ROS source/install参数构造2/2一致，Node.execute=0，controller/bridge速度10/10/1。没有重跑未修改核心测试，没有启动ROS控制器、Isaac或真人飞行。
- 影响与边界：固定目标改为人工意图，不能把原图goal成绩或时延直接当手柄成绩。历史缓存只提名，运动和E仍完整重认证。用户后续要求只给指令，故未继续自动启动。
- 回滚/备份：performance/manual20_10_20261016/depth_angular_controller_node.before.cpp；恢复后主构建。
- 后续事项：用户按提供命令启动试用。

### 2026-10-16 13:31 — [EGO2P2] 选中20m/10m/s候选接入手柄，启动准备

- 状态：进行中。
- 目的：用户要求将27.667s原图单次版本用于手柄试用。
- 项目路径：/home/starry/isaac-data/EGO2P2。
- 涉及文件：src/pc_gvf/src/depth_angular_controller_node.cpp、performance/manual20_10_20261016、WORK_LOG.md、HISTORY/CHANGELOG.md。
- 修改内容：paper_goal_proposal_cache显式开启时允许人工输入使用（缺省仍关），人工缓存范围不依赖fixed_goal；增加实际缓存启用日志。本次拟用goal几何复用核、人工意图、原图、10m/s、depth20/H.5/C.3、a16/tau.10、垂速1、floor0、E开；不修改日常6m/s默认。
- 验证：已读项目/统一规则，北通BFM js0可读；无其他仿真控制器。代码尚待构建、缓存改向/松杆检查、启动配置核对；未启动飞行。
- 影响与边界：固定目标变成人工意图后候选选择路径不同，不承诺复现27.667s或1ms；完整名义/紧急制动认证仍逐命令执行。
- 回滚/备份：performance/manual20_10_20261016/depth_angular_controller_node.before.cpp。来源与场景保留。
- 后续事项：构建和验证后直接启动可见手柄仿真，按用户授权无需再次确认。

### 2026-10-16 13:22 — [EGO2P2] 完成10m/s/1ms有限试验，未达标，恢复原认证核心

- 状态：尝试、归档与判断完成；10m/s双图到达及基本1ms联合目标未通过。
- 目的：按用户要求先参数后算法，以实测判断可行性。
- 项目路径：`/home/starry/isaac-data/EGO2P2`。
- 涉及文件：生产保留控制器逐次CSV、两个launch分辨率/int覆盖；README.md、AGENTS.md、EGO2P2_VERSION.md、LATENCY10_TUNING.md、WORK_LOG.md；performance/latency10_20261016（备份、分析、完整候选源码/产物/负例、验证/结束摘要）及performance/p2_speed_20261016/latency10_*；HISTORY的CHANGELOG/README/FOV_GVF_FAMILY/PROJECT_INDEX。
- 修改内容：先试32×24/12×10、history8/H.35/C.4，再depth20/H.5/C.3；固定a16/tau.10、机体.48/认证.58、floor0、E开。实现块最小深度/同次制动证明复用及整段胶囊凸覆盖两版。未证明闭环收益，paper_guidance.cpp/头/CMake恢复任务前字节，新增测试移动至归档；goal重放二进制与算法前参考一致。手柄6m/s/depth10默认保持，10m/s候选仅显式研究命令。
- 验证：10实际飞行（5配置×双图）：1安全达速到达、9探索卡停，另1沙箱ROS套接字拒绝启动中断。最佳20m参数原图27.667s、峰9.999903m/s、>=9.5累计5.9s；求解P95 2.513/P99 19.363/max461.012ms、88.19%<=1ms，同参数扩图失败。两算法均两图失败，算法1扩图最坏10195.526ms。10条扫掠/保护/限值违规全0、冻结执行身份全通过；跨版总数不作成功率。没有正式重复稳定组，也没有真人手柄飞行。
- 验证补充：算法1/2各最终核心42/42；差分重放21/26快照×3次均输出一致，进程+载入中位数合计约2.48%/2.10%改善，不是在线求解收益。首轮重放与CTest并发计时单列保留；第二版首次测试与增量构建重叠，最终42项独立重验通过。恢复后主构建3包、CTest40/40、工具149/149、source/install实际ROS参数构造8/8通过，Node.execute=0；未额外飞行。首轮来源核对仅预期3源差异失败，检查发现外部于11:18:17更正sim4 --out帮助文字P1→P2，纯文字且保留；最终核对通过。精确执行时间以日志epoch/manifest为准。
- 影响与边界：只研究P2；来源目录、地图、机体/认证半径、看门狗与HISTORY规则未改；两失败算法不设默认。快慢规划拆分、增量证据索引与可重启规划为后续判断，尚未实施。未提交推送。
- 回滚/备份：before_source.tar.gz/before_sha256.json；algorithm1/source.tar.gz与重放器；algorithm2/complete_source.tar.gz、四执行产物、补丁及负例；每轮完整frozen和audit均保留。源码3核心与goal重放参考相同，两个launch源/安装一致，导航锁可用。
- 后续事项：本轮无后台飞行、自动续跑或待执行计划；新任务再依据用户指令开展结构性优化。

### 2026-10-16 11:36 — EGO2P2：第一版缓存结果与整段凸覆盖尝试

- 第一版：核心42/42通过；21快照×3次×双版本输出一致，独立无CTest干扰轮的进程+载入中位数合计799.888→780.061ms（约2.48%，不是在线求解降幅）；同时运行CTest的首轮另存，不混入。
- 闭环：同depth20/H.5/C.3参数两图都卡停。原29.817s/P95 44.686ms/max735.168ms，扩49.917s/走211.078m/P95 18.227ms/max10195.526ms。全部失败保留，不宣称局部缓存改善导航稳定性。
- 第二版：新增完整胶囊在单个原始相机内的凸截锥充分证明，覆盖两端球投影联合矩形内全部原像素，并要求最小深度严格大于整段光学z上界；否则回到原有认证。求解内用块最小值，前端用原像素；未知孔洞不允许端点桥接。
- 新负例：已知两端球、中间未知或NaN必须拒绝整段；相机背面及深度截断外拒绝；原缓存失效测试保留。第二版两次主构建3包通过；第一次42项测试与最后增量构建时间重叠，最终产物42项独立复验正在执行，后续再跑双图。
- 记录时间说明：本任务先前11:24、11:35条目是人工阶段标签；运行精确时间以各日志epoch和manifest为准，尤其11:35条目实际写在第二组参数结束附近。

### 2026-10-16 11:35 — EGO2P2：参数阶段结束，进入几何证明加速尝试

- 状态：尚未达标；优化实现构建中。
- 参数试验：降到32×24/12×10、history8、H.35、C.4双图卡停，原P95约41.44ms、扩126.18ms；恢复64×48/24×18并depth20/H.5/C.3，原图安全到达27.667s、P95=2.513ms/max461.012ms，扩图24.933s探索卡停、P95=96.686ms/max914.683ms。保持a16/tau.10。两组均未证明稳定1ms，原图单次通过不是稳定结论。
- 修改：paper_guidance.hpp/cpp增加只在当前求解内有效的支持观测8×8块最小深度界，严格满足原像素跳过条件才快通；否则原未知锥精确计算不变。单次速度搜索内复用完全同一证据/状态/时刻的正向独立紧急制动证明，重新step/ingest/reset失效，不跨帧缓存安全许可。新增certificate_cache_check普通/goal负例，覆盖同指针/同时间/同版本下未知与NaN修改不得复用旧证明。
- 验证：计时/参数版主构建3包、CTest40/40通过；算法版尚待构建/差分重放/真实仿真，不宣称成功。
- 证据：performance/latency10_20261016及performance/p2_speed_20261016全部原始数据；instrumented_reference保留算法修改前重放器与静态库。
- 后续：先检查输出等价与缓存失效，再测试同参数闭环。


### 2026-10-16 11:24 — EGO2P2：10m/s与1ms求解目标，基线与参数阶段

- 状态：进行中，目标尚未通过。
- 目的：先调参数，失败后再尝试算法优化；不将设置速度或静止拒绝视作达标。
- 修改：增加结束时写出的逐调用CSV（候选、快照、核心、总求解、速度及状态），角域/虚拟chart分辨率提供环境变量入口；缺省64×48/24×18保持。
- 验证：初次沙箱ROS通信被拒后中断；主机双图10m/s/a16/tau.10探索均卡停，原/扩求解P95=41.470/30.603ms、max=327.273/1182.771ms；扫掠/外部保护0，非到达验收。此处新源码尚未构建。
- 备份/证据：performance/latency10_20261016/before_source.tar.gz及before_sha256.json；原始试次performance/p2_speed_20261016/latency10_baseline_host；全部失败保留，未改来源项目。
- 后续：构建计时与参数入口、先作参数对照。
### 2026-10-15 19:39 — [EGO2P1] 分析上次扩图手柄求解长尾并绘制耗时占比

- 状态：完成离线分析；缺少逐次原始计时，不能还原任意分段的精确直方图或最慢单次内部阶段归因。
- 目的：回应用户对“P50约0.55ms、P95约5.24ms差别大”的问题，画主要时长分布和时间占比。
- 项目路径：`/home/starry/isaac-data/EGO2P1`。
- 涉及文件：新增`performance/latency_analysis_20261015_192842/plot_latency.py`、`source_performance_excerpt.txt`、`source_controller.log`、`analysis.json`、`analysis_notes.txt`、`latency_distribution.png`、`latency_distribution.pdf`、`plot.log`、`plot_api_attempt.log`、`validation.json`；WORK_LOG和HISTORY/CHANGELOG。
- 修改内容：只读取19:28:42手柄run的既有汇总和控制器输出，8711次avoid与11243次发布回调。近似次数分位段50%/45%/5%，明确不是逐次直方图；按均值乘各自样本数估算回调内累计35.876s中avoid17.823s(49.678%)、DDS发布.989s(2.758%)、未细分差额17.064s(47.564%)。不同计数口径不直接相除。图表不是全进程CPU占比，也未将证据各view均值拼成虚假精细比例。
- 验证：图表数字与原日志匹配、分项合计和百分比100%检查通过；PNG三面板目视无裁切重叠。源码确认候选复用/快速拒绝/复杂几何证明工作量不同，80ms期限在求解返回后检查，无法限制底层最长运算到80ms；未断言某具体函数就是894.766ms根因。首次绘图NumPy2/系统matplotlib ABI失败，隔离用户包后旧matplotlib布局API失败，改兼容布局后PNG/PDF成功；未改依赖或生产代码，未新建导航/测试飞行。
- 影响与边界：保留当前扩图6m/s默认；只新增分析和记录。原始统计无逐次输出，细粒度分档/精确尖峰归因需未来独立测量，未启动。
- 回滚/备份：原日志未改；本目录含本次分析使用的原run汇总副本、控制器日志副本及SHA来源。原源码和安装产物未动。
- 后续事项：无；本次请求的图表已交付。

### 2026-10-15 19:27 — [EGO2P1] 手柄默认改为三段扩图6m/s

- 状态：完成。
- 目的：落实用户“手柄控制的版本改为在扩图上以6m/s的速度运行”。
- 项目路径：`/home/starry/isaac-data/EGO2P1`。
- 涉及文件：`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/isaac/beitong_joystick.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`；README、EGO2P1_VERSION、AGENTS、P1_SPEED_TUNING、WORK_LOG；`performance/joystick_long6_20261015/`中的before备份、check_manual6_launch.py、构建/测试/参数构造日志、default_checks.json、manual6_launch_validation.json、completion_summary.json；HISTORY的README/FOV_GVF_FAMILY/PROJECT_INDEX/CHANGELOG。
- 修改内容：手柄输入与人工launch的JOYSTICK_HORIZONTAL_SPEED缺省4→6m/s，控制器speed/max_speed和桥接max_vx/max_vy同源同步；主入口cloud场景在joystick模式下缺省改为`scenes/speed_cloud_long/speed_cloud_long.usda`，其余输入模式保留原Cloud缺省，显式场景/flat/速度覆盖有效。扩图对应occupancy自动选取，a16/τ.10、控制器与桥接垂速上限1保持。
- 验证：`bash scripts/build_isaac_ros_workspace.sh`3包成功；主入口bash语法通过；既有`test_beitong_joystick.py`5/5通过；真实场景选择片段5种输入/覆盖组合通过，模拟手柄默认满推输出(6,0,0)；source与install实际ROS launch构造2/2通过，controller/bridge速度6与扩图seed/occupancy一致，Node.execute=0。未启动Isaac或真人手柄飞行，未重跑旧导航实验或未变核心测试。
- 影响与边界：仅当前手柄默认和相应说明更新；旧性能数据、完成摘要、冻结包保留原身份。现行源码已经不同于19:17完成时的冻结，不能沿用“当前源完全等于旧冻结”结论。没有新增可靠性结论。
- 回滚/备份：`performance/joystick_long6_20261015/before/`保存3源码及4说明原文件；恢复源码后用主构建入口重新安装。没有提交或推送，HISTORY规则R-001未改。
- 后续事项：无；用户可按README启动手柄。

### 2026-10-15 19:17 — [EGO2P1] 交付8档失败轨迹视频、最终报告与手柄命令，结束本轮任务

- 状态：本轮有限实验、归档和交付完成；用户要求结束，不再自动追加导航。稳定双图4m/s未通过、数学最快上限未证明，区别于交付完成。
- 目的：响应“每种速度没到达的各录像一种”，保留原失败以便用户判断，完成前述总结/保留版本/手柄代码要求。
- 项目路径：`/home/starry/isaac-data/EGO2P1`。
- 涉及文件：`P1_SPEED_TUNING.md`、`README.md`、`EGO2P1_VERSION.md`、`AGENTS.md`、`WORK_LOG.md`；`performance/p1_speed_20261015/progress.json`、`completion_summary.json`、`final_review/final_documentation_checks.json`、`final_review/final_provenance_v1/post_video_source_check.json`及log；`video_evidence/`中的README、select_failure_videos.py、selection.json、replay_failure_video.py、render_selected_videos.py、render_driver_state.json、8段failure_v*mps.mp4及各自frames.csv/replay.json/截图/解码与渲染log、all_failed_speeds.mp4及ffconcat/ffmetadata、all_final_frames_contact_sheet.png、video_manifest.json、source_selection_check.json、root_visual_qa.json；HISTORY的README/FOV_GVF_FAMILY/PROJECT_INDEX/CHANGELOG。
- 修改内容：2/3/4/5/6/8/10/12m/s每档1条真实未到达CSV原位置在原USD场景中的离线重建，1倍仿真时间、960×540、15fps，含2s预热和原末尾约15s停滞。2/4/8用最终版、3/5/6/10/12用明确标注的历史版；6最终双图成功，故失败片选历史。CSV无yaw，按原goal规则重建，并与稀疏原日志核对最大误差0.05°（打印精度）；没有把意图qx/qy/qz当姿态。没有ROS/控制器/物理推进，观察相机仅调整视角，USD仅SessionLayer，不保存地图。
- 验证：8段独立片及426.933s合集均完整解码返回0，共6404帧；逐帧记录的位置/状态/时间/原CSV行全部匹配，含精确首末行。各段scene前后SHA一致，末帧5/5视线通；root目视8张最终帧联系图及3/12独立编码末帧通过。3速途中4帧约.27s局部遮挡、主体可辨，末段清楚，限制原样保留。source/install/controller共423文件内容/大小/模式/集合以及4地图文件复核零差异；精简6m/s文档命令干净环境关键参数一致，纯AST不构造节点；89个本地链接存在、6段bash语法通过，均未执行飞行命令。已有核心40/40、生产Python153/153、分组合成142/142、新手柄6参数构造2/2的证据保持，不重复未改测试；本次视频不是新导航成绩。
- 技术失败保留：首个默认沙箱GPU不可用未产帧、首个可渲染smoke最终视角遮挡、视线工具初版缺省occupancyOrigin失败，均留原日志；修正为原脚本同一默认原点、无遮挡观察相机和场景变换刷新，最终smoke目视通过后才批量。无自动审批拒绝，不混入76启动/74飞行总账。
- 结论与边界：当前安全修复源码、a16/τ.10/native90°/.22/floor0保留；显式6m/s为最终版本最高双图单次成功候选26.950/93.650s，非稳定证明；源码手柄默认4。历史8最快冻结包独立保留。已给可复制手柄代码、完整实验表、参数重要性排序与未实施的算法建议，没有真人手柄飞行。最终4原1/3、扩0/3和秒级计算峰值未掩盖。
- 回滚/备份：全部旧试次与失败留存；当前源码仍等于最终冻结身份，历史最快`performance/preserved_versions/historical_fastest_single_moderate_v8_a16.tar.gz`保留，未覆盖当前代码。全部视频可由selection+原CSV/场景重建。未提交推送，HISTORY规则R-001未改。
- 后续事项：无。本轮导航和渲染均已退出，共用导航锁可用，无自动续跑；建议仅供未来新的明确任务。

### 2026-10-15 19:04 — [EGO2P1] 完成有限速度扫描，保留6m/s单次性能候选与历史最快快照

- 状态：导航实验完成；按用户要求不再扩展参数试验。稳定双图4m/s目标未通过，新增失败视频正在制作，不能写整个可靠性目标已完成。
- 目的：结束本轮优化、保留当前最好已测性能配置，交付手柄命令、实验判断、参数排序与大胆但未实施的算法建议；用户又要求每种未到达速度各一段视频。
- 项目路径：`/home/starry/isaac-data/EGO2P1`。
- 涉及生产文件（相对任务前167源文件，共15修改+4新增、零删除）：`scripts/isaac/run_fov_gvf_navigation.py`；`scripts/run_isaac_fov_gvf_navigation.sh`；`src/pc_gvf/CMakeLists.txt`；`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`；`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`；`src/pc_gvf/launch/navigation_benchmark.launch.py`；`src/pc_gvf/src/depth_angular_controller_node.cpp`；`src/pc_gvf/src/paper_guidance.cpp`；`src/pc_gvf/src/paper_replay.cpp`；`src/pc_gvf/test/cpp/continuous_recovery_check.cpp`；`src/pc_gvf/test/cpp/incremental_field_check.cpp`；`src/pc_gvf/test/cpp/intent_response_check.cpp`；`src/pc_gvf/test/cpp/observed_space_check.cpp`；`tools/sim4_benchmark.py`；`tools/test_sim4_benchmark.py`；`scripts/isaac/depth_camera_projection.py`；`scripts/isaac/read_camera_projection.py`；`src/pc_gvf/test/cpp/free_space_floor_check.cpp`；`src/pc_gvf/test/test_navigation_ground_guard.py`。完整前后SHA见`performance/p1_speed_20261015/final_review/intentional_source_diff.json`。
- 涉及记录：README、AGENTS、EGO2P1_VERSION、P1_SPEED_TUNING、WORK_LOG；HISTORY的README/FOV_GVF_FAMILY/PROJECT_INDEX/CHANGELOG；本轮summarize.py、check_grouping.py、build_final_report.py、final_scan_driver.py、ranking_and_recommendations.md、结果CSV/JSON/图、final_review、video_evidence及保留归档。全部旧条目和失败不覆盖。
- 实验总账：76次启动，74条结束实际飞行，22安全达速、48探索卡停、2到达后独立审计拒绝、2完整超时，另2无轨迹启动失败；P0继承数据不混入。上述跨版本总数不是单一配置成功率。实际测试速度2/3/4/5/6/8/10/12m/s。
- 最后固定版12轮：native90°/.22m、a16/τ.10、垂速1、floor启用z0、H1、E开、delay0、restart0；2速双图0/1，4速原1/3(37.5167s)/扩0/3，6速原26.950s/扩93.650s各1/1，8速双图0/1。全部带15s卡停阈值，称重复筛查，不是完整240/600s正式重复；正式至少3次全过组仍0。
- 保留判断：当前安全修复源码保留，**6m/s为当前最高双图单次成功性能profile**，未证明重复稳定。源手柄默认仍4，交付显式JOYSTICK_HORIZONTAL_SPEED=6命令；普通手柄核、C关，与goal核、C.15不同，不冒称真人改向已验证。历史8m/s双图33.1667/77.45s及原图另一垂速4的19.9333s全部保留，不能混成当前版或同一配置最短。
- 安全/性能：最后6速两图净距.188635/.135473m，扫掠/保护/限值违规全0；平均实际航速5.089/5.001m/s；callback均2.596/3.248ms、P95为8.214/7.877ms、峰值98.903/1170.440ms，严格60Hz实时未通过。本轮旧阶段最坏callback仍有秒级峰值，失败计入报告。
- 验证：最新生产工具153/153=sim4 142+speed7+ground4；完整核心40/40及最终floor fixture普通/goal2/2、主构建3包均有原日志。最终未重复运行未改的153/40。新手柄6实际source/install构造2/2、Node.execute=0。source/install floor56/56、camera46/46、actualUSD18与8fixture交叉核查、真实ROS120/140原图探针成功；严格汇总分组142/142。
- 最终身份：12轮执行源/产物完全一致；当前live内容/大小/模式/集合与末轮冻结一致，两launch源/安装一致，两地图SHA一致；1895路径哈希零差异，见final_provenance_v1/final_provenance.json。没有修改P0或场景文件，没有提交推送。
- 归档/恢复：历史moderate_v8的420/420 blob SHA核验通过，打包`performance/preserved_versions/historical_fastest_single_moderate_v8_a16.tar.gz`（17,626,870B，SHA7db7f8435a22…）；441普通成员逐项核对通过。仅独立归档，未恢复旧源码。任务前before_source.tar.gz、documentation_before、各轮冻结与二进制及所有失败保留。
- 视频：选2/3/4/5/6/8/10/12各一条真实未到达；6失败用历史capsule_v6，因最终6双图成功。CSV无yaw，位置按原记录、yaw按冻结goal规则重建，必须标非原始录像/非新导航。首默认沙箱GPU不可用失败日志保留；本机GPU样片成功、scene SHA未改，但末段观察视角遮挡已发现，正在修正观察相机并重验，不能提前写8视频已完成。
- 后续事项：只完成新增视频、最后文档和退出核验；参数/算法建议均未自动实施，不再启动导航研究。

### 2026-10-15 18:26 — [EGO2P1] 相机外参调节落地与地面高度扫描入口

- 状态：继续优化；midcam42_v4正在运行，尚无正式重复全通过结论。
- 目的：降低近机体感知盲区与卡停，继续用户授权的仿真4m/s、多速度与性能研究。
- 项目路径：`/home/starry/isaac-data/EGO2P1`。
- 涉及文件：`scripts/isaac/depth_camera_projection.py`、`read_camera_projection.py`、`run_fov_gvf_navigation.py`、主运行Shell、两个launch、`tools/sim4_benchmark.py`及测试；`performance/p1_speed_20261015`与`performance/fov_candidate/camera_offset_v1`、本日志及HISTORY。
- 修改：可选ISAAC_DEPTH_CAMERA_OFFSET_M默认原生.22m，仿真只在USD SessionLayer调整四相机安装位置，同步控制器/点云camera_offset。v6协议与传感器v2记录冻结实际位置、旋转、局部矩阵及内参；严禁近裁切屏蔽障碍。新增--floor-z默认0、有限值验证，用于收紧规划最低高度，独立审计始终使用原plane0。
- 验证：camera_offset主构建3包成功.79s；source/install实际启动参数构造23+23=46/46、Node.execute0；actualUSD双图18项及8fixture工具交叉检查通过。此前完整Python144/144=sim4 133+speed7+ground4；floor CLI新增9项后完整153/153=142+7+4，原失败/成功日志均保留。最近完整核心CTest40/40及fixture修正后2/2。外参.6的120°和140°原图独立四CameraInfo/Image metadata探针均ok；每轮实际USD与冻结身份另审计，不将其写成每轮ROS探针。
- 新实验结果（原图/扩图，均单次探索）：120°/.6/a16/.10为安全36.483s/卡停44.567s；140°/.6/a16/.10为安全42.083s/卡停21.133s；120°/.6/a256/.025为卡停36.450s/安全127.217s；后一组H.4双图卡停29.567/24.233s。原生90°/.22/a16/.10、垂速.1为安全142.867s/卡停167.233s，低垂速明显拖慢且未解决长图。
- 影响与边界：起点自遮挡改善不代表连续稳定；.6相机可能使静止认证球过度依赖历史证据。所有失败保留。新--floor-z仅规划参数，负值虽有限值可解析，本轮拟试非负值；独立审计边界、机体.48、基础认证.58、80/100ms期限、P0与场景文件均不改。没有提交推送。
- 回滚/备份：各应用补丁、manifest、before_source.tar.gz、逐轮冻结与原始日志保留。floor CLI准备稿及生产153项证据位于floor_cli_candidate_v1；准备阶段仅AST通过，应用后才运行pytest。
- 后续事项：完成中间相机位置.42和较高规划下界候选，再对可用配置进行重复/速度测试，最终同步全部状态入口与报告。

### 2026-10-15 17:57 — [EGO2P1] 地面约束回归完成、宽视场失败与自身遮挡定位

- 状态：继续优化；当前没有活动飞行。原始失败全部保留，仍无正式三次全通过组。
- 目的/项目路径：继续用户授权的仿真4m/s与速度上限研究；`/home/starry/isaac-data/EGO2P1`。
- 涉及文件：8项floor核心/节点/重放/测试/CMake（清单final_review/floor_core_files.json）；两个launch；sim4工具/测试；4个runtime相机脚本（清单fov_runtime_application）；README、AGENTS、EGO2P1_VERSION、性能汇总与本日志/HISTORY。
- 验证：floor主构建3包29.1s成功；CTest40/40、12.70s。最后刹车尾fixture余量.12→.14在首构建之后，增量主构建1.42s后确认测试产物mtime较源新，普通/goal两floor目标2/2再通过。source/install floor参数56/56，Node.execute0。floor阶段三Python文件67/67（speed7+sim4 56+ground4）。
- 飞行：floor_confirm_v4原图完整240s TIMEOUT，停在(-8.370,-4.994,1.138)，机体最小净空+.165855m、零扫掠/保护；其余正式重复未执行。新增可选ISAAC_DEPTH_HFOV_DEG，仅SessionLayer改投影、默认不设置仍原90，无地图保存。wide140_v4、wide120_v4各双图均原地15s探索卡停、均未达速/未到达；不能算成功。
- 相机证据：v5逐轮冻结实际四USD投影与请求，sensor profile参与严格组。生产完整speed+sim4+ground为104/104（7+93+4），.55s；此前103通过/1暂缓汇总集成测试的日志保留，集成修复仅performance文件。分组独立116/116。runtime17轻量检查通过；旧14项actualUSD fixture对应相同helper，但本应用阶段未重跑该旧地图fixture。
- 真实ROS与失败：140原图probe因未带主运行ROS_LOCALHOST_ONLY设置而未收数据，140扩图补设置后启动偏晚也未收数据，均保留为失败。120原图在启动前监听、使用域42/localonly1/fastrtps，四CameraInfo+Image metadata检查ok，实际HFOV120/VFOV104.821822与USD一致；这是一次现场读数，不冒称每轮ROS读数。
- 原因：140首Q快照四路均3072有效深度，最近z深度.0854m，重建近点距机体中心.327–.340m，在机体.48球内；四视图内参也匹配实际140。宽视角看到自身几何，fresh-hit撤销起点seed，导致UNOBSERVED_BODY_ENVELOPE；不是ground0或无深度。120同症状，继续核对。
- 后续修改（进行中未验证）：增加可选虚拟相机径向安装距离ISAAC_DEPTH_CAMERA_OFFSET_M（默认.22，候选.60），同步四mount实际外参与核心camera_offset；新v6记录/审核外参，保持旧v5审计。不得用近裁切、忽略外部障碍或放宽机体/地面标准掩盖自身遮挡。此候选尚未构建或飞行。
- 来源/恢复：P0、地图及旧试验不写入；source前备份、floor精确补丁与/tmp/ego2p1_floor_prechange_20261015保留。README/版本/AGENTS更新当前进行中状态，旧文本在documentation_before。未提交或推送。

### 2026-10-15 17:34 — [EGO2P1] 正式4m/s暴露保守地面越界，新增规划半空间约束；计数核对补正

- 状态：修复/回归进行中，没有活动飞行；此前单次成功不代表正式重复通过。
- 目的/项目路径：遵守既有z=0审计边界，继续用户授权的EGO2P1优化，路径`/home/starry/isaac-data/EGO2P1`。
- 失败证据：confirm_moderate_v4原图正式r1在51.3s ARRIVED，但547段plane0扫掠重叠、最小-.003637190m；实际最低中心z=.477134648，机体球对z0也真实越界约.002865m。体素障碍与侧界重叠均0，体素最小净空+.167623m。独立复核见final_review/confirm_moderate_v4_r01_ground_probe.json；余项按stop-on-failure未执行。
- 修改范围：paper_guidance头/源、节点参数、replay和回归；两个launch、sim4工具/测试。新增free_space_floor_enabled与free_space_floor_z，通用核心默认禁用，Cloud及新sim4协议显式启用z=0。所有球/段证书先与保守半空间求交，再做原观测证明；未知空间仍拒绝。旧协议缺字段按禁用读取；新Q重放保存开关/高度并由旧二进制明确拒绝，旧3..P保持可读。
- 验证：launch语法通过，工具floor新增18项正负例，sim4+ground当前60/60、.36s。核心构建与新飞行尚未执行，不作通过声明。
- **17:27计数更正条的补正**：42/45与38/42来自不同测试文件组合，不是历史数字错误。原audit_tests_v1=sim4当时35+speed工具7，共42；delay及final default更新前组合为38+7，共45；audit_tests_final_default单独sim4=38，moderate_default=38+ground4共42。以日志及测试文件清单为准；之前笼统称16:27/16:55“误计”不准确，本条纠正并保留历史。
- 影响/备份：保持.48机体、.58基础认证、原地图、USD地板及原审计plane0，新增约束不依赖外部guard接管。各旧失败与冻结源保持；文档编辑前版本另存performance/p1_speed_20261015/documentation_before。
- 后续：完成构建/回归和真实启动参数核验，然后重新做4m/s正式重复与多速度测试，最终更新README、版本、统一状态。

### 2026-10-15 17:27 — [EGO2P1] 中等响应多速度筛查、真实地面保护与测试计数更正

- 状态：继续正式重复验收；a16/τ.10的4、6、8m/s在双图各单次安全到达，尚未获得同版双图各3次全通过。
- 目的/路径：继续用户授权的纯仿真优化；`/home/starry/isaac-data/EGO2P1`，P0与地图保持不变。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、两个launch、`tools/sim4_benchmark.py`、`src/pc_gvf/test/test_navigation_ground_guard.py`及CMake；`performance/p1_speed_20261015/`证据、progress.json、本日志与HISTORY。
- 修改：ESDF模式也读取USD实际Ground，并先独立检查真实地板，再检查ESDF。默认执行/认证加减速度64→16m/s²、τ.03→.10s，pipeline delay0保持，速度默认4、垂速1；旧协议显式覆盖保留。机体.48、认证.58、地图及审计z=0平面不改。
- 实验：旧a64/τ.03正式confirm_v4原图38.45s通过、扩图600s完整超时，余项因stop-on-failure未执行。dense_wide_reserve10_v6双图卡停。a16/τ.10标准参数探索4m/s=35.7167/120.0667s、6=26.95/90.05s、8=33.1667/77.45s（原/扩），均通过独立安全与达速。8m/s垂速4原图19.9333s通过、扩图卡停；10和12m/s双图卡停，均不计成功。当前confirm_moderate_v4以240/600s完整协议、各3次、遇失败停止，进行中。
- 验证：主入口build_moderate_default_v5成功；最近完整CTest为38/38，12.75s。source/install人工默认/目标默认/旧显式覆盖6/6；Node.execute=0。显式Python两文件工具+ground guard最新42/42，0.30s，见python_final_explicit_suite.log。
- **计数更正**：前面16:27/16:55条目把工具通过数记为42/45，系摘要误计；原日志audit_tests_v1.log、audit_tests_delay.log、audit_tests_final_default.log是权威依据，后者为38/38；python_tests_moderate_default_v5.log合计42/42（38工具+4地面），不是49。保留旧条和全部原日志，本条更正不影响任何飞行独立审计。
- 边界/备份：a16降低离散转弯偏差但不消除异步及积分差异；回调仍有秒级峰值，不宣称严格60Hz。扩大FOV和执行器模型证书只存候选归档，未应用/未飞行。每轮源/安装/参数冻结与原始失败保留，before_source.tar.gz支持恢复；未提交或推送。
- 后续：完成正式重复、高速边界与最终资料更新，按原始审计报告最高已测安全速度及最短时间，不能把单次、不同参数或不同版本拼成稳定成绩。

### 2026-10-15 16:55 — [EGO2P1] 静态来源门控、速度扩展与正式4m/s失败暴露

- 状态：继续优化；尚未达同版双图各3次稳定通过。confirm_v4原图r1正式通过38.45s，扩图r1在x369.538处卡停，正按原600s协议等待终止；未执行项不会当成完成。
- 源码/参数：新增有界递归静态来源门控，阻止嵌套/时变观测经旧pending球、测量球转成永久胶囊；不增加成员/重放格式，旧不带来源的重放证据无法追溯。主构建成功，CTest37/37、12.51s。仿真主launch及新审计工具默认pipeline delay由.2改0，匹配即时执行候选；旧协议显式.2仍保留。最新工具45/45；source/install的manual0、goal0、旧goal.2实际构造6/6，Node.execute=0。
- 参数验证：clearance reward、history间隔/容量的source/install×manual/goal默认/自定义8正例、60非法值负例，共68/68。首次脚本完整metadata容器比较误报4个goal正例失败（64/68），规范JSON比较后在新目录重试通过，旧失败保留，非生产源码故障。
- 实验：capsule_retry_v4探索双图36.483/124.900s安全到达；capsule_v6双图卡停；static_v5原图29.150s安全、长图39.317s卡停；static_v8双图卡停。dense_wide_v6（H=.667,reward=.06,history=.25/32）原图28.367s安全，长图虽ARRIVED103.283s但独立审计302段z=0安全平面重叠、minclear=-.076734m，必须拒绝，不列安全最快。
- 几何核查：长图最低中心z=.403403，半径.48的球底=-.076597；实际USD Ground顶面约-.200000，故没有穿入渲染地板，距实际地板仍约.1234m。审计z=0是更保守安全域，原审计与失败保留不改。ESDF在线guard未读地板高度且先返回occupancy检查，因而0block不能证明地面安全；计划补USD实际ground guard独立检查（本条时尚未应用）。
- 诊断归档：performance/p1_speed_20261015/capsule_review.md、motion_diagnosis/executor_candidate/。60Hz与40Hz路径离散差异仍存在；a16/tau.10较a64/.03减少90度转弯偏差但制动距离4倍。两种执行器证候选仅/tmp应用上下文检查、公式3200项相等及27参数比较，未应用/未编译/未飞行；不把草案当已实现。最新汇总脚本严格区分启动失败/实际卡停/不安全到达/正式重复，不以ARRIVED单字段算成功。
- 后续：完成当前600s失败记录；修补真实ground guard并回归；继续restart .10、垂速/中等响应及多速度筛查，最终在同版参数做重复验收。所有新增飞行都通过主入口串行运行，源/安装逐轮冻结，P0与场景保持不变。

### 2026-10-15 16:27 — [EGO2P1] 第一轮参数筛查与已证胶囊体积保存修复

- 状态：进行中；截至本条14次实际探索飞行，仅原图两次安全到达，扩图尚未通过；另2次启动错误无飞行，修复后重试正在运行。
- 修改：运动弦膨胀修复主入口构建3包成功、CTest36/36；审计工具42/42，增加pipeline-delay显式参数后45/45。新增完整常规+紧急制动路径均通过后保存原segmentKnown胶囊到既有256条verified_tubes_，纯motionSafe及失败候选不保存，不增加重放字段；新增狭壁/未知边界/撤销/容量等回归并覆盖goal核。两启动器新增clearance_reward、history采样间隔/容量环境参数，默认值保持。
- 实验：a64/tau.03与a8/tau.22、restart0/.05、delay.2/0、深度不确定性开/关、3/4m/s共14次探索；12次卡停、2次原图到达。raw+delay0+restart0+a64/tau.03原图4m/s为35.00s、3m/s为45.85s；扩图同参数4/3m/s分别45.667/138.800s卡停。现有完整审计均未见碰撞、外部保护或运动限值违规，但卡停不计成功。具体全量数值见performance/p1_speed_20261015/results.csv与各轮audit。
- 诊断边界：.05 restart使多份失败快照无法证，而0可接受；不能把放宽80ms期限当作修复。旧固定.2s预测延迟与实际即时执行不一致，delay0只作为纯仿真候选。25ms认证和60Hz执行仍有差异；raw_v3扩图末帧还出现约9mm历史自由包络缺口及6.35s回调峰值，不宣称严格实时或全场景安全保证。
- 验证与失败：胶囊版主入口构建3包28.8s成功，完整37/37 CTest、12.74s。首次capsule_v4双图因history_max_observations未加入benchmark AST求值context启动失败，无CSV；补context及显式数值校验后再次主入口构建成功。真实launch构造首次因沙箱ROS_LOG_DIR默认不可写导致10失败/12通过，改到/tmp后22/22通过，Node.execute=0。失败日志和JSON另存不覆盖；飞行重试使用新批次capsule_retry_v4。
- 来源/恢复：本次仅P1；P0/旧实验/场景均保留。修复前源码包/二进制、逐轮冻结与审计保留；胶囊补丁来源/tmp准备稿复制到新performance归档。当前静态配置全部memory_uncertainty_rate=0；混合时变来源门控的进一步保守检查在评审中，尚不将此版本定为最终配置。
- 后续：完成胶囊修复筛查、必要参数优化与多速度/重复验收，最终更新版本说明与结果报告。

### 2026-10-15 15:55 — [EGO2P1] 开始新一轮纯仿真运动参数与多速度优化

- 状态：进行中；新基线双图均卡停，没有到达通过结论。
- 目的：用户明确授权在EGO2P1继续优化加减速度及其他仿真参数，目标4m/s安全快速到达并实测更高速度、用时和计算性能；覆盖先前P0停止研究的限制。
- 项目路径：`/home/starry/isaac-data/EGO2P1`。
- 涉及文件：`src/pc_gvf/src/paper_guidance.cpp`、`src/pc_gvf/test/cpp/observed_space_check.cpp`、`tools/sim4_benchmark.py`、`tools/test_sim4_benchmark.py`；新证据`performance/p1_speed_20261015/`及基线`performance/sim4_20261015/p1_baseline_v4/`；本日志/HISTORY。
- 修改内容：认证恒加速度步的弦线膨胀由全局最大加速度改为该步实际加速度范数，静止段不再无条件增加5mm；新增紧边界与未知运动负例。工具显式支持tau、垂速和restart余量，独立新输出目录、逐轮源码/安装/执行二进制冻结及变化检测，完整安全审计保持。
- 验证：修改前主入口基线原图34.867s/扩图38.400s因20s探索卡停终止，均实际达4m/s、扫掠/外部保护/限值0，但未到达。原图本体0.58m亦不可证，扩图0.58可证/.585不可证；故修正并非全部问题的保证。补丁应用检查通过；新工具/tmp隔离测试42/42由协作者完成，当前源码构建/完整回归尚未运行。
- 影响与边界：仅P1；不修改来源P0、地图或机体.48m。25ms认证与60Hz实际离散积分仍非完全相同，修复只对认证模型内部严格成立，继续以独立扫掠验收实际飞行。不宣称实物能力或全参数空间绝对最优。
- 回滚/备份：本批次`before_source.tar.gz`、167源文件SHA、旧重放二进制与基线全轨迹保留；不使用Git reset覆盖继承改动。
- 后续事项：构建回归、4m/s参数筛查与双图重复、多速度扫描、性能/失败完整汇总并更新最终状态。

# EGO2P1 工作日志（完整继承EGO2P0及此前历史）

### 2026-10-15 15:52 — [EGO2P1] 完整复制EGO2P0、独立路径适配与当前算法状态说明完成

- 状态：完成（复制/说明/构建回归）；没有新增飞行或算法优化，继承4m/s稳定目标仍未通过。
- 目的：执行用户“EGO2P0全部复制到新EGO2P1，并说明代码算法项目进展状态”的要求，原目录保留。
- 项目路径：`/home/starry/isaac-data/EGO2P1`；来源`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：新目录完整树；11个最小适配文件为`scripts/build_isaac_ros_workspace.sh`、`run_isaac_fov_gvf_navigation.sh`，`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`navigation_benchmark.launch.py`，`src/pc_gvf/src/depth_angular_controller_node.cpp`，`tools/run_navigation_benchmark.py`、`run_omni_comparison.py`、`analyze_navigation_benchmark.py`、`verify_navigation_replays.py`、`speed_benchmark.py`、`sim4_benchmark.py`；新README/AGENTS/EGO2P1_VERSION、WORK_LOG、来源README/AGENTS/WORK_LOG原样备份；`performance/copy_from_ego2p0_20261015/`完整复制清单/适配前备份、验证脚本/日志/参数/冻结/摘要。HISTORY的CHANGELOG、PROJECT_INDEX、FOV_GVF_FAMILY、README同步更新。
- 修改内容：`cp -a --reflink=auto`包含隐藏Git、地图、旧实验、失败、冻结包和未提交源码，原样14,093文件、1,186目录、15,279条目、3,238,865,395字节；SHA/权限/所有者/mtime一致，无符号链接、无源副本共享文件inode。随后仅适配根目录、P1独立/tmp构建/安装/日志/重放前缀、机器人ID、通用工具新版本枚举，保留P0等旧枚举和系列共享锁。当前①开range²=0、②③④⑤⑥关、4/a64/tau.03、world/E及人工C关/目标C.15均不改。文档说明模块、代码索引、参数、地图、27次继承尝试及未修复5mm认证阻塞；没有把继承17次到达算成P1成绩。
- 验证：先完整原样复制校验通过；最终原目录全部15,279条目与开始清单一致，继承performance内容零变更，非适配的167个源码/脚本/工具文件内容一致，无缺失或意外改动。`bash -n`两个主Shell通过；`bash scripts/build_isaac_ros_workspace.sh`独立3包成功1分32秒。加载基础ROS/P1安装后`PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego2p1_isaac_build/pc_gvf --output-on-failure`36/36、11.31s；平台+两审计工具pytest41/41（18+23）、0.21s；真实源/安装参数构造22/22、Node.execute=0；P1 goal replay读取继承旧O/N两帧2/2输出一致。两个重放器二进制与P0相同，控制器因默认ID改变SHA不同；选定新运行源/安装170SHA已冻结。
- 失败保留：首次Python测试未加载ROS，收集阶段3导入错误；第二次只加载基础ROS、缺P1生成消息，1导入错误；完整加载环境后41通过，未改源码。`python_checks.log`、`python_checks_ros_retry.log`保留，成功日志另存，不省略失败。Git状态检查刷新副本`.git/index`的stat缓存，已在最终差异中单列；来源index和工作树未变，无提交/暂存内容操作。
- 影响与边界：只建立独立开发副本与说明，未改避障策略、数值参数、地图或来源项目。未运行新的ROS控制节点、Isaac飞行、手柄实测或遗留计划；原4m/s候选最新扩图卡停、同版双图各3次未通过的结论保留。旧performance、旧报告、Git历史和专项工具的来源绝对路径不批量替换。普通构建/测试产生缓存不逐文件记录；没有人工清理/迁移旧缓存。没有提交、推送、改远端或回退Git HEAD。
- 回滚/备份：来源原目录保留；新目录`source_before.json`、`destination_raw.json`、`raw_copy_summary.json`为原样证明，`adaptation_before.tar.gz`和`adapted_files.json`保存11文件修改前后，来源三份说明/日志原样备份在P1根目录。最终`source_final.json`、`final_copy_state.json`、`validated_runtime_sha256.json`和`completion_summary.json`支持恢复/复核；来源历史失败不覆盖。
- 后续事项：无本次复制必做余项。后续开发使用EGO2P1并遵循新AGENTS双重记录；继续调参或飞行需依用户后续指令，复制不自动恢复旧研究。


### 2026-10-15 15:46 — [EGO2P1] 完整复制当前EGO2P0，开始独立适配与检查

- 状态：原样复制已完成并核验；P1主入口构建正在执行，最终验证待完成记录。
- 目的：用户要求复制全部内容到EGO2P1并说明当前代码算法、项目进展与状态。
- 项目路径：`/home/starry/isaac-data/EGO2P1`，来源`/home/starry/isaac-data/EGO2P0`保留不写入。
- 涉及文件：完整来源树；11项运行路径/身份文件见`performance/copy_from_ego2p0_20261015/adapted_files.json`；本目录README、AGENTS、EGO2P1_VERSION、WORK_LOG及新复制证据；HISTORY统一记录。
- 修改内容：使用`cp -a --reflink=auto`完整复制含隐藏Git/历史试验。复制前后14,093文件、1,186目录、15,279条目、3,238,865,395字节内容/权限/所有者/mtime一致，无源/副本共享文件inode，来源复制期间未变。随后仅适配新根目录、独立/tmp安装/构建/日志/重放、机器人ID和通用工具版本枚举；保留旧枚举/历史身份。来源README/AGENTS另存EGO2P0_README/EGO2P0_AGENTS；EGO2P0_VERSION原样留存。
- 验证：`raw_copy_summary.json`通过，`bash -n`两个主Shell通过；当前`bash scripts/build_isaac_ros_workspace.sh`运行中，日志`build.log`，此刻尚不宣称构建或回归通过。来源检查点的36/18/23等只是继承证据。未运行新飞行/优化。
- 影响与边界：算法核心、默认运动/感知/安全参数和地图不改；当前4/a64/tau.03仍未稳定验收。普通控制器/目标专用核区分保留；共享导航锁和域42保留。Git历史、旧失败与原数据不批量改写，无提交推送。
- 回滚/备份：来源原树及完整`source_before.json`保留；原样副本清单`destination_raw.json`；路径适配前11文件归档`adaptation_before.tar.gz`与逐项修改前后SHA。
- 后续事项：完成独立构建/回归/真实ROS参数构造、补全当前算法说明、最终来源未变和变化范围核对，再写完成结论；不启动试飞。


### 2026-10-15 15:38 — [EGO2P0] 纯仿真4m/s优化按用户要求停止并整理全部结果

- 状态：部分完成；用户要求停止。代码、试验和收尾资料已整理，4m/s同版双图稳定目标未通过，不再自动续跑。
- 目的：承接13:37纯仿真授权，探索高加减速度和快速响应下安全到达；按最新“先到这，整理做过什么及数据”指令停止优化，交付可核对的调整依据。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`scripts/isaac/beitong_joystick.py`、`run_fov_gvf_navigation.py`；`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`navigation_benchmark.launch.py`；`src/pc_gvf/src/depth_angular_controller_node.cpp`、`paper_guidance.cpp`、`paper_replay.cpp`及`include/pc_gvf/paper_guidance.hpp`；`src/pc_gvf/test/cpp/incremental_field_check.cpp`、`intent_response_check.cpp`、`observed_space_check.cpp`；`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、`test/test_platform_math.py`；新增`tools/sim4_benchmark.py`、`test_sim4_benchmark.py`及本批次脚本/协议/数据/归档；`SIM4_TUNING.md`、`README.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`；`HISTORY/CHANGELOG.md`、`FOV_GVF_FAMILY.md`、`README.md`。
- 修改内容：手柄2→4m/s；执行/认证加减速度1.2→4→8→64m/s²，响应τ.22→.03s、候选方向率8rad/s，前视3→1s、额外平滑关。world桥修复缓存body命令受当前yaw再次旋转；显式制动能力/响应参数同步且拒绝τ<.025。E默认开；80ms期限和100ms命令新鲜度未放宽。超时世界候选恢复最长5s不续期，显式候选纳入恢复，已接受速度只降不升，恢复优先于短C缓存；每次重新完整运动/E认证。目标核增加同步内已证球缓存及连续相同查询去重；P重放保存恢复速度并读旧O/N。纯单机关闭④球面与⑥共享，①保留σ0.005/pose.01但range²系数.001→0；②③⑤仍关。机体.48/基础包络.58、深度10、垂速1保持。工具独立核验完整扫掠、真实高速、运动限值、到达保持及实际参数。
- 试验结果：本批次14组，54计划/27启动尝试/26实际飞行；17安全到达（原12/扩5）、2完整240s超时、4探索20s卡停终止、3人工否决扩图部分轨迹、1扩图启动中断，另27项未执行。23条完整审计扫掠重叠/外部保护/限值全0；3人工部分轨迹不补称扫掠通过。原图最快35.033s来自a64/τ.03的fast_response，扩图最快126.883s来自a8/τ.22的ideal_depth，不能拼成同版稳定通过。proposal_recovery_confirm原图r1为123.033s真实到达，r3才是240s失败；扩图r3未跑。最新P候选扩图68.483s卡停在(123.028,-5.380,4.020)，未到终点；该版原图未运行。
- 性能与未修复：缓存16快照×旧/新×3共96重放字段一致，进程级均时73.314→61.903ms（约-15.6%），不代表在线实时。最新扩图callbackP95/峰值70.631/497.656ms，仍不满足严格60Hz。最新P只读诊断显示32方向但静止.58m可证/.585m不可证，a64固定5mm膨胀使零命令E认证失败；此帧阻塞已定位未修复，不外推全部失败。扩大视场未实现、未试验。
- 验证：通过主Shell `bash scripts/build_isaac_ros_workspace.sh`，最新`build_accepted_speed.log`三包成功28.9s；最新`ctest`日志36/36、11.19s；审计工具pytest23/23；平台18/18在本轮初始验证后无变更。ROS参数真实构造最终18/18及感知协议矩阵20/20，不执行节点。最新P只读读取旧O/N各1份共2/2输出一致。中间`ctest_recovery.log`34/36曾因用户目录NumPy污染失败2项，`PYTHONNOUSERSITE=1`重试2/2通过，失败和后续全36通过日志均保留。试飞均由`tools/sim4_benchmark.py`串行调用主运行Shell；人工部分中止原RUNTIME_ERROR和rejection保持，不写成600s超时。
- 收尾验证：只读重建`all_trials_inventory.json`并由独立代理复核分类；生成27行`results_table.md/.csv`、`stop_summary.json`、`final_integrity.json`及停止检查点。当前冻结170/170源/安装SHA匹配；46项既有场景/体素引用哈希匹配，原始文件链接无缺失；主机进程检查无本轮driver/Isaac/控制器/ROS导航。用户要求停止后没有新飞行、构建或算法修改，只有资料整理。文档/JSON链接和结构检查见收尾检查文件；未重新运行核心/飞行测试。
- 影响与边界：当前4/a64/τ.03为未稳定验收的纯仿真候选，不自动恢复2m/s；键盘/旧trace仍2m/s，人工C关/目标C.15。目标专用核飞行不等于真人手柄验收。地图几何/颜色、机体大小、EGO1P5/EGO1P6不改；没有提交或推送。仅把HISTORY中13:12标题归回相应正文之前以修复条目嵌套格式，旧正文与失败记录保留，记录规则R-001未改。
- 回滚/备份：修改前选定源码`performance/sim4_20261015/before.tar.gz`和SHA；各阶段candidate归档/旧二进制全部保留；当前`candidate_accepted_speed.tar.gz`和170项SHA。原进度/说明先存`pre_stop_metadata.tar.gz`再将task/progress/final_profile/RESUME_STATUS标为用户停止；原飞行结果/协议/审计不重写。恢复特定版本须核对对应协议和SHA再用主入口重建，不使用Git reset覆盖继承树。
- 后续事项：等待用户依据报告决定调整；不自动执行未运行项。完整修改、默认参数、每阶段失败、性能限制和证据入口见`SIM4_TUNING.md`及本批次结果表。


### 2026-10-15 13:37 — [EGO2P0] 开始纯仿真4m/s运动参数与执行一致性优化

- 状态：进行中；尚无本轮飞行通过结论。
- 目的：用户明确授权不匹配真实Seeker Mini，允许调整加减速度及其他无人机参数，目标为4m/s上限下安全快速抵达终点。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：本轮`performance/sim4_20261015/`，预计修改运动参数、world命令链及独立测试工具；最终以完成记录为准。
- 修改内容：计划初始总加减速度4m/s²、tau=.22s、速度上限4m/s；认证与执行同步，修复缓存body命令受yaw改变的问题。保留原机体几何和碰撞审计，原图/扩图串行试验。
- 验证：开始前沙箱内GPU驱动不可访问；在已授权本机GPU环境只读检查确认RTX5070可用且无导航运行。未启动新飞行，不以参数配置当作通过。
- 影响与边界：仅EGO2P0纯仿真，不用于真实无人机物理结论；历史试验与来源目录保留。
- 回滚/备份：`performance/sim4_20261015/before.tar.gz`、`before_sha256.json`保存本轮修改前源/状态，HISTORY另有独立备份。
- 后续事项：完成最小修改、构建回归，完整轨迹独立审计与重复到达测试；结束时追加实际结果。


### 2026-10-15 13:12 — [EGO2P0] 以3.5m/s选择高速优化基准、4m/s作为提升探测

- 状态：基准选择和实验标准配置完成；未证明临界稳定速度，未执行新算法修改或飞行。
- 目的：用户要求尽可能快、按当前障碍密度与刹车转弯/运行模型极限选择较高基准，不把算法超时/卡停当成物理速度上限。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：新增`PHYSICAL_SPEED_BASELINE.md`、`performance/physical_speed_baseline_20261015/baseline_definition.json`、`existing_trajectory_evidence.json`及原图/扩图各2、3.5、4m/s共6份协议；`README.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`；`HISTORY/CHANGELOG.md`、`FOV_GVF_FAMILY.md`。
- 修改内容：主优化目标cap3.5、下一提升档4、同算法对照2；手柄当前2m/s不变。新协议E+C0.15、G=false，移除二档回退对物理目标的定义，保留现有完整认证的局部减速；物理a1.2、tau.22、垂速1、机体.48、深度10保持；确认重复数5、原图240s/扩图600s。保存对应启动环境要求，全部标记未运行。
- 验证：两项并行只读核查分别检查47轮原始数据和动力学/执行链。本轮另读CSV复核既有3.5/4 G案例10条（含3条失败），成功轨迹高档持续/占比/横向加速度保存在JSON；6份新协议用现有GoalTrial读取通过，未构建、未运行核心回归或新飞行。3.5已有两图真实运动/到达证据但并非稳定全通过；4请求成功仅实测3.660，不当作4持续能力。物理公式3.5刹车5.804m、恒速R10.208m；4为7.467m/R13.333m；包含额外执行段和认证球的稳态墙前空间7.084/8.847m。当前10m为算法截断，不是无人机物理量程；理想约4.30m/s单前向观测认证边界不得写成全图或实机上限。
- 影响与边界：不把旧失败删掉后称安全，不保证密集急弯全程恒速3.5；只确定要攻克的名义速度上限与对照标准。当前运行模型不含真实多旋翼姿态/推力动力学，不能给出实机极限。另识别手柄yaw与缓存机体Twist转回世界的时间/坐标不一致及额外紧急制动分支问题，作为后续算法工作，未在本条宣称修复。运行源、速度默认、地图、原目录和历史实验不修改，无提交推送。
- 回滚/备份：新标准与协议独立目录，不覆盖旧批次；撤销本轮研究标准只需停止引用这些新协议，无需改运行程序。最终SHA与摘要在本批次保存。
- 后续事项：以3.5为主目标解决认证/执行一致性、紧急直线停车及连续曲率/空间限速；随后按4m/s压力档评估，不用当前2m/s调试档定义物理上限。

### 2026-10-15 12:57 — [EGO2P0] 手柄恢复2m/s并排查用户报告的高速撞墙

- 状态：2m/s回退及配置验证完成；撞墙原因仅完成初步排查，未复现碰撞或确认唯一根因。
- 目的：用户要求将速度恢复2m/s，并询问是否提速后安全刹车距离未随速度更新。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`scripts/isaac/beitong_joystick.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`README.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`；新增`performance/joystick_restore2_20261015/`；`HISTORY/CHANGELOG.md`、`FOV_GVF_FAMILY.md`。
- 修改内容：手柄默认输入速度及Cloud的JOYSTICK_HORIZONTAL_SPEED默认值3.5→2.0m/s，保留同一设置同步控制器speed/max_speed和桥接max_vx/max_vy，仍可显式环境变量覆盖；垂向1m/s、加减速度1.2m/s²保持。只恢复速度，没有悄然启用其他实验开关或修改刹车模型。
- 验证：主Shell`bash scripts/build_isaac_ros_workspace.sh`构建3包成功（1分31秒）；安装的两个launch与当前源逐字节一致。44/44手柄/控制数学既有Python检查通过；源码/安装各手柄默认、显式2、键盘和目标协议8/8实际ROS Node参数构造均控制器/桥接2m/s、垂速1、加速度1.2。模拟手柄满杆直行2m/s、斜杆合速度2m/s，桥接不改变该命令；不执行Node.execute，不启动ROS控制或Isaac飞行，未重新做全36项或碰撞闭环。
- 排查证据：保留用户最近`20261015_125048`完整性能段、控制器日志、实际启动参数及一份N重放。实际启动参数speed/max_speed=3.5，重放的运动配置reference_speed=3.5、max_accel/brake_accel=1.2、tau=.22、delay=.2，未遗留2m/s参考速度。`paper_guidance.cpp`的required stopping及完整运动/制动积分直接使用测得速度和候选指令，所以刹车距离会随速度变化，并非固定2m/s距离。按v×.2+v²/(2×1.2)作直线示意，2m/s为2.067m，3.5m/s为5.804m；此数字不含机体、观测等额外余量，也不代替完整弯曲运动证明。该次日志命令仿真时间间隔最大366.667ms、回调墙钟最大401.181ms，存在25条COMPUTE_DEADLINE状态日志；配置中额外独立紧急直线制动认证为false。超时/过期会归零命令，普通转向后的制动尾部与该独立归零轨迹存在不同，需进一步核查时延/执行分支覆盖。本次没有带碰撞时刻的完整轨迹或独立扫掠审计，日志ESDF保护0不能否定用户观察；不能把上述风险直接写成已确认的碰撞原因。详细数值见`diagnosis.json`。
- 影响与边界：本轮只将手柄速度恢复2m/s；不宣称2m/s消除所有问题。上次3.5试用只做参数/输入/桥接一致性验证，没有3.5手柄闭环碰撞验收；旧47轮固定目标测试也不代表用户这次手柄轨迹通过。原图/渐变地图、机体、算法及其他默认开关、EGO1P5/EGO1P6均不修改；未中止用户进程，未提交推送。
- 回滚/备份：本批次`before/`与`before_sha256.json`保存修改前两个3.5源文件；原参数、用户日志、重放和性能快照独立保存，不覆盖旧批次。最终配置/诊断/源SHA与归档均在本批次。
- 后续事项：用户重启原手柄命令后生效；如曾显式设置JOYSTICK_HORIZONTAL_SPEED=3.5，应改为2。碰撞根因仍需在带逐帧轨迹和超时执行来源的数据上定位，不能仅加速度数值就宣称修复。

### 2026-10-15 12:46 — [EGO2P0] 按用户要求将手柄控制速度改为3.5m/s试用

- 状态：完成配置与检查，留给用户手柄试用；未启动新飞行，不宣称3.5m/s新增安全验收。
- 目的：用户明确要求“将手柄控制速度改到3.5m/s我试试”。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`scripts/isaac/beitong_joystick.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf/launch/navigation_benchmark.launch.py`、`README.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`、`performance/joystick_speed35_20261015/`；`HISTORY/CHANGELOG.md`、`FOV_GVF_FAMILY.md`。
- 修改内容：手柄默认输入速度2→3.5m/s；Cloud启动器在joystick模式从同一个`JOYSTICK_HORIZONTAL_SPEED`（默认3.5）同步设置控制器speed/max_speed及桥接max_vx/max_vy，避免后级仍被2m/s限制；拒绝非有限/非正限速。键盘/trace原限速2m/s保留，目标benchmark原生参数求值加入非手柄manual_speed=2，最终限速仍由原协议覆盖。垂向执行上限1m/s、物理加减速度1.2m/s²、tau0.22s、机体/认证半径、全套避障及新地图颜色保持。
- 验证：经主Shell`bash scripts/build_isaac_ros_workspace.sh`构建3包成功（0.75秒），安装后的两个launch与源码逐字节一致。`/usr/bin/python3 -m pytest -q src/pc_gvf/test/test_beitong_joystick.py src/pc_gvf/test/test_manual_control_math.py`44/44通过。源码及实际安装位置分别构造手柄默认3.5、手柄显式2、键盘2、目标协议2，8/8实际ROS Node参数求值一致，控制器与桥接四处速度一致、垂速1/加速度1.2保持；没有调用Node.execute或启动ROS控制。模拟手柄满杆直行输出(3.5,0,0)，斜杆合速度3.5（非每轴3.5），实际桥接转换不再截为2。首次配置构造因默认`~/.ros/log`只读失败，指定本批次可写ROS_LOG_DIR后重试8/8通过，原错误日志保留。未重跑全36项/Isaac飞行或手柄实操。
- 影响与边界：只把用户手柄试用档位改为3.5，实际飞行仍会因避障、加速与证据不足减速/停车；不是扩图3.5可靠性或实时性已通过。所有旧速度研究中的2m/s默认描述属于当时检查点，不覆盖本条当前试用配置。EGO1P5/EGO1P6、地图、物理模型及历史失败未修改，未中止已有进程，未提交推送。
- 回滚/备份：本批次`before/`与`before_sha256.json`保存三个改前源文件；`JOYSTICK_HORIZONTAL_SPEED=2`可让手柄输入和上下游限速一同回到2m/s，或恢复三源后通过主Shell重建。配置/测试/构建及初次失败日志、最终源SHA归档保留。
- 后续事项：用户重启之前的新地图手柄命令试用；无需再构建。

### 2026-10-15 12:41 — [EGO2P0] 渐变方向改为从下往上赤橙黄绿青蓝紫

- 状态：完成。
- 目的：按用户对上次“上红下紫”的修正，把新增两段改为下红上紫。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`tools/prepare_speed_maps.py`、`scenes/speed_cloud_long/height_colors.usdc`、`manifest.json`、`src/pc_gvf/src/verified_start.cpp`、`README.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`；`performance/scene_colors_bottom_up_20261015/`；`HISTORY/CHANGELOG.md`、`ENVIRONMENT_AND_ASSETS.md`、`FOV_GVF_FAMILY.md`。
- 修改内容：反转高度色表为从低到高赤、橙、黄、绿、青、蓝、紫，底部RGB(1,.05,.03)、顶部(.55,.06,1)，原第一段仍蓝色。同步生成器与manifest的bottom_to_top/top_to_bottom，颜色层新SHA为`fea2e740be1dcd4a64884c4d202f6bd6bb89ddbc8a2e766be44f2d706b51801a`。USD包装文件逐字节未变，SHA仍`4c080a85a6bf73fd972e2024e3e3a84def7bc4c81b20ca66f1a271d2cba4eb17`；起步校验接受新颜色层精确SHA，并保留旧精确SHA供历史恢复，其他依赖校验不变。
- 验证：主Shell构建3包成功（30.6秒）；`ctest --test-dir /tmp/fov_gvf_ego2p0_isaac_build/pc_gvf -R '^observed_space_check(_goal)?$' --output-on-failure`2/2通过（人工及目标核心的新场景起步/错误场景拒绝），没有重新运行全36项。Python语法检查通过；6个Mesh的几何/拓扑/法线/变换及体素与上一版相同；51个高度色样和原段蓝色验证通过；临时重建USD/颜色层/体素逐字节相同。无ROS/无控制器/无飞行的离线渲染生成两张1400×900预览，新增段近景已目视确认下红上紫。本轮未运行飞行、未重测速度。
- 影响与边界：仅反转新增部分颜色并更新固定颜色层SHA，原图、障碍几何、碰撞/深度、控制参数与速度不变。未修改EGO1P5/EGO1P6，未中止用户运行；未提交或推送。上次上红下紫的截图/记录代表上一版，全部保留。
- 回滚/备份：本批次`before/`及`before_sha256.json`保存修改前五份文件（含旧颜色层）；旧实验不覆盖。恢复后通过主构建入口重建。当前源/资产及证据SHA保存在本批次`sha256.json`，完整性汇总为`completion_summary.json`。
- 后续事项：重启原新地图手柄命令即可看到新色序，无需更改命令或再构建。

### 2026-10-15 12:39 — [EGO2P0] 新增两段地图模型改为高度彩虹渐变

- 状态：完成。
- 目的：按用户截图要求消除新增部分的全灰外观，从上到下显示赤橙黄绿青蓝紫；原有第一段继续蓝色。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`tools/prepare_speed_maps.py`、`scenes/speed_cloud_long/speed_cloud_long.usda`、`manifest.json`、新增`height_colors.usdc`、`src/pc_gvf/src/verified_start.cpp`；`README.md`、`EGO2P0_VERSION.md`、`SPEED_ENVELOPE.md`、`WORK_LOG.md`；新增`performance/scene_colors_20261015/`；`HISTORY/CHANGELOG.md`、`ENVIRONMENT_AND_ASSETS.md`、`FOV_GVF_FAMILY.md`。
- 修改内容：确认旧灰色来自Mesh引用越出引用范围的材质绑定（原Blue/White材质未随Mesh子树引用）。对`SpeedTile1/2`障碍显式绑定HeightRainbow预览材质，按世界Z=-0.2至19.8m的51个高度采样写入索引式顶点颜色，顶部红、依次橙黄绿青蓝、底部紫，连续插值。独立508KiB二进制层只含displayColor及索引，不含几何；两段共用该层。原段保留Blue，新增地面显式绑定原White。生成器同步生成新材质并保持拒绝覆盖已有地图。新场景SHA为`4c080a85a6bf73fd972e2024e3e3a84def7bc4c81b20ca66f1a271d2cba4eb17`，颜色层SHA为`decb259e9e786874e0dc5c2d154ca4394ba47acd43d207dad5ef30a4c6045e9e`。起步白名单明确接受新SHA且校验颜色层及原USD依赖；旧精确SHA仍支持历史复现，没有取消验证。
- 验证：经主入口`bash scripts/build_isaac_ros_workspace.sh`构建3包成功（约61秒）；ROS/install环境下`ctest --test-dir /tmp/fov_gvf_ego2p0_isaac_build/pc_gvf --output-on-failure`36/36通过，含新着色场景起步和错误场景/体素组合拒绝。离线USD验证全部6个Mesh的顶点/拓扑/法线/范围/变换与旧层一致，体素SHA不变，原段蓝色、两新增段彩虹和地面绑定检查通过。临时目录重建USD/颜色层/体素三份逐字节一致。验证脚本首轮临时目录未配相邻原资产，生成检查失败；补齐临时只读符号链接后通过，初次失败日志保留。以与主程序一致的MinimalRendering模式2做无ROS/无输入/无飞行的离线RGB渲染，两张1400×900真实渲染已目视确认；预览不计飞行或到达测试。Python语法检查通过。没有重新进行47轮速度测试，不能把外观验证写成新速度成绩。
- 影响与边界：改变材质、颜色及精确场景白名单，障碍几何、碰撞网格、传感器布局、运动模型、算法参数和速度上限不变；原Cloud文件和EGO1P5/EGO1P6未改。上轮冻结v4/最终完整性结论只针对2026-10-15 00:36研究检查点，当前白名单源码和场景外观已有本条明确更新；旧原始结果和冻结不重写。本轮未启动手柄控制、未中止用户进程、未提交推送。
- 回滚/备份：修改前四份文件及SHA在`performance/scene_colors_20261015/before/`与`before_sha256.json`；旧完整源码/实验在上轮冻结包。恢复旧四文件后通过主Shell重建，旧USD不引用新颜色层，可保留该层文件而不使用。`validation.json`、构建/CTest/渲染日志、原始失败及两张预览均保留，最终源和资产校验见本批次`sha256.json`。
- 后续事项：退出旧仿真后，使用此前新地图手柄命令重新启动即可生效；无需变更命令或另行重新构建。

### 2026-10-15 00:36 — [EGO2P0] 分档提速、双分支认证及三段扩图研究结束

- 状态：本轮实现与测试完成；总体目标部分完成。原图最高三次全通过档位3m/s已确认；扩图可靠速度上限、可靠最短时间及整体导航提速未建立。
- 目的：阅读现有算法/文本/历史后，优化固定目标避障、逐档增加真实速度，并在保留障碍尺度的扩增地图评估安全、时间及计算性能。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`navigation_benchmark.launch.py`；`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`verified_start.hpp`、新增`goal_proposal_cache.hpp`；`src/pc_gvf/src/depth_angular_controller_node.cpp`、`paper_guidance.cpp`、`paper_replay.cpp`、`verified_start.cpp`；`src/pc_gvf/test/cpp/observed_space_check.cpp`、`intent_response_check.cpp`；`scripts/isaac/run_fov_gvf_navigation.py`；新增`tools/prepare_speed_maps.py`、`speed_benchmark.py`、`test_speed_benchmark.py`、`summarize_speed_benchmark.py`；新增`scenes/speed_cloud_long/speed_cloud_long.usda`、`occupancy.bin`、`manifest.json`；`README.md`、`EGO2P0_VERSION.md`、`SPEED_ENVELOPE.md`、`WORK_LOG.md`、`performance/speed_envelope_20261014/`；`HISTORY/README.md`、`CHANGELOG.md`、`FOV_GVF_FAMILY.md`、`ENVIRONMENT_AND_ASSETS.md`。
- 修改内容：修正benchmark桥接固定2m/s，使后级和控制器同协议限速，保存真实协议哈希/实际参数；渲染器从场景元数据读取网格，拒绝错误单位/坐标；固定SHA认可三段扩图及原USD依赖，一次性起步自由球不扩权。可选E认证同时证明正常转向和独立超时直线制动；可选C缓存世界目标，默认TTL0.15s、位移0.75m/改向0.05rad失效，每条最终命令仍重新认证；可选G在目标走廊具备制动距离+2m空间时提速，否则按2m/s绕行。R余量沿完整扫掠路径收取，默认0；R0.05和C5s失败未采用。新N重放格式向后读取旧M及更早格式。所有新机制默认关，常用手柄速度不升档。原地图160×120×20m/.4m沿X复制三段，中段Y镜像，480×120×20m、1200×300×50体素；起点(-64,-11.2,2.8)，目标(384,11.2,2.8)，航程约449m。新增分档驱动、完整机体独立扫掠/限值/配置审计、失败早停和实际达到95%档位至少1s的检查，所有失败及未执行项分别保存。
- 验证：主构建`bash scripts/build_isaac_ros_workspace.sh`最终3包成功；`ctest --test-dir /tmp/fov_gvf_ego2p0_isaac_build/pc_gvf --output-on-failure`最终36/36；ROS/install环境下Python平台15项及审计负例7项共22/22；旧M16份与新N代表样本合计82/82在冻结v4重放决定一致（不重放壁钟看门狗）。扩图60几何样本、网格三段一致及临时重建USD/体素逐字节一致通过。两个隔离ROS域142真实构造正常，3s SIGINT工具退出124为有意终止，不计飞行。最终运行源码/脚本与冻结v4逐SHA一致；图表已打开目视核对。全部飞行通过原主Shell串行启动：47条已结束飞行含29安全到达、14完整240/600s超时、4探索卡停终止，完整0.48m机体扫掠重叠/外部保护/总速垂速加速度执行命令限值违规全0；另6条非完整记录不计成功率。原图独立3m/s E+C0.15三次70.133/73.267/78.567s，均73.989；原2m/s三次69.117/70.567/69.317s，均69.667，整体提速未通过。原图最快单次2.5m/s E为59.850s，但确认1/3；长图最快2.25m/s为221.833s，确认1/3。长图2m/s E+C与3.5m/s E+C+G均2/3，无可靠上限。请求4m/s原图实测峰值3.660m/s，未达到4档，不宣称已验证4m/s。控制回调最坏4485.2ms，严格60Hz未通过；完整成本/jerk/净空在表及原始数据。
- 失败与修正：初版扩图缺米制/Z向上元数据导致静止并中断，保留`invalid_map_v1`；两次平台测试环境路径收集失败，修正ROS/PYTHONPATH后通过；审计初次5/7、NumPy布尔序列化2失败，转换原生整数后7/7；新增C++变量与旧变量重名构建失败，作用域修正；v2重复声明restart参数导致2次真实启动失败，去重后构造及飞行通过。两个已否决配置的排队未飞行取消不计分母。原2m/s长图基线约516仿真秒后每帧约10秒，此前COMPUTE_DEADLINE持续卡停，读取GPU2%、显存2.2/12GB、可用RAM18GB后仅中止本任务PID50239；不写成600s完整超时。长图2.25第三次取消尝试找到PID为空、没有发送信号，其600s超时照实计入。所有失败日志/CSV/快照/版本保留；最终核对无本轮飞行或控制器进程。
- 影响与边界：研究对象为原Cloud及三段扩图固定目标仿真，三次全通过仅是本轮有限测试证据，不证明全场景或实飞安全。机体0.48m/基础认证0.58m、真实1.2m/s²加减速度、tau0.22s、垂速1m/s与外部保护保持；不以保护救援计成功，不给控制器完整离线网格作在线规划。原图/EGO1P5/EGO1P6、历史记录、系统依赖和常用入口保持；继承未提交工作树未回退，未提交/推送。本轮结论覆盖本次最终源码，不借用复制检查点验收。
- 回滚/备份：`performance/speed_envelope_20261014/baseline/source_sha256.json`及`sources_complete.tar.gz`完整保存修改前174份源/脚本/工具/场景、原二进制；`frozen_v2/v3/v4`保留各实际版本（v2启动失败不得常用）；`frozen_final`保存最终源、工具、文档/总账及二进制校验，`final_integrity.json`记载完整性。恢复明确快照后通过主Shell重建，不能Git reset丢弃继承改动。原图3m/s速度能力协议单列`accepted/original3_protocol.json`，要求E=1，范围及时间代价见`accepted/profiles.json`；没有扩图高速推荐。
- 后续事项：优先定位几何证明计算尾延迟和超时后的持续停滞，再进行新的独立扩图确认；当前没有继续运行的后台试验。不要放宽未知空间/包络/制动参数来获取速度成绩。

### 2026-10-14 21:13 — [EGO2P0] 复制检查点完成，后续并发开发单独保留

- 状态：复制检查点完成；当前实时目录已由另一任务继续修改，不能把本次检查点测试当作其后续代码验收。
- 目的：区分本次完整复制与正在并发进行的速度/地图研究，保留所有新工作。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：`README.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`、`performance/copy_from_ego1p5_20261014/completion_summary.json`及两次最终复核记录；`HISTORY/CHANGELOG.md`与项目总入口补注。
- 修改内容：原样复制/路径适配/构建/测试在21:07完成后，活动任务“EGO2P0”已启动自身基线并开始新地图与代码调整。本任务未修改或删除其`performance/speed_envelope_20261014/`、`scenes/speed_cloud_long/`、`tools/prepare_speed_maps.py`等内容，也未中止其进程；文档明确限定为本次复制验证检查点。
- 验证：初始原样复制SHA/元数据全部一致；主构建3包、核心36/36、平台15/15、launch纯构造16/16、工具30/30均通过。最终EGO1P5原13000条目仍全部不变，副本无源条目缺失、无共享文件inode。第一次严格最终审计因新目录Git索引字节刷新与并发实验新增内容失败；Git87项暂存路径/模式/对象ID及HEAD核对相同。第二次审计又检测到并发地图/工具新增与构建后launch变动，因此未声称当前实时目录等于已测快照。两次失败完整保存在`final_preservation_check.initial.json`和`final_preservation_check.concurrent.json`；构建前161份SHA及16配置验证证据保留。汇总生成前的链接检查曾因汇总缺失失败，汇总已生成，随后检查链接。本任务未重新测试另一任务的新代码。
- 影响与边界：本复制任务未新增算法、未启动飞行、未提交推送；并发任务已开始新的飞行和开发，其运行结果不计入本次验证。README/版本说明的开关、参数、产物身份均是21:07复制检查点状态。来源EGO1P5未变。
- 回滚/备份：完整P5来源、原样复制清单、构建前源码SHA、运行适配差异及失败核验均保留；未回写旧索引、未回滚并发修改。
- 后续事项：本次复制任务无；后续EGO2P0优化由已有任务继续并独立验证。

### 2026-10-14 21:11 — [EGO2P0] 开始分阶段速度与扩增地图优化

- 状态：进行中；不提前声明安全速度或最短时间。
- 目的：按用户要求阅读当前代码与历史后，补齐①④⑥默认组合基线，扩增原Cloud地图并逐档提速、独立审计安全和性能。
- 项目路径：`/home/starry/isaac-data/EGO2P0`。
- 涉及文件：本轮`performance/speed_envelope_20261014/`；后续将修改benchmark启动器、场景准备/运行、初始自由球验证、速度测试工具及项目状态文档，完成后逐项记录。
- 修改内容：冻结174份当前源码/脚本/工具/场景哈希、关键源码和两份安装控制器；原场景、来源目录和旧实验保留。基线使用当前默认①④⑥和2m/s、1.2m/s²动力学。
- 验证：阅读统一记录规则、EGO2P0/AGENTS、README、最新WORK_LOG、RETAINED_OPTIMIZATIONS、核心认证/控制/场景/benchmark代码。已启动主Shell GPU基线，当前正在飞行，尚未得到完成结论。发现benchmark桥接仍固定2/2/1m/s，需与协议同步；扩增初始认证不能跳过原有SHA证明。
- 影响与边界：此任务只开发EGO2P0；不推送、不改EGO1P5/EGO1P6或依赖。安全包络0.58m、机体0.48m、物理加减速度1.2m/s²保留。高速度先在明确协议中测试，默认人工速度暂保持2m/s。
- 回滚/备份：`performance/speed_envelope_20261014/baseline/`。
- 后续事项：完成扩增资产、协议一致性、核心回归及分档重复飞行；记录所有失败和实际最高速度，区分设置上限与实测速度。


### 2026-10-14 21:09 — [EGO2P0] 从当前EGO1P5完整复制并建立项目状态说明

- 状态：完成；复制和独立构建/回归完成，未启动新飞行。
- 目的：按用户要求将EGO1P5全部内容复制到新文件夹EGO2P0，并说明当前代码算法、项目进展与状态。
- 项目路径：`/home/starry/isaac-data/EGO2P0`；来源`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：完整目录复制（12041文件、959目录含根、13000条目、2175646100字节）；9个路径/身份适配文件：`scripts/build_isaac_ros_workspace.sh`；`scripts/run_isaac_fov_gvf_navigation.sh`；`src/pc_gvf/src/depth_angular_controller_node.cpp`；`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`；`src/pc_gvf/launch/navigation_benchmark.launch.py`；`tools/run_navigation_benchmark.py`；`tools/analyze_navigation_benchmark.py`；`tools/run_omni_comparison.py`；`tools/verify_navigation_replays.py`；新/更新`README.md`、`AGENTS.md`、`EGO2P0_VERSION.md`、`WORK_LOG.md`，另存`EGO1P5_README.md`、`EGO1P5_AGENTS.md`；`performance/copy_from_ego1p5_20261014/`校验与测试证据；工作区`HISTORY/README.md`、`PROJECT_INDEX.md`、`FOV_GVF_FAMILY.md`、`CHANGELOG.md`。
- 修改内容：使用`cp -a --reflink=auto`完整复制，包含`.git`、隐藏文件、缓存和全部成功/失败实验；主构建/安装/日志/重放路径从ego1p5改ego2p0，默认机器人ID从ego1p5改ego2p0，通用工具添加新标签并保留旧映射。九文件完整差异在`adaptation_files.json`。算法、六开关①④⑥开②③⑤关、.48/.58m包络、2/1m/s限速及1.2m/s²动力学不变。补充基础算法链、实现/启用区别、分批成果与退化、组合未完成、最新手柄1733.236ms计算峰值，以及10月9日P5远端ego1p0发布与继承本地旧HEAD的区别。
- 验证：复制原样SHA256、mode/uid/gid/mtime逐项相同，无共享常规文件inode；`bash scripts/build_isaac_ros_workspace.sh`3包成功31.9s，只有禁用Python字节码产生的跳过byte-compiling提示；`PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ctest --test-dir /tmp/fov_gvf_ego2p0_isaac_build/pc_gvf --output-on-failure`36/36通过；source ROS与新install后`/usr/bin/python3 -m pytest -p no:cacheprovider src/pc_gvf_platforms/test -q`15/15通过（同样隔离用户Python环境）。真实launch构造16/16、验证工具30/30通过；161份构建前冻结源码未变，双core/双replay与P5冻结归档SHA相同4/4。6个Python AST、2个Shell语法及git diff --check通过。最终来源与适配范围复核见`final_preservation_check.json`，全部结果见`completion_summary.json`；未启动ROS控制节点或Isaac新飞行。
- 影响与边界：EGO1P5来源和EGO1P6等项目保留，历史成绩仍属原版本；旧专项drivers保留P5路径，后续使用需另建P0适配批次。共享ROS域42及单实例锁不改。新目录继承本地ego1p1/50f7c55和未提交工作树，没有创建提交、分支或推送，也未修改远端。构建/复制通过不等于最终①④⑥组合整机不退化。
- 回滚/备份：EGO1P5原目录保留；原README/AGENTS在新副本另存；源文件清单、9文件前后SHA和差异、构建前源码、测试日志、归档二进制身份全部保存在本次证据目录。
- 后续事项：复制任务无遗留；后续算法研究仍需补齐最终组合目标/人工对照、定位1733ms延迟峰值及动态起点过载，不在本次复制任务自动续跑。

### 2026-10-09 19:53 — [EGO1P5] GitHub原ego1p0支线已接收EGO1P5完整快照

- 状态：源码快照推送成功，并已读取远程引用确认。
- 目的：执行用户“EGO1P5只替换EGO1P0支线，不动其他内容”的要求。
- 项目路径：`/home/starry/isaac-data/EGO1P5`；仓库`https://github.com/Chenwill1899/Fov-gvf.git`。
- 涉及文件：当前项目Git可见的6039个文件及本工作日志；本机HISTORY总账与发布回执。
- 修改内容：`ego1p0`从`50f7c5500af86e31d0d9af03de8ec9a29ffb10e1`快进到源码快照`c6f2e3d0531d30e8934a51469d414ebef8411cf6`。原Git历史保留，源码、场景、测试、工具、报告、冻结/原始实验与失败记录均纳入，现有缓存忽略规则保持。随后只追加本发布记录。
- 验证：完整快照与本地文件一致；无≥50MiB单文件，最大场景17,460,539字节；普通推送返回0。远程复核仅ego1p0变化，dev仍`804f26e2e631d0571f5e1cc5088100481df1b16d`，默认分支仍dev，无新增标签。当前本地分支与原索引未改。此次未重跑算法测试、构建或飞行，既有性能/验收局限保持。
- 影响与边界：此次Git发布没有修改算法、配置、地图、依赖或EGO1P6等其他项目。本分支保留历史名称ego1p0，内容为EGO1P5。
- 回滚/备份：旧tip作为新快照父提交保留。最终远程提交及其他引用复核详见本机`HISTORY/EGO1P5_GIT_PUBLISH_20261009.json`。
- 后续事项：无。

### 2026-10-09 19:48 — [EGO1P5] 准备向GitHub原ego1p0支线发布当前版本

- 状态：发布快照准备中；远程推送结果随后追加记录。
- 目的：按用户明确要求，将EGO1P5发布至`Chenwill1899/Fov-gvf`，只替换原`ego1p0`支线内容。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：项目`WORK_LOG.md`、`HISTORY/CHANGELOG.md`及`HISTORY/EGO1P5_GIT_PUBLISH_20261009.json`；Git对象与远程`refs/heads/ego1p0`。
- 修改内容：沿用现有`.gitignore`构建当前完整项目快照，包括源码、场景、回归fixture、工具、报告及成功/失败实验记录。使用独立临时索引，保留本地`ego1p1`分支与原索引；以远程原tip为父提交，使用普通快进推送，不重写历史。
- 验证：远程只读清单确认默认`dev`与原`ego1p0`，前者为`804f26e2e631d0571f5e1cc5088100481df1b16d`，后者为`50f7c5500af86e31d0d9af03de8ec9a29ffb10e1`；项目无≥50MiB单文件。此次不修改算法、不重跑构建或飞行，旧实验结论保持其原始边界。
- 影响与边界：目标仅`ego1p0`；其他远程分支、标签、默认分支和本地EGO1P6等项目不变。
- 回滚/备份：原tip完整保留为新提交父级，可由历史恢复；发布前后引用与提交SHA记录到上述HISTORY JSON。
- 后续事项：核对快照与上传内容，推送后复核所有远程引用。

### 2026-10-08 20:44 — [EGO1P5] 恢复缺失的独立ROS构建产物供手柄运行

- 状态：完成构建恢复与启动前静态检查，未启动仿真。
- 目的：用户明确要求运行保留的EGO1P5；原命令因`/tmp/fov_gvf_ego1p5_isaac_install/setup.bash`不存在退出。检查时P5临时build/install目录均不存在，原因未推定。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：自动生成的`/tmp/fov_gvf_ego1p5_isaac_{build,install,log}`；`performance/runtime_restore_20261008/build.log`、`preflight.log`；本项目`WORK_LOG.md`与`HISTORY/CHANGELOG.md`。
- 修改内容：通过既有`bash scripts/build_isaac_ros_workspace.sh`重新构建P5自身源码，恢复独立安装目录。算法源码、功能开关、启动脚本和依赖版本未修改。
- 验证：主构建3包成功，约61秒；source ROS Humble及P5 setup后`ros2 pkg prefix pc_gvf`指向P5安装目录；六个包执行入口可枚举，人工控制器可执行且`ldd`无缺失动态库；主机`/dev/input/js0`存在。本次未执行CTest、平台回归或新飞行，不将构建成功写成避障性能验证。
- 影响与边界：用户可使用原P5手柄命令。未修改EGO1P6或其他版本，未启动导航进程。
- 回滚/备份：无算法改动需要回滚；临时构建产物可由同一主构建脚本再生成，原运行失败与本次结果均保留。
- 后续事项：由用户运行P5手柄启动命令。

### 2026-10-08 12:53 — [EGO1P5] 中断收尾：启用①④⑥，撤销动态卡停项，组合验证未完成

- 状态：部分完成；本轮被中断，后台试验与ROS进程已停止。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 目的与修改：按最新用户要求①深度不确定性、④球面记忆、⑥共享占据默认1；②动态单开240s原地超时，已恢复默认0；③增量、⑤输入滤波仍0。callback修复、goal专用几何复用保持。共享发布采用各当前本地观测最大stamp，修复较慢前相机阻塞侧向新障碍发布；逐track原时间和有效期仍保留。
- 涉及文件：`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、新`tools/ros_shared_async_probe.py`、`tools/run_enabled_features_ablation.py`、`tools/run_enabled_features_confirmation.py`、本轮验证/冻结/数据`performance/enable_gains_20261008`；同步项目与HISTORY状态入口。
- 验证：主构建3包成功；36项核心34首过、2因用户NumPy环境收集失败后PYTHONNOUSERSITE=1重跑通过；首次0/16旧二进制身份门失败保留，试开四项真实launch16/16、工具22/22、汇总工具11/11。共享异步旧版复现、新版10/10，原ROS共享正负探针通过。已完成轨迹及最后启动项见`interruption_status.json`；基线69.2s、共享69.566667s、球面71.683333s、不确定性71.0s到达，动态240s原地超时，扫掠/外部保护0。共享/球面单次jerk较高等不利数据保留，不冒称全面优胜。
- 影响与边界：最终三项默认值已通过主Shell重新安装；不确定性单开已完成到达；未完成的组合/人工比较不得写成通过。原始六profile manifest保留complete=false，SIGINT与KeyboardInterrupt记录保留。未修改P3/P4、地图、物理参数或依赖；未提交推送。
- 回滚/备份：`performance/enable_gains_20261008/before`保留此前165源及两控制器；每项仍可用既有环境变量设0关闭。
- 后续事项：本次中断后停止新实验，剩余验证待后续继续。


### 2026-10-08 — [EGO1P5] 按用户放宽采用条件试开①②④⑥，核查遗漏退化并验证默认组合

- 状态：进行中，尚未将新组合写成验证通过。
- 目的：用户要求不再因整机证据不足保持关闭；启用没有明确现版退化的已有能力，保留逐项比较并撤销实测退化项。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`默认①②④⑥0→1；共享发布时间戳修复及新验证脚本随后另记；实验归档`performance/enable_gains_20261008`。
- 修改内容：③增量和⑤输入滤波仍0，callback修复仍开，A-v4仍只goal；显式环境变量仍可逐项关闭。已冻结此前165份源码及两控制器。
- 更正：此前聊天将①④概括为仅证据不足并不完整。旧①单开有182.567s卡停轨迹，旧④一对jerk3.829→3.961。后续已有等价提速但未重测开启，不能宣称它们从无退化；本次重新比较当前实现，不删旧失败。②解析让行有预期进展代价。
- 验证：尚未执行新默认构建/运行。计划核心回归、真实launch启用/关闭检查、共享异步时间验证、逐项及组合主Shell运行；所有失败保留。
- 影响与边界：只改P5；不改P3/P4、物理模型或地图。
- 回滚/备份：`performance/enable_gains_20261008/before`。
- 后续事项：完成实际验证，按新发现的退化决定最终默认值并同步最终记录。


### 2026-10-08 11:36 — [EGO1P5] 完成43轨逐项消融，固定目标保留几何复用、人工撤销退化优化

- 状态：完成本轮逐项/边际消融和限定范围部署；未证明全部工况/指标非劣。
- 目的：遵循任意幅度正收益可保留、有运行退化则撤销对应路径，取消旧10%最低收益政策。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`src/pc_gvf/src/paper_guidance.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/launch/navigation_benchmark.launch.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`；新增`tools/run_retained_ablation.py`、`tools/run_goal_retention_ablation.py`、`RETAINED_OPTIMIZATIONS.md`；更新`README.md`、`EGO1P5_VERSION.md`、`ADAPTIVE_OPTIMIZATION.md`、`WORK_LOG.md`；实验/评估/冻结/失败材料在`performance/retain_gains_20261008/`。同步HISTORY的README、PROJECT_INDEX、FOV_GVF_FAMILY、CHANGELOG，记录规则版本未改。
- 修改内容：原计算及纯v4复用以PRIVATE编译宏产生独立core/controller/replay；主P5固定目标默认选goal产物，人工原产物逐字节等于冻结baseline。根盒预筛与组合移除，六项算法开关仍关、callback同样本修复仍开。补充默认人工程序身份记录；显式可执行文件override优先，旧demo未变。
- 验证：主Shell `scripts/build_isaac_ros_workspace.sh`构建成功；`ctest --test-dir /tmp/fov_gvf_ego1p5_isaac_build/pc_gvf --output-on-failure`最终36/36（31原项+5 goal），本轮平台15/15（源未改）、执行辅助39/39；评估工具47/47、运动审计工具18/18；真实launch三路由3/3。正式主构建8批37帧四版本1184重放/888相对比较严格一致，纯v4逐批P95均值降3.04%。实际运行均经主Shell，原36轨四版本×三场景×三重复，加全新3对goal和1条默认spin，共43轨；18/18目标到达、25条人工轨迹完成，扫掠/外部保护0，152495行限速/加速度审计0违规。六份部署产物与已测baseline/v4 SHA相同；P3/P4保护136/150份源码相同。
- 收益与退化：根预筛目标jerk及微扰跟随不利；全局v4旋转jerk +44.27%，组合也有独立/边际不利，全部撤销全局采用。纯v4固定目标原批jerk1.605629→1.569363，新批2.408522→2.033052，各3/3改善；新到达均时71.311111→70.672222s但标准差0.483716→1.429290s，单对慢0.95s、其他个别指标不利完整保留。采用仅限按预定工程规则未发现一致退化的固定目标范围，不以均值/重放收益覆盖不利数据。
- 失败记录：原36轨机器评估仍`REPRODUCIBLE_TASK_REGRESSION`；新通用采集器只跑goal仍`PENDING_EVIDENCE`，专门范围复核无一致不利，不冒称新36轨通过。初次CTest TAG采集失败、路由YAML tuple解析0/3后修复3/3；原36轨deadline文本21、关闭rosout异常36，新goal deadline基线3/候选2、关闭rosout各1/轨，子进程均exit0，日志均保存。
- 影响与边界：人工算法没有本轮新增收益；主固定目标已实际改动默认执行程序。静态Cloud、脚本输入证据不扩展至动态Isaac、多机实飞或真人操作体验。未修改PaperConfig、节点源码、M重放布局、P3/P4、环境依赖；未提交推送。
- 回滚/备份：原161文件baseline和四版本冻结、部署163文件及六产物均保存。固定目标可设置`FOV_GVF_CONTROLLER_EXECUTABLE=/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/lib/pc_gvf/depth_angular_controller`复现本轮原计算；最终文档/源码/证据索引保存在`final_state/`，不覆盖早期冻结。
- 后续事项：本轮既定消融已完成，无待运行作业；更多场景、到达方差及跨地图泛化未验证，属于后续研究，不当作当前通过项。


### 2026-10-08 11:05 — [EGO1P5] 36轨逐项消融检出退化，撤销全局部署并实施模式隔离

- 状态：受限采用方案正在构建验证，未提前判定完成。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 修改：`src/pc_gvf/src/paper_guidance.cpp`从组合恢复为编译期原版/纯v4两条路径；`src/pc_gvf/CMakeLists.txt`新增goal专用库/控制器/重放器及5项库回归；`src/pc_gvf/launch/navigation_benchmark.launch.py`按真实固定目标模式默认选goal控制器，人工入口与显式消融覆盖仍保留。没有改PaperConfig、节点源或重放M格式。
- 已完成实验：三场景×四变体×三次=36轨，12/12目标到达，扫掠/外部保护0，36/36实际身份和条件通过；独立运动硬门36/36、122370行，速度/加速度/目标限值0违规。核心工具47/47，新增动力学工具18/18；全部原始结果归档，未重写旧政策。
- 退化：v2目标jerk和微扰跟随三对不利；全局v4旋转jerk均值2.823168→4.072982(+44.27%)、方向均误差/进展/尾延迟三对不利；组合也有独立和边际不利项。全部停止全局采用；不能用离线计算收益抵消。v4旋转停车额外2.383s只发生一次，不冒称三次重复；组合停车未增。
- 有限收益范围：v4固定目标jerk三次均改善，均值约1.605629→1.569363；到达69.227778→69.238889s的微小波动未分辨，未宣称导航加快。采用模式隔离继续验证：人工使用原计算二进制，固定目标使用纯v4，禁止按实验标签选择算法。
- 新证据：assessment/tasks_assessment.json、tasks_review.json、motion_limits_assessment.json、audit_motion_limits.py及18项负例；IMPLEMENTATION_AND_REPLAY.md、SMALL_SAMPLE_INTERPRETATION.md。正式36轨非零child退出0、deadline日志21次，关闭阶段rosout异常36次均保留。
- 后续：核验隔离产物与已测baseline/v4二进制SHA；主入口默认选路检查和独立实际对照，若受限部署仍退化则放弃该项。最后同步全部状态入口和恢复证据。


### 2026-10-08 10:11 — [EGO1P5] 两项等价几何优化通过正式重放，进入逐项与组合整机消融

- 状态：进行中，尚未判定正式采用。
- 目的：执行用户“每一项都有正收益、有退化就放弃该项”，取消旧10%最低幅度但保留安全和能力约束。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`src/pc_gvf/src/paper_guidance.cpp`、新增`tools/run_retained_ablation.py`；`performance/retain_gains_20261008`中的四份控制器冻结、geometry/production_build_validation、capability_checks、assessment、closed_loop_plan和真实运行原始记录。
- 修改内容：分别恢复根盒有序管段预筛v2、单次查询固定段量复用v4及组合。通过主Shell入口构建三份冻结控制器，当前工作目录为待验收组合版本；保留未改动基线控制器作相同配置对照。新driver每种场景按3个交错重复运行4版本，主入口锁保证Isaac串行。
- 验证：正式构建重放8批1184次，296组四版本/888比较输出严格一致；逐批全组P95均值改善分别B→v2 7.11%、B→v4 3.04%、v2→组合4.16%、v4→组合8.18%，各8/8向好；v2合并P95仍慢0.23%、继承组P95点退化0.37%且区间跨0，未隐藏。主组合核心31/31、平台15/15、执行辅助XML39/39；四个冻结库各5项能力检查共20/20，评估工具47/47。三类×四版本×三重复的36条实际轨迹刚启动，不能当作已通过。
- 影响与边界：查询运算等价不等于整机全面优胜；旧六项可选机制仍关闭、callback绑定执行源保持开启。记录真实到达、安全、停车、jerk、跟随与回调尾延迟，并检查每项组合边际，不用平均改善掩盖退化。
- 失败记录：初次收集普通CTest误假定存在dashboard TAG文件，FileNotFoundError后改归档真实LastTest.log/CTestTestfile/stdout及ament XML，未伪造或重跑覆盖；全部原失败/旧10%政策保留。
- 回滚/备份：本轮baseline和controllers/{v2,v4,combined}完整源码/二进制哈希冻结，已注册36条计划；P3/P4未改，未提交推送。
- 后续事项：完成真实逐项消融，按结果保留通过项、移除退化项，再更新默认状态与全部入口。


### 2026-10-08 09:49 — [EGO1P5] 按用户新标准重新验证并保留小幅正收益

- 状态：进行中。
- 目的：用户取消最小收益幅度，并追加“每个加入优化独立消融，有退化则放弃该项”。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：新增`performance/retain_gains_20261008/PROTOCOL.md`、baseline、geometry/plan.json和METHOD.md、四变体独立源、inventory、assessment新政策与工具；新闭环驱动待建立。生产源码尚未改动。
- 修改内容：冻结当前161份源/文档与控制器、重放器、核心库，155份运行文件与上一轮最终一致；保留原历史10%政策，另立无最小幅度的新实验。A-v2/A-v4/组合分别比较单项及边际收益，存在可复现退化的项不采用。
- 验证：冻结成功，当前二进制与上一轮baseline一致；旧六项模块与final_execution的154份运行源仍一致。新差分、重放与Isaac尚未完成，不宣称本轮已有采用结果。
- 影响与边界：取消收益幅度门槛不取消安全/能力约束；不盲目同时打开已有明确运行退化或未验证条件的功能。P3/P4不改。
- 回滚/备份：`performance/retain_gains_20261008/baseline`，不覆盖20261007原始数据/政策。
- 后续事项：完成逐项与组合消融、实际运行检查、主入口构建和记录，通过项进入生产。

### 2026-10-08 09:25 — [EGO1P5] 整理六个新实现的失败原因与归因边界

- 状态：完成（文档复盘）。
- 目的：应用户要求整理上一轮三个方向、六个实现版本未被采用的原因，区分性能退化、收益不足和因果解释尚缺证据。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`ADAPTIVE_OPTIMIZATION.md`新增“2026-10-08逐项失败原因复盘”；`WORK_LOG.md`及`/home/starry/isaac-data/HISTORY/CHANGELOG.md`。
- 修改内容：逐版整理观测事实、冻结源码支持的机制、未证实假设和下一步应先测量的量。特别澄清A-v2/v4有收益但未达事前10%门槛；B方向连贯性/计算超时共同退化但未做唯一根因分解；C合成收益未传递且没有超时，不能套用B归因。对原几何报告的v1成本和v2大根盒解释标明证据强度。
- 验证：只读核对四版几何汇总、B/C报告、B诊断JSON与冻结node源码；检查新节链接和文档差异。未构建、未运行测试、未启动新仿真或性能实验。所有数值为2026-10-07既有结果。
- 影响与边界：仅文档修改，生产算法/参数和历史原始数据不变；没有宣称已找出唯一根因或完成新的验证。既有报告SHA由2f307e8353baa99ab933792012c08ec0f07968d44de424608fdceacc92990509变为e02a1e07daa29719cafa96751648ad318f3e8aad190a826af131fe1f160139e5，原冻结报告保留在final_restored/sources.tar.gz，未重写冻结哈希或机器成绩。
- 回滚/备份：移除本次新增复盘节即可恢复原报告内容；原报告另有完整冻结副本。日志新增条目保留追溯，不覆盖旧记录。
- 后续事项：无；文中诊断建议未实施，需要新的实验才能验证假设。

### 2026-10-07 23:34 — [EGO1P5] 六个新实现完成消融，均未达保留门槛并恢复生产基线

- 状态：实验与回退完成；本轮无新算法被采用，未宣称运行全面更优。
- 目的：依据前期经验提出新想法、逐项比较，以预先指标决定保留/回退。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`ADAPTIVE_OPTIMIZATION.md`、`tools/run_adaptive_ablation.py`；生产paper_guidance/omni_depth/node/header/check/CMake及两launch恢复基线，goal_refinement两新源移出生产。新增`performance/adaptive_trials_20261007/`内baseline、candidates_bc、preregistration、closed_loop、candidate_geometry/**、geometry_profile/**、candidate_goal/REVIEW.md与analyze_failed_explore.py/failed_explore_diagnosis.json、candidate_manual/REPORT.md、assessment/**、README.md、behavior_rollback.json、completion_summary.json和最终恢复快照；完整几何/评估产物清单分别见candidate_geometry/history_files.json及assessment/final_tools_summary.json。同步本项目README/EGO1P5_VERSION/WORK_LOG及HISTORY README/PROJECT_INDEX/FOV_GVF_FAMILY/CHANGELOG。
- 修改内容：实现A几何预筛/复用四版、B认证切弯、C连续球面细化。四几何版本均未达全组10%P95门槛，B真实超时与多指标退化，C微扰主收益失败提前停止；全部不保留生产改动。保留冻结、交错试验与独立多指标评估工具；新driver在launch缺失已回退实验开关时明确拒绝，避免把两次基线误作消融。CMake安装不会删除旧header，逐SHA核对归档后人工删除/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/include/pc_gvf/goal_refinement.hpp唯一退役头，并记录在behavior_rollback.json。
- 验证：四几何版共20,358,498次查询零差异、740对重放输出一致；5批P95均值变化v1 +26.52%、v2 -7.95%、v3 +33.43%、v4 -2.99%，合并P95变化依次+39.57%、-1.23%、+58.38%、-2.90%，未进入新确认或Isaac。B固定目标基线3/3到达、候选2/3+240s超时；含失败均值69.239→133.383s，实际jerk1.666→4.834，完整callback P958.385→64.153ms。C三对微扰角误差9.200→9.342°，仅1对向好，jerk仅减0.163%，提前停止spin/sweep与确认。12条新Isaac扫掠重叠/外部保护0，但其中目标1次失败，保留未complete manifest。恢复主构建3包18.2s；最终核心31/31（6.20s）、平台15/15、评估工具64/64；新driver退役开关退出2且不创建运行目录，语法检查通过。
- 影响与边界：154份原运行源、控制器/重放器/核心库、安装launch均恢复与冻结基线SHA一致，P3/P4保护136/150份源码一致。默认callback与原六项开关全关不变，无新算法默认启用。回退后未再跑Isaac；采用源/构建二进制一致性及完整回归验证恢复。只测静态Cloud/脚本输入，未真人被试、实飞、动态Isaac或多机飞行。首轮环境收集失败、B超时、C缺场景、几何退化与工具收集范围修正均保留，不删除或重标失败。
- 回滚/备份：完整baseline及candidates_bc源/二进制、四版几何独立源、12条原始轨迹及其条件/哈希、最终恢复快照。未使用git reset，未提交、推送或修改其他项目生产源码。
- 后续事项：无必须完成的本轮实验；所有候选因探索门槛未达而未启动独立确认。进一步算法探索需另立假设和新实验，不把本轮探索重复当确认。

### 2026-10-07 23:18 — [EGO1P5] 两个行为新方案实跑退化并精确回退

- 状态：B目标切弯失败并回退；C连续评分主收益失败提前停止并回退；A-v3仍在独立计时。
- 目的：执行“更优保留、变差回退”，不以合成方向平滑收益替代整机结果。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`src/pc_gvf/src/depth_angular_controller_node.cpp`、`omni_depth.cpp`、`include/pc_gvf/omni_depth.hpp`、`test/cpp/omni_depth_check.cpp`、`CMakeLists.txt`、两个launch恢复本轮baseline；移除生产新增goal_refinement头与check（已归档）；`performance/adaptive_trials_20261007/behavior_rollback.json`、assessment采集/评估与candidate_goal/REVIEW.md。
- 修改内容：B/C全部生产逻辑和实验参数恢复冻结基线，逐文件SHA核对后还原7份、归档移除2份。不改变上一轮算法、绑定样本执行源或物理参数；独立driver/评估工具继续保留。
- 验证：本轮12条新Isaac轨迹，扫掠重叠与外部保护均0。B基线3/3到达、候选2/3到达，第三条240s超时；含超时均值69.238889→133.383333s，实际jerk1.666085→4.834317，完整callback P95 8.384667→64.152667ms，正式工具REJECT。C微扰3对角均9.200178→9.341946°，仅1对向好；jerk4.469191→4.461913仅约0.16%，未达预登记角误差主收益。C正式工具INSUFFICIENT（spin/sweep未跑），按已有主收益失败提前回退，不冒称完成全协议。原B失败manifest.complete=false原样保留。
- 影响与边界：上述是静态Cloud固定目标和脚本微扰，非动态/真人实飞；原始数据、源码、二进制、采集前launch/源一致性证明全部保留。A-v3仅独立副本，2710528几何查询首个证据及bool均0差异，性能尚未判定。
- 回滚/备份：`baseline/sources.tar.gz`及`candidates_bc/sources.tar.gz`；源码回退完成，安装产物重建待几何计时结束，避免污染性能实验。未使用git reset或修改P3/P4。
- 后续事项：完成A-v3计时决策后主入口重建、回归和最终状态记录。

### 2026-10-07 23:02 — [EGO1P5] 回退未达标几何候选，开始两个行为方案交错对照

- 状态：进行中；几何v1/v2未采用，B/C实跑验收进行中。
- 目的：执行用户“更优保留、变差回退”的要求，避免局部指标掩盖整体退化。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`paper_guidance.cpp`恢复本轮基线；`omni_depth.cpp/hpp`、`omni_depth_check.cpp`实现可选连续评分；固定目标细化相关node/header/check/launch、新driver；新增`performance/adaptive_trials_20261007/assessment/`评估工具、policy/模板/回归与证据，ADAPTIVE_OPTIMIZATION.md。
- 修改内容：A-v1逐查询胶囊AABB预筛、A-v2递归根盒管列表均冻结后回退/未采用。B最多3次认证切弯、C插值评分+有证据局部细化默认均0。新driver每个目标也单独保存全回调perf，通过唯一主入口运行；冻结相同B/C控制器，0/1或0/3显式消融。
- 验证：A-v1/v2各185对输出一致，边界差分约480万/704万一致；v1继承P95+43.2%，v2全组P95仅-1.2%，按预登记门槛不保留。B/C主构建3包23.0s，C关闭开关96/96一致，完整C++检查通过；初次CTest30/32后Python隔离2/2通过（原NumPy/SciPy收集错误保留）。平台两次路径收集失败后source ROS+项目15/15；无环境依赖修改。assessment46/46通过；新Isaac尚无优胜结论。
- 影响与边界：生产几何源码与本轮基线字节一致，行为新开关默认关闭；不降低认证或物理限制。几何采样/编译/CPU基准均暂停，避免干扰闭环比较。
- 回滚/备份：baseline、candidate_geometry各失败源码/微基准、candidates_bc冻结162份源及二进制；不删除原始失败数据，未提交推送。
- 后续事项：完成B/C三对探索，达标才进新五对确认；几何下一分层列表仅在独立副本设计，未经性能通过不落生产。

### 2026-10-07 22:47 — [EGO1P5] 新想法实验：冻结当前默认并预登记保留/回退判据

- 状态：进行中，尚无本轮优胜结论。
- 目的：按用户要求大胆提出新方案，多指标前后对照，更优保留、变差回退。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`ADAPTIVE_OPTIMIZATION.md`、`tools/run_adaptive_ablation.py`、`src/pc_gvf/include/pc_gvf/goal_refinement.hpp`、`src/pc_gvf/test/cpp/goal_refinement_check.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、两项launch、`src/pc_gvf/CMakeLists.txt`；并行候选将涉及paper_guidance.cpp、omni_depth.cpp/hpp及相应回归，新assessment工具均独立归档。
- 修改内容：冻结159份源及运行二进制，默认callback/六算法关闭；提出胶囊包围盒精确预筛、认证切弯、连续球面评分三候选，新增显式实验开关，默认均关闭。预定安全硬门、目标/jerk/停车/路径/尾延迟容差及3对探索+5对新确认规则。
- 验证：固定目标方向细化独立C++回归已通过，覆盖全部拒绝、非单调可见域、接缝、预算、无效输入；新Python驱动/launch语法通过。主构建与新Isaac实验尚未运行。perf采样权限拒绝后改用独立gprof，未修改系统权限；热点是旧管段距离计算，完整性能结论待测。
- 影响与边界：不降低安全半径、不改变物理模型；几何预筛不得改变原判定，行为新候选仍需完整运动/制动证明。P3/P4不改。
- 回滚/备份：`performance/adaptive_trials_20261007/baseline`；生产退化候选会恢复，失败源码/日志保留，不用git reset，未提交推送。
- 后续事项：完成候选回归、逐项冻结对照与独立确认，按预登记门槛选择保留/回退。

### 2026-10-07 22:31 — [EGO1P5] 六方向继续优化、执行链修复与最终消融验收

- 状态：部分完成；六项实现和逐项模块收益验证完成，整体全面优胜仍未达到，默认六算法开关保持关闭。
- 目的：延续上一轮，降低六项机制开销、补安全边界，并用冻结旧/新消融验证；处理真实操控执行向量/时间戳错位。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`src/pc_gvf/src/{depth_uncertainty,dynamic_obstacles,paper_guidance,spherical_memory,shared_obstacles,paper_replay}.cpp`、`include/pc_gvf/{operator_intent,shared_obstacles}.hpp`及对应`test/cpp`回归/`operator_lag_fixture.hpp`；两项launch可执行路径/SHA记录；`scripts/isaac/{run_fov_gvf_navigation,manual_control_math}.py`、`test/test_manual_control_math.py`、`tools/{run_refinement_ablation,analyze_execution_samples}.py`；README、EGO1P5_VERSION、SIX_AXIS_OPTIMIZATION、SIX_AXIS_REFINEMENT、WORK_LOG及本轮performance证据。同步HISTORY/README、PROJECT_INDEX、FOV_GVF_FAMILY与本日志；RECORDING_RULES未改。 完整24份运行源码逐文件列表见`EGO1P5/performance/six_axis_refinement_20261007/final_source_changes.json`。
- 修改内容：等价可分离遮挡最小值；带小集合直扫的精确动态关联索引；SE(3)暖解及统一绝对残差；球面缓存保守块裁剪；同低频延迟的双级小角往复滤波；共享占据包络及第九危险来源容量停机、M格式重放。修复大世界坐标/时间消去和最终角滞后边界反例。执行命令绑定vector/receipt/sequence，默认由omnigraph切到callback；六算法独立默认仍0，安全半径/速度/响应/地图不改。
- 验证：主构建3包成功、核心31/31、平台15/15、9机制/64组合+容量锁重放通过；21继承/16卡停快照旧布局兼容。执行样本Python39/39（新增23项）、针对性CTest1/1；Python语法及diff检查通过。模块收益见报告/机器汇总。本轮34条Isaac=10探索+12算法正式+12执行源正式；18目标安全到达，16人工等效轨迹全部扫掠重叠/外部保护0，24正式实际可执行SHA核对通过。首组目标69.239→69.867s、人工jerk4.513→4.553，失败保留；执行源手柄jerk4.532→4.463、停车1.033→.967s，目标均值71.322→70.756s但标准差.415→1.550s/停车.250→.311s退化。ROS共享606样本危险期.5→0、解除恢复.493，双节点各接纳/拒绝92包；合成机位非飞行。P3 136/136、P4 150/150保护哈希匹配。
- 影响与边界：条件性计算收益不等于整机全指标优胜；输入滤波本身的真实jerk退化仍成立，执行链收益另计。默认callback基于同样本正确性及平均轨迹收益，目标波动/停车代价明确保留。Twist接收年龄非端到端年龄。关闭阶段一次命令桥-11及rosout context-invalid记录保留。动态Isaac、多机实飞、真实窄FOV多场景、标定及人类被试未运行。
- 回滚/备份：本轮`performance/six_axis_refinement_20261007/baseline`冻结旧源码/二进制；`final`冻结首批正式版本；`final_execution`冻结154份最终运行文件与二进制引用。设`ISAAC_TWIST_SAMPLE_SOURCE=omnigraph`恢复旧执行源，六算法以独立环境开关启用；未删除原失败数据，未提交推送。
- 后续事项：总体优胜仍未验收；后续应针对目标波动/计算尾延迟、真实动态/窄FOV/多机工况继续验证，不可直接宣称全开最优。

### 2026-10-07 22:14 — [EGO1P5] 六方向正式闭环退化记录与执行样本一致性后续

- 状态：部分完成；六项模块消融完成，正式整机全面优胜未通过；执行源独立消融进行中。
- 目的：保留真实退化，修复命令向量与接收时间来自两个订阅的执行不一致。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`manual_control_math.py`、`src/pc_gvf/test/test_manual_control_math.py`、`tools/run_refinement_ablation.py`、`SIX_AXIS_REFINEMENT.md`及性能证据。
- 修改内容：新增绑定vector/receipt/sequence的TimedVelocitySample；callback与原OmniGraph源显式切换，默认暂保留omnigraph；CSV记录接收/应用序号和向量。松手立即零目标，.10s超时、.22s响应、1.2m/s²及几何认证不变。
- 验证：本轮首批正式12条轨迹均扫掠重叠/外部保护0；目标旧/新均值69.238889/69.866667s，手柄实际jerk4.513039/4.553247，全面优胜失败。首轮旧目标到达后命令桥退出-11保留。Python辅助测试39/39通过（新增23项）；语法检查通过。共享合成ROS危险101样本指令.5→0，清除后恢复.493117；两节点各接受正确坐标92份、拒绝错误92份。
- 影响与边界：新鲜度只表征本机接收年龄，非端到端时延；同冻结控制器交错源切换目标/手柄各三对正在执行，未提前判胜。动态Isaac、多机实飞、人类被试未运行。
- 回滚/备份：本轮旧源码/二进制在`performance/six_axis_refinement_20261007/baseline`，六模块冻结在`final`，原OmniGraph路径保留可复现；前期失败不删除。
- 后续事项：完成执行源消融，更新最终验收与默认选择。

### 2026-10-07 21:43 — [EGO1P5] 六方向继续优化集成通过，正式闭环进行中

- 状态：构建与回归完成；正式前后交错闭环进行中，未判定整机优胜。
- 目的：将逐项改进与审查边界修复集成后，使用真实运行版本对照验证。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：六模块、paper_replay、对应check及operator_lag_fixture.hpp；两个launch实验入口、tools/run_refinement_ablation.py；SIX_AXIS_REFINEMENT.md及本轮performance目录。
- 修改内容：独立审查复现并修复共享包络两种浮点时间边界、球面大世界坐标包围盒低估、双级滤波最终输出角滞后轻微超限。共享容量锁保存为M格式；旧L及之前继续兼容。增加冻结二进制选择/启动SHA实证/显式安全审计/清理实验环境变量的交错工具，日常默认参数不变。
- 验证：主构建3包成功24.2s；核心31/31、平台15/15、9机制检查及64组合+容量锁重放通过。增量40个新边界/遮挡/小rhs/坏初值/位姿案例最大冷暖势差3.921e-9。21继承快照、16真实卡停快照旧决定完全一致；21帧增量平均141.525→141.971ms无收益，16帧159.077→5.353ms为已有增量复用能力复验，不能作为本轮新提速。正式目标首轮启动参数确认实际旧二进制SHA=1de6792ae5e85e2e90561359173b54f827324e81c1cdf15c74b441966f04aa6f。
- 影响与边界：最终六机制源及二进制冻结在final，静态Cloud固定目标/脚本手柄将交错对照；尚未做本轮动态Isaac、多机实飞或人类被试。旧版探索6目标+4手柄全部保留，正式组不与其混算。
- 回滚/备份：baseline及final源包/二进制/哈希，四个审查反例与修前修后探针，逐阶段早期退化全部保留。沙箱补丁读取同样mountinfo错误，改用已授权工作区脚本编辑；无提交推送。
- 后续事项：完成正式交错目标/手柄，比较安全、时延、进展、停车和jerk，再根据证据选择默认并更新最终结论。

### 2026-10-07 21:25 — [EGO1P5] 六方向继续优化：冻结基线与机制改进进行中

- 状态：进行中，尚无本轮整机优胜结论。
- 目的：针对上一轮条件收益、重复计算及操控jerk退化继续实现与逐项消融。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：深度不确定性、动态障碍、球面记忆、人工意图、共享占据、调和场与重放源码及对应测试；`performance/six_axis_refinement_20261007/`。
- 修改内容：①等价可分离遮挡最小值；②精确关联候选索引；③与冷解相同绝对残差目标及同位姿直接重投影；④原生命中块级保守排除；⑤反转噪声处理；⑥每源保守包络与有效超容量消息的短时停止保护。新保护状态用M快照，向后兼容旧格式。后三项阶段实现/验证仍在进行，不写成已通过。
- 验证：修改前3轮六项关闭均ARRIVED，69.250/71.033/69.367秒，旧二进制③⑤开启3轮继续执行；①3,416,657像素旧新位级一致，默认半径1四视图处理减少39.8–45.6%，ASan/UBSan定向检查通过；③7组成对100帧迭代7119→5369、耗时中位69.5965→46.5391ms、对冷解最大势差2.517e-9，未知拒绝通过。本轮主构建、最终集成及新Isaac运行尚未执行。
- 影响与边界：六项启动默认均关闭，.48/.58m机体/认证及动力学参数不变；计算收益不等于导航时间收益，保持旧失败。沙箱exec因bubblewrap mountinfo错误未启动，后续通过相同目录授权执行；未改P3/P4。
- 回滚/备份：本轮baseline含155份源码/文档SHA、源包、旧控制器/重放器/核心静态库/机制二进制及P3/P4源SHA。
- 后续事项：完成④⑤⑥消融，主构建/核心/平台/重放验证，再做冻结前后Isaac对照与文档收尾。

### 2026-10-07 17:38 — [EGO1P5] 六方向最终归档与验收结论

- 状态：六项方案、实现和逐项消融完成；整体运行全面优于修改前的要求未通过，默认保留实测基线，六项为可选能力。
- 目的：完成最终验证、冻结源码和二进制，给出可复现结果及可直接使用的手柄命令。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：六模块及控制器/launch/消息/测试共38项运行源差异；SIX_AXIS_OPTIMIZATION.md、README.md、EGO1P5_VERSION.md、WORK_LOG.md；performance/six_axis_20261007下汇总/图/冻结/原始实验；统一HISTORY的README/FOV_GVF_FAMILY/PROJECT_INDEX/CHANGELOG。
- 修改内容：保持原四相机全向候选、7层/1024预算联合完整证明与.48/.58m包络；新增原生深度置信下界、动态相对扫掠、SE(3)残差增量场、有限期球面原证据、人工小角EMA与连续转向直通、带来源和时间约束的共享占据。最终六开关均关闭，参数与实测v2基线一致；动态开启时≤.1s采样/≥32容量修复已通过新回归。文档区分复制历史、各阶段真实证据和未验证事项。
- 验证：主入口最终构建成功，CTest31/31；9个机制检查全过、64/64组合重放；平台15/15（平台之后未修改）。最终21继承状态关闭新机制与原P5二进制决定完全一致；增量均值146.939→147.388ms无收益，最大约2.45s，未隐瞒。P5新16卡停帧×3配对158.452→5.289ms，48/48均接受。全量机器汇总18/18目标安全到达、12/12人工轨迹安全，30条完整扫掠重叠0、外部保护0。v2基线/③⑤各3轮69.256/70.039s、jerk1.695/1.859；手柄各2轮方向误差9.419→9.393°而jerk4.534→4.562，整体优胜未通过。ROS共享危险使独立.5/共享0，解除恢复约.493；全开两节点各92接收/92错frame拒绝，均为固定合成机位。51份Python AST、3份Shell语法、git diff --check、报告本地链接通过。P3的125份、P4的139份受保护源SHA均未改。
- 影响与边界：噪声错误机体认证8→0；动态解析闭环碰撞5→0（不是30条Isaac之一）；窄FOV已见方向0→74；人工解析抖动TV约-91.1%；同伴危险32→0均为限定场景主指标。未做传感器标定、动态Isaac、多机实飞或人类被试。新增超时前端捷径未在3次v2组合运行触发，不声称整链路CPU提速已证实。无提交推送，未启动持续人工驾驶。
- 回滚/备份：baseline、逐阶段、final第一版、frontend_followup第二版均保留；最终accepted_optional冻结151份运行源、两二进制及SHA，归档5份ROS原始日志；测试初次ABI/环境失败、模拟退化和完整原始结果均保留。所有实验命令保持唯一标签，不覆盖既有结果。
- 后续事项：当前可直接通过主入口手柄运行基线，报告给出六项开关和逐项复现命令。整体优胜仍需针对动态/噪声/盲区实际场景和更大样本验证；共享自由空间/SLAM与自动坐标配准不属于本次实现。


### 2026-10-07 — [EGO1P5] 第二版30轮总验收完成，保留可选机制，默认回到已测基线配置

- 状态：六项实现与机制消融完成；用户要求的整体运行更优未通过，不能宣称全面成功。
- 目的：根据真实闭环全量结果选择默认，避免用单项理论/离线收益替代整机收益。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：launch/isaac_cloud_navigation.launch.py、paper_guidance.cpp、dynamic_obstacles_check.cpp；报告和最终汇总目录accepted_optional。
- 修改内容：六个新机制默认均关闭，保留独立开关及已验证场景的可选能力；FOV_GVF_INCREMENTAL_FRONTEND只有③开启且超时后才可能生效。复查动态状态发现静态1s采样大于动态.5sTTL，改为动态模式≤.1s采样、至少32份容量，并补充证据不断档检查；不改变默认静态模式。
- 验证：第二版基线3/3安全到达69.256s，③⑤3/3安全到达70.039s，停车均.250，jerk1.695→1.859；第二版操作各2轮安全，角误差9.419→9.393°、进展16.449→16.454m，jerk4.534→4.562仍略退。前端捷径本批3轮未实际触发，因此仅资格/当前证明回归通过，不声称真实CPU捷径速度已验收。①③旧组70.642s明显改善①单开125.933s；作为条件收益保留。两版本与各阶段累计30条Isaac轨迹安全审计0/保护0，确切汇总待最终脚本核验。
- 影响与边界：六项各自机制目标有可复现收益，但理想静态Cloud不应一键全开；动态/窄FOV/共享试验分别为解析、几何、ROS合成证据，未做动态Isaac、真实多机飞行或人类被试。默认配置就是本任务已实测的all-off基线，无新增强制算法介入。
- 回滚/备份：baseline、各阶段、第一版final和第二版frontend_followup冻结及全部退化保留；无提交推送，P3/P4不改。
- 后续事项：验证动态采样补丁、最终源码/参数/安全汇总及文档；使用者可按已标定噪声、动态模型或短期盲区需求选择相应开关。


### 2026-10-07 — [EGO1P5] 冻结实跑发现③⑤退化，按原因修正前端恢复与连续转向

- 状态：首批最终冻结14轮已完成，全部安全；整体性能不通过，继续修正而非采纳最好一轮。
- 目的：解决增量核心之外的全向搜索导致的计算截止，以及角保持将慢转变为阶跃的问题。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：paper_guidance.hpp/.cpp、controller、operator_intent.hpp、launch、incremental_field_check/operator_intent_check、ros_shared_obstacles_probe.py；frontend_followup准备脚本/拟议补丁。
- 修改内容：仅在上一周期COMPUTE_DEADLINE且旧场目标仍满足.5s/意图资格时，先跳过重复前端候选，再做完整当前运动/制动认证；松手/失效清恢复状态，可由FOV_GVF_INCREMENTAL_FRONTEND关闭对照。人工小角噪声用球面归一化低通，三次同向变化识别连续转向直接通过，大转向/松手保持零附加帧延迟，角误差限.018rad。共享全开负探针max_depth_age从.5改.3以与.58包络一致（此前.60负例记录保留）。
- 验证：冻结目标baseline两轮71.017/69.533s，③⑤69.250/86.650s，其中第二轮停车12.517s；球面70.950/71.033s两轮。人工抖动开关各2轮均安全，但初版⑤jerk4.532→4.557、投影进展16.460→16.452、误差9.343→9.541°，慢转相位RMS .0288→.0338，判为退化。sphere恢复单对进展82.602→82.479、jerk3.829→3.961，未获整机收益。回退机制实验和所有完整轨迹保留。
- 影响与边界：新修正尚待构建/同样消融，不写通过。仅优化候选计算调度和人工微扰处理，当前完整证据证明与物理安全参数不变。临时预览脚本曾因exec共享变量导致写补丁目标路径错误，实际运行源未改，已修正预览输出。
- 回滚/备份：final/sources.tar.gz及冻结二进制记录第一版；新实验独立frontend_followup标签，不覆盖旧组。
- 后续事项：31项回归、操控/资格消融，复跑冻结目标及手柄；不满足收益则降为可选而非默认。


### 2026-10-07 — [EGO1P5] 六项机制逐项验收完成，最终组合冻结前检查

- 状态：31/31回归、⑥32例危险放行32→0、16种内部状态组合重放、三真实ROS进程共享命令对照通过；最终组合实跑待完成。
- 目的：在P5全向链上确认六项均可独立开关，冻结前补全外部预处理组合与默认候选。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：paper_guidance.cpp、incremental_field_check.cpp、six_axis_replay_check.cpp、launch；ROS共享/全向探针、最终实验目录。
- 修改内容：增量求解每次先清有效位，防止调用者传入旧valid状态在全阻塞域提前返回；扩展含深度预处理及人工输入的64组合重放。候选默认设③⑤开，①②④⑥关。ROS探针补齐.48/.58与1.2动力学参数，先前未显式指定的全开负探针半径.605仅作为消息/未知拒绝证据，不冒称原Cloud参数。
- 验证：ROS共享正向对照使用.48/.58/1.2，clear两节点平均命令.4991，peer_hazard本机独立.5/共享0，cleared共享.4931；606样本，危险解除恢复。全开负探针两机器人各92接收/92错坐标拒绝，P5 --omni缺前相机/未知/过期/松手检查通过；均非飞行。31/31在新增有效位和64组合补充前通过，补充后即将重跑。
- 影响与边界：尚无全面导航优胜结论；后续固定目标、操作抖动与球面恢复均使用冻结二进制和原场景，逐项保存不利指标。
- 回滚/备份：各阶段src快照、基线二进制、原ROS消息日志与失败全部保留，P3/P4源码哈希最终复核。
- 后续事项：最终构建、64组合、平台回归与继承重放；固定目标/人工轨迹成对比较，更新报告、HISTORY与可直接运行的手柄指令。


### 2026-10-07 — [EGO1P5] ⑤操控消融通过，⑥多机器人共享占据接入

- 状态：⑤29/29及闭环微扰消融通过；⑥实现完成待构建与ROS验证。
- 目的：允许对齐同坐标系的机器人共同维护短时占据证据，补充各机独立观测的遮挡危险。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：paper_guidance.cpp、shared_obstacles.cpp、controller、launch、SharedObstacles.msg、两包CMake、shared_obstacles_check/six_axis_replay_check、tools/ros_shared_control_probe.py、ros_shared_obstacles_probe.py。
- 修改内容：按robot/session/sequence/time融合占据和速度；错坐标、未来/陈旧、乱序、超量/异常消息拒绝，最大8源，消息保留原观察时间，不转发他人证据冒充自己。补充过期源回收，避免8个离开机器人永久占槽。共享危险作用于完整运动/制动段，本机未知仍拒绝。
- 验证：⑤指令总变化.241851→.000104623，前进11.3515→11.3518m，松手/大转向附加延迟0帧；累计慢转误差不超过.018rad，29/29通过。此为生产核心解析闭环；真实Isaac等效输入将在最终冻结版本成对执行。
- 影响与边界：提供共同占据地图，不包含多机SLAM/自动坐标配准或共享自由空间授权。时钟和world对齐、独立robot_id、传感器/控制话题隔离是使用条件；无真实多机飞行验收。
- 回滚/备份：stage5_operator/sources.tar.gz及全部先前阶段；P3/P4保持独立。
- 后续事项：⑥31项回归、32例遮挡危险开关消融、三真实ROS进程命令对照、全开组合拒绝；冻结后做目标/手柄完整轨迹及文档收尾。


### 2026-10-07 — [EGO1P5] ④盲区证据消融通过，⑤操控角滞回接入

- 状态：④28/28及两个消融通过，⑤待构建。
- 目的：保留松手/大转向即时响应的前提下，减少微小手抖反复扰动三维避障指令。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：operator_intent.hpp、controller、launch、CMake、operator_intent_check；isaac_operator_jitter_trace.json、tools/run_six_axis_manual.py、analyze_omni_comparison.py。
- 修改内容：人工输入相对于最后接受输入方向使用.018rad角滞回，速度幅值保持，松手清空，大改向即响应，慢转累计过阈值后更新；固定目标不经过该滤波。诊断输出启用状态和输入改变量。新增同二进制交替重复手柄等效消融与独立整段扫掠审计入口，原平台/场景不变。
- 验证：④74个窄FOV已见方向认证0→74，过期放行0，矛盾借用引用撤销通过；原生76800点×80帧维护预筛选与精确基线保留证据一致。①③Cloud两轮71.567/69.717s安全到达，平均70.642s，较①单开125.933s恢复，但仍比初始基线70.142s慢，完整扫掠0/保护0。
- 影响与边界：球面74例为三维证据几何测试，不是74次真实飛行。操控测试无真实人类被试，不声称主观满意度提升。
- 回滚/备份：stage4_spherical/sources.tar.gz及此前阶段冻结；全部旧失败保留。
- 后续事项：⑤指令抖动/慢转/松手回归，再完成⑥共享占据ROS控制消融及最终组合实跑。


### 2026-10-07 — [EGO1P5] ③快照耗时显著降低，④球面记忆接入

- 状态：③26/26回归及冷暖解消融通过；④实现完成待构建。
- 目的：利用短期已观测的侧后方向，解决窄FOV转头后仅因历史采样稀疏失去证据的问题。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：paper_guidance.cpp、controller、CMake、spherical_memory_check/spherical_ingest_check；tools/compare_six_axis_replays.py。
- 修改内容：24×12世界球面索引、最多48份原始观测出处、4s寿命和.05m/s年龄扩张；动态模式有效寿命.5s。保留相机原视锥和命中矛盾撤销，像素预筛选只排除确定不矛盾点。P5仅在当前统一候选缺失时查询记忆，避免覆盖有效四相机提案；未知仍不能由记忆插值授权。
- 验证：③100帧冷解17450→7119次迭代，最大势误差1.695e-9；本次P5的16卡停帧×3交替对照，平均158.452→5.289ms，最大173.803→6.585ms（含进程启动），两侧48/48核心接受，关闭增量与修改前决定完全一致。①③两轮Cloud验证正在执行，不由快照推断实际闭环结论。
- 影响与边界：球面记忆是有限期有出处证据，不是无盲区传感器或永久地图；P3/P4未改。
- 回滚/备份：stage3_incremental/sources.tar.gz与二进制，所有失败保留。
- 后续事项：④窄FOV已见方向/过期/矛盾消融、维护开销及Cloud组合检验；再接入⑤。


### 2026-10-07 — [EGO1P5] ②验收通过，③自运动补偿增量调和场接入

- 状态：②25/25与两项动态消融通过；③待构建。
- 目的：减少相邻帧重复冷解及实跑出现的长时计算截止停车。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：paper_guidance.cpp、controller、launch、CMake、incremental_field_check；intent_response_check的重放魔数I→L；tools/run_six_axis_ablation.py与SIX_AXIS_OPTIMIZATION.md。
- 修改内容：旧场按采集位姿旋转/平移投影，解Aδ=b-Aφ；残差异常退回冷解。无新提案时短时复用旧场世界目标，但当前完整运动证明必须通过，累计意图改向会失效。保留P5的7层完整包含证明。③初始默认关闭，等待验收。
- 验证：②初次因copy2旧mtime导致测试与库ABI不同而失败，强制更新时间重编译后仅旧魔数断言失败；修正格式预期后25/25，行为断言不改。动态穿越40危险放行→0；解析闭环5碰撞→0，空场和离开障碍负控制通过。①Cloud两轮均安全到达、扫掠0，但平均125.933s较原70.142s明显退化，平均停车54.458s，不能默认开启。失败和退化日志全保留。
- 影响与边界：动态闭环为生产核心+解析深度渲染+同动力学，非Isaac动态场景或真机。增量场仅提案，不复用旧几何授权。
- 回滚/备份：stage2_dynamic/sources.tar.gz，baseline及stage1冻结保留；P3/P4未改。
- 后续事项：③26项回归、冷/暖解及本次P5原生16帧卡停快照消融，再接入④。


### 2026-10-07 16:27 — [EGO1P5] 完整复制检查点验收完成，保留并行后续优化

- 状态：用户要求的完整复制、原版本保留、算法/进展/状态说明完成；构建与既有回归通过，后续六方向优化属于另一项进行中的工作。
- 目的：在EGO1P5建立可追溯的新副本并准确描述来源状态，不覆盖EGO1P3或混淆后续测试范围。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：全部3978文件/4467条目复制；scripts/build_isaac_ros_workspace.sh、scripts/run_isaac_fov_gvf_navigation.sh、src/pc_gvf/launch/navigation_benchmark.launch.py、tools/run_navigation_benchmark.py、analyze_navigation_benchmark.py、run_omni_comparison.py、verify_navigation_replays.py、ros_vector_guidance_probe.py；新EGO1P5_VERSION.md、EGO1P3_README.md及README/AGENTS/WORK_LOG；performance/copy_from_ego1p3_20261007证据；HISTORY/README、PROJECT_INDEX、FOV_GVF_FAMILY、CHANGELOG。
- 修改内容：复制包括隐藏文件、Git、全部未提交内容、场景、成功/失败实验和最新手柄日志。仅8项运行文件适配P5根目录、独立构建/安装/日志/重放和实验身份，保留P3作为显式对照项；复制检查点117份其他运行源与v22一致，算法/参数/场景/桥/手柄映射不改。来源README另存原文，旧验收及冻结包不改标签。
- 本次实际验证：主Shell构建3包成功（16:15:34日志结束）；CTest22/22（16:17:31结束）、平台15/15；45份Python AST、3份Shell语法通过。配置构造检查解析到/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf及其控制器，360°开启；未启动节点或仿真。复制前后内容/元数据SHA一致，无共享普通文件inode，最终16:24核验EGO1P3原3978文件/4467条目无变化或新增缺失。
- 并行边界：检查发现另一个“六方向优化”工作已在新P5冻结baseline并运行试验，随后约16:20起修改CMake、控制节点、默认launch并新增不确定性模块。其.git/index、公共性能日志、新源码与试验全部保留，未回滚/覆盖。另一个任务的NumPy2首轮失败及后续修正记录属于该任务；本复制任务在PYTHONNOUSERSITE=1环境下的22/22与15/15结果独立归档。
- 证据：copy_verification.json与source_manifest.json证明适配前完整复制；final_copy_integrity.json证明原P3未变并列出观察到的后续运行源变化。copy_checkpoint_runtime.tar.gz从保留P3加本次8项确定性适配重建125份源，117份未适配源与后续任务所存基线交叉核对一致；不是当前正在变化的工作树快照。build.log、ctest.log、platform_tests.log、syntax_checks.json、configuration_check.json及复制摘要均位于本次独立证据目录。
- 来源状态说明：v22连续扫向明显优于P2但全面优胜未通过，独立旋转和目标部分退化，正式24轮和旧失败全部保留。新读取到来源20261007_160119手柄模式147.5s仿真USER_EXIT、外部保护0、avoid峰值633.340ms，未重演/新审计；不能把长指令不变间隔直接认作卡停，也不能把它改标为P5新成绩。
- 影响与边界：EGO1P3原样保留，EGO1P2/EGO1P4未修改；P5继续共用ROS域42和单实例锁。版本说明将复制检查点与随后六方向工作明确分开，当前源码是否通过以其后续WORK_LOG为准。
- 回滚/备份：原EGO1P3完整保留；新目录复制清单、纯路径适配运行源包及后续任务baseline均保留。Git分支名仍继承ego1p1；本复制任务未提交或推送，未启动新导航/手柄动态验收。
- 后续事项：本次复制请求无剩余事项；后续算法优化及其验收由正在进行的独立任务记录，不将其未完成状态写成本复制失败或测试通过。


### 2026-10-07 — [EGO1P5] ②动态障碍接入，统一可重放扩展状态

- 状态：第一项23/23回归通过；第二项接入完成，待构建消融。
- 目的：把障碍运动纳入预测制动约束，避免仅凭当前静态图放行穿越轨迹。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：paper_guidance.hpp/.cpp、dynamic_obstacles.hpp/.cpp、paper_replay.cpp及重放工具、controller、CMake、dynamic_obstacles_check/dynamic_closed_loop_check；spherical_memory/shared_obstacles状态模块预接入序列化，尚未接入控制流程。
- 修改内容：复用P4世界坐标稀疏跟踪与相对运动段检验，速度/加速度误差界，数据超量拒绝；动态开关关闭长期静态自由体积，历史限制.5s且扩张不确定性。P5的四相机候选及7层完整包含保留。扩展重放至兼容P4的L格式，为后续增量/球面/共享状态留出相同编码。
- 验证：第一项1000样本深度高估488→0、虚假机体放行8→0；原生扩张前景28格对粗图42格，23/23通过。原Cloud不确定性两轮正在运行，尚未判定导航收益。
- 影响与边界：动态模型不保证不可观测/超速度界对象；最近邻关联有误差，动态闭环待执行。其他四项目前无控制行为改变。
- 回滚/备份：stage1_uncertainty/sources.tar.gz及二进制，原始baseline保留。
- 后续事项：动态解析闭环、运动补偿误报检查，再逐项启用③④⑤⑥。


### 2026-10-07 — [EGO1P5] ①原生分辨率深度不确定性实现

- 状态：实现完成，阶段构建/消融进行中。
- 目的：将深度量测距离误差、位姿界和遮挡/无效像素纳入自由空间下界，减轻旧P4粗分辨率形态扩张的额外保守性。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：src/pc_gvf/include/pc_gvf/depth_uncertainty.hpp、src/depth_uncertainty.cpp、depth_angular_controller_node.cpp、launch/isaac_cloud_navigation.launch.py、CMakeLists.txt、test/cpp/depth_uncertainty_check.cpp。
- 修改内容：移植P4误差模型并改为原生深度先处理再保守降采样；sigma=.005+.001*z²，3sigma+.01m，抵扣原包络已计入的.03m径向界；边缘只扩原生1像素，未知不填补。原始命中保持用于矛盾撤销；默认关闭，须按传感器标定参数。
- 验证：基线CTest首轮20/22，用户NumPy2污染导致2项Python ABI失败；加PYTHONNOUSERSITE=1后22/22。第一轮基线目标69.267s到达，第二轮进行中，完整几何审计待执行。
- 影响与边界：模型为参数化下界，尚未实传感器标定，不宣称3sigma绝对保证或未知变自由。
- 回滚/备份：baseline冻结完整源及二进制，功能可由FOV_GVF_DEPTH_UNCERTAINTY关闭。
- 后续事项：构建与固定种子噪声/轮廓消融、真实Cloud开关比较；未通过收益不得默认开启。


### 2026-10-07 — [EGO1P5] 六方向优化启动，冻结当前v22副本为独立基线

- 状态：读取源码与历史完成，实现和新实验进行中。
- 目的：按用户六项要求逐项实现、消融，保留所有退化，以本目录当前状态比较。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：`performance/six_axis_20261007/baseline/`；后续各阶段追加精确范围。
- 修改内容：冻结当前src/scripts/tools及根文档、控制器/重放二进制、launch；保存P3/P4源码哈希。P5已完成独立路径适配，本任务不再重复改名或重标历史成绩。
- 验证：读取复制构建日志成功；本任务核心回归和动态基线待运行，尚无P5优化收益结论。
- 影响与边界：保留四相机融合、7层/1024预算联合证明、.48机体/.58认证及真实1.2加减速度。不假定六项全开就全面优胜。
- 回滚/备份：本目录`performance/six_axis_20261007/baseline/sources.tar.gz`和SHA清单；不修改P3/P4。
- 后续事项：深度下界与遮挡建模、动态扫掠约束、场增量更新、球面历史证据、操控滞回、共享占据，逐阶段构建与开关消融，再做组合闭环与完整扫掠审计。


### 2026-10-07 16:15 — [EGO1P5] 完整复制EGO1P3并隔离路径，构建验证进行中

- 状态：完整复制及来源一致性核验完成，独立构建/回归进行中。
- 目的：按用户要求保留EGO1P3全部内容，并在新EGO1P5说明代码、算法、进展与已知边界。
- 项目路径：`/home/starry/isaac-data/EGO1P5`。
- 涉及文件：全部复制3978文件/4467条目；适配scripts两个主入口、navigation_benchmark.launch.py及5个tools脚本；新增EGO1P5_VERSION.md、EGO1P3_README.md，更新README.md、AGENTS.md、WORK_LOG.md；HISTORY/CHANGELOG.md。
- 修改内容：只将默认工程根、构建/安装/日志/重放前缀改为EGO1P5/ego1p5，实验工具增加P5身份并保留P3比较项；算法C++、默认导航参数、场景、桥与手柄映射不改。旧报告和冻结包不改标签；Git和全部未提交内容继承，不创建提交/推送。
- 验证：cp -a --reflink=auto复制前后逐文件SHA256和mode/uid/gid/mtime一致，源目录未变，无共享普通文件inode；来源125份运行源与v22冻结清单一致。独立构建/回归待执行，未运行新导航/手柄动态验收。
- 来源状态：v22连续扫向明显改善但全面优胜未通过，24轮安全审计0、目标5/5到达；另保留20261007_160119来源手柄运行，147.5s仿真USER_EXIT、外部保护0、avoid峰值633.340ms，未独立重演/审计，不能当P5新成绩。
- 影响与边界：保留EGO1P3原目录，EGO1P2/EGO1P4未修改。新旧仍共享ROS域42及单实例锁；临时构建产物不在原目录，P5使用新独立产物。
- 回滚/备份：完整来源EGO1P3及新目录performance/copy_from_ego1p3_20261007下来源清单、复制核验保留。
- 后续事项：完成独立主构建、现有测试、算法/历史证据未改和源目录完整性核验，更新版本说明及HISTORY总入口/索引。


### 2026-10-07 02:33 — [EGO1P3] v22完整24轮冻结验收完成，扫向改善、全面优胜未通过

- 状态：实现、构建、测试和完整对照完成；用户要求的主要工况全面优于EGO1P2尚未通过，明确保留五项失败判据。
- 目的：完成连续360°控制/感知实现后的成对重复验收，区分方向融合收益、安全和全面性能结论。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：src/pc_gvf下omni_depth.hpp/.cpp、depth_angular_controller_node.cpp、paper_guidance.cpp、两项新增C++测试及CMake/launch；scripts构建/运行入口与tools对照/审计/ROS脚本（精确19项运行差异见v22_runtime_changes.json与v22_diff_vs_EGO1P2.patch）；README.md、EGO1P3_VERSION.md、OMNI_ACCEPTANCE.md、WORK_LOG.md；HISTORY/README.md、PROJECT_INDEX.md、FOV_GVF_FAMILY.md、CHANGELOG.md；performance/omni_20261006下最终分析/冻结/图表/验证清单与汇总脚本。
- 最终实现：四路光轴深度按采集位姿与偏移反投影至world；180×37周期球面方向域额外接纳精确输入与历史世界方向，人工不再按最近90°相机切换。固定目标连续维护各子视图候选，真实通道已认证的精确目标方向提前返回。最终单矢量整形及完整运动/制动证明保留。联合观测包含证明递归5→7、1024内部细分预算不变，真实薄缺口/未知仍拒绝。
- 冻结协议：final_v22扫向/旋转各3对、恢复1对、目标5对，总24轮，交替顺序、共享锁串行、相同地图/输入/起终点。所有进程正常退出。无探索轮混入，没有丢弃失败或更改原严格判据。最终人工结果final_v22_analysis.json、目标omni_final_v22_analysis.json、完整表final_v22_results.md。
- 正向结果：扫向平均误差30.681→7.534°（-75.4%），向量RMS .550→.181m/s，投影进展21.136→28.413m（+34.4%），停车.717→.250s，jerk4.894→2.302，避障均时1.418→.363ms。新版三个工况旧相机规划切换均0；旧版分别5/14/6次。恢复进展81.917→82.529m、停车1.250→1.233s，仅单对。
- 退化与失败：旋转误差11.321→12.343°、向量RMS .570→.608、进展92.044→89.796m（-2.4%）、jerk3.161→3.735，计算均时.611→1.359ms；停车.500→.494s。目标两版5/5安全到达，均值69.717±.923→70.143±1.267仿真秒、墙钟70.318→73.279，停车.240→.250s；jerk2.108→2.011有所下降。未通过spin角误差/RMS/进展、goal耗时/停车五项严格门槛，不能写成全面领先；.010s停车差接近采样周期但仍照原判据记失败。
- 计算边界：P3旋转及目标各有累计1/60s计算截止状态，目标计算峰值101.910ms；不能声称严格50Hz。各工况p95/峰值与基线所有退化列在完整数据表。渲染图表已目检，首对轨迹只作示意，统计使用所有重复；目标底图是低空XY投影，不替代三维安全审计。
- 安全/回归：24轮完整.48m扫掠球对原占据体素/地面重叠0、外部保护0，速度≤2m/s、垂向≤1m/s、实测加速度≤1.2m/s²契约通过。主build成功，CTest22/22；平台15/15及原P2 CTest20/20为本任务早期通过、相关源码随后未改。45份Python AST、3份Shell语法、git diff --check通过。最终v22 ROS --omni在域83共享锁下运行通过缺前相机/未知/过期深度/过期输入/松手检查，日志归档；合成零输出拒绝不是飞行到达证据。
- 重放/完整性：21个旧状态4个原决定保持、17个原拒绝变为完整认证非零；v22与已测v21重放二进制相同，不称21/21决定一致。P2原2922文件/3257条目完全未改；125份冻结运行源、控制器/重放、24轮元数据及同协议SHA核验通过。安全核心变更有独立SHA清单，其余8项保护文件逐SHA未改。最终汇总final_v22_validation.json，不覆盖旧v5 validation_notes.json或v17失败结果。
- 影响与边界：实际手柄、GUI/RViz负载、其他地图/起终点、动态障碍、真机均未验收；四相机的近场/接缝/上下盲区保持未知。仅确认本批连续扫向更好；旋转候选稳定性、目标计算开销和全面优胜仍待解决。未修改EGO1P4、物理场景/桥/手柄映射/安全包络或实际动力学。
- 回滚/备份：EGO1P2原目录、v1至v22探索/失败及v22_frozen均保留；FOV_GVF_OMNI_DEPTH=0只关闭P3新前端，不等同完整P2（P3联合证明仍7层）。运行完整P2使用其原生目录/二进制；完整恢复P3可用冻结源主入口重建。无提交推送。
- 后续事项：全面性能要求未达标，不能标记全面优胜；后续应基于完整前端时序定位旋转退化及减少目标重复证据处理，再使用新冻结标签验证，不覆盖本批结果。


### 2026-10-07 — [EGO1P3] v22构建/目标探索通过，冻结完整24轮对照

- 状态：实现及核心回归完成，正式性能验收进行中，尚无全面优胜结论。
- 目的：以同一冻结版本比较连续扫向、独立转头、恢复和固定目标，避免用探索最佳轮代替正式结果。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：EGO1P3_VERSION.md、OMNI_ACCEPTANCE.md、WORK_LOG.md、HISTORY/CHANGELOG.md；performance/omni_20261006/v22_frozen及final_v22数据。
- 修改内容：当前文档明确固定目标连续维护局部候选池及已认证精确方向提前返回；修正“安全核心文件未改”的过期描述，联合覆盖上限5→7、预算1024及证据包含条件均明确记录。冻结125份运行源码、控制器和重放二进制；v22重放程序与v21字节一致。
- 验证：主入口build_v22.log成功，ctest_v22.log为22/22。目标探索三轮69.567/69.283/69.283s，3/3安全到达，均值69.378±.164s、停车.25s、jerk均值1.723，平均避障1.442/1.382/1.378ms；最大80.438/60.064/35.212ms。探索与正式统计隔离。
- 影响与边界：final_v22正式组为sweep/spin各3对、recovery1对、固定目标5对；交错顺序、同场景与输入、每轮原生二进制和launch哈希。测试期间不修改运行源码或重建。物理/动力学及另外8项受保护文件不变，paper_guidance.cpp差异有明确SHA清单。真实手柄、GUI/RViz负载、跨地图与真机未验收。
- 回滚/备份：v22_frozen与全部历史失败/探索数据保留；EGO1P2原目录继续保留。无提交推送。
- 后续事项：完成全部24轮、最终v22 ROS接口检查、独立安全/完整性审计并据全部结果更新验收结论。


### 2026-10-07 — [EGO1P3] v21证明修复通过，v22消除已认证固定目标方向的重复搜索

- 状态：v21核心/探索/ROS已通过，正式全面优胜仍未验收；v22待构建/目标探索。
- 目的：减少固定目标已有精确可认证方向时的额外侧视角候选计算。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：depth_angular_controller_node.cpp、版本/验收/工作日志、HISTORY/CHANGELOG.md；performance/omni_20261006下v21冻结、旧状态重放分类、ROS探针和探索数据。
- 修改内容：仅固定目标中，局部候选方向与世界目标方向点积>1-1e-12且已通过局部可达选择及全部真实证据走廊认证时直接返回该方向；其他观测仍完整接入认证，绕障时仍汇总各有效视角候选。人工模式逻辑保持v21，最终完整运动/制动证明不变。
- v21验证：新增解析union_refinement_check在原5层下明确失败，提升为7层、保持1024预算后主build/22项全通过。旋转单轮角误差9.895°、RMS .521m/s、进展91.348m、停车.5s、jerk4.326、安全审计0；计算截止实际累计1/60s，不能把日志1行说成1秒，最大avoid74.174ms。扫向7.355°/.178m/s/28.445m/.25s、安全审计0。
- v21目标：三轮69.050/69.750/70.883s，3/3安全到达、平均69.894±.925s、停车均.25s；探索均值不替代正式成对比较。日志显示直达尾段仍local_candidates=3，首轮平均避障2.293ms，为v22提前返回提供代码和运行依据。
- v21重放与接口：21个旧状态中4个原决定保持、17个原拒绝变为完整认证非零命令（原真实卡停5帧中的1帧+v17目标失败16帧）；这属于有意决策改善，不得写成21/21原决定相同。快照不含完整原始命中/前端时序，不宣称原现场已解决。ROS --omni缺前相机/未知机体/过期深度/过期输入/松手检查通过，日志归档。重放分类脚本首次误读摘要为frames，KeyError后按已记录目录读取并校验5帧数，错误记录保留。
- 影响与边界：包络证明包含关系、1024预算与所有物理/安全参数不变；paper_guidance.cpp仅5→7递归深度及注释变化已逐字反向校验，显式SHA差异在v21_frozen/intentional_core_changes.json。其余8项受保护文件继续按原SHA核验。
- 回滚/备份：v21_frozen完整125份源码、控制器/重放二进制；删除固定目标精确方向提前返回可回到v21。无提交推送。
- 后续事项：v22主build/22项核心、三轮目标探索；然后冻结完整sweep/spin各3对、recovery1对、目标各5轮。


### 2026-10-07 — [EGO1P3] 定位联合证明浅层拒绝，v21增加细分深度但保留工作预算

- 状态：离线反例已证实；解析回归及v21主构建/全套回归进行中，尚未动态验收。
- 目的：解决360°融合后多个真实观测联合覆盖机体却被有限层次包围球近似拒绝的可复现机制。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：src/pc_gvf/src/paper_guidance.cpp、src/pc_gvf/test/cpp/union_refinement_check.cpp、src/pc_gvf/CMakeLists.txt；版本/验收/工作日志及HISTORY/CHANGELOG.md；performance/omni_20261006下影子诊断脚本、结果、失败快照审计与v18目标五轮证据。
- v18正式结果：新旧均5/5安全到达、扫掠重叠/外部保护0；P2/P3均耗时69.283±.042/71.390±1.799s、墙钟69.303/74.567s、停车.237/.250s、jerk1.950/2.094，性能未胜出。因已有明确拒绝反例，v18人工正式组尚未开始，不能写成24轮完整比较。10轮元数据与124份冻结源一致性保留于v18_goal_integrity.json。
- 离线定位：仅复制冻结v17的核心源到/tmp/ego1p3_union_cert_diagnostic，建立从未安装/连接ROS的影子重放程序；编译和回放在共享锁下执行，不改变正在运行的正式组源码/二进制。原失败快照：5层/1024预算和5层/8192均拒绝，6层/1024仍拒绝；7层/1024证明.58m及+.01m余量，输出完整认证非零命令；8层额外开销更高。五次含进程启动/加载的重放墙钟中位5层约3.86ms、7层6.93ms、8层15.80ms，不能当在线控制计算时延。
- 解析反例：六个轴向已知球联合，完整覆盖半径R所需球半径可解析计算为max(d,sqrt(R²+d²-2Rd/sqrt(3)))；+8mm余量在5层拒绝、7层通过，-2mm真实薄缺口在5/7/8层始终拒绝。新增C++测试覆盖平移不变性、完整机体覆盖、薄缺口、缺失扇区、全未知及未知运动拒绝；先在旧核心证明测试失败，再改深度。
- 修改内容：仅把envelopeKnownImpl递归深度上限5→7，原1024次内部细分预算保持不变；所有被接受的盒子仍须由包围球完全包含于原真实证据，不使用表面采样授权。原先浅层已能证明的查询不增加递归。候选恢复v18，机体/认证半径、速度/动力学、保守深度/原始命中检查、未知拒绝、重放格式V18保持。
- 安全边界：本轮开始有意改变paper_guidance.cpp的证明精细度，不能继续说“安全核心文件未改”；认证包含关系和包络没有放宽。旧9文件完整性中此文件须列为明确修改，其余8项继续逐SHA核对。离线快照没有完整原始命中点与前端时序，证明改进不替代新实跑。
- 诊断失败：合成诊断首次把audit当stdout读取（实际写stderr），汇总IndexError；首次日志保留，合并stdout/stderr后重新汇总通过。不是核心验收通过的替代品。
- 回滚/备份：v17_frozen、v18_frozen、影子源/程序和解析诊断全部保留；恢复depth上限5并主入口重建可撤回此修正，无提交推送。
- 后续事项：完成先失败后通过的22项回归、重新分类旧快照变化、候选实跑及最终冻结成对比较。


### 2026-10-07 — [EGO1P3] v20插值退化撤回，恢复v18并开始目标五轮对照

- 状态：v20探索未通过；恢复v18准备冻结和固定目标五轮交错对照。
- 目的：保留实际有效的连续360°人工规划及固定目标候选连续性修正，不保留只在形式上更平滑却实跑更差的成本修改。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：src/pc_gvf/src/omni_depth.cpp、版本/验收/工作日志、HISTORY/CHANGELOG.md；performance/omni_20261006/v20_candidate、v18_frozen及后续omni_final_v18数据。
- 修改内容：球面源码完整恢复v17（俯仰附加1、最近格角域净空），控制节点保留v18固定目标连续局部候选池；校验节点与v18_candidate字节一致。新冻结标签v18，不把v19/v20探索混入。
- 验证：v20主build/21/21通过；旋转均角14.264°、RMS .696m/s、投影85.266m、停车.5s、jerk4.921、计算截止日志4行，明确退化；扫向7.458°/.179m/s/28.429m/.25s。两项扫掠重叠和外部保护均0，全部成果保留。
- 影响与边界：未放宽安全认证/物理，当前人工策略恢复至已有三轮正式证据的v17；固定目标v18目前仅3/3探索安全到达，尚不能宣称五轮正式通过。
- 回滚/备份：v19_candidate、v20_candidate保留源码与二进制；v17_frozen为上次完整冻结失败组，v18_candidate为已测候选。无提交推送。
- 后续事项：主入口恢复构建与核心回归，冻结124份运行源码/二进制；先固定目标各5轮，再据结果推进最终人工组，所有结果如实保留。


### 2026-10-07 — [EGO1P3] 撤回v19俯仰权重，v20检查方向评分连续性

- 状态：v19探索失败并撤回；v20待构建与探索。
- 目的：减少球面亚像素候选跨网格半格边界时的人工评分跳变，而非继续抬高升降代价。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：src/pc_gvf/src/omni_depth.cpp、WORK_LOG.md、HISTORY/CHANGELOG.md；performance/omni_20261006/v19_candidate及候选分析。
- 修改内容：相对俯仰附加权重2恢复1；方向评分的角域净空由最近格查询改为周期方位/俯仰双线性插值，精确输入、保留方向和细化方向的成本连续。碰撞锥几何、已知观测掩码与完整认证仍用原逻辑，插值不参与安全证明。固定目标保持v18策略。
- 验证：v19主build/21/21通过，但旋转均角14.293°、RMS .709m/s、投影84.157m、停车.933s、jerk6.521、截止状态日志9行，明确退化；扫向7.731°/.187m/s/28.368m/.25s，两项扫掠重叠和保护0。不能用扫向通过保留旋转退化版本。
- 影响与边界：v20是针对代码中离散评分不连续的独立假设，实际性能仍待实跑；未改场景、物理参数、安全证明或历史格式。
- 回滚/备份：v19_candidate保存当时球面源码、二进制/SHA及全部输出；恢复angular_room原最近格查询即回到v18。
- 后续事项：构建、21项核心回归、旋转与扫向探索，然后确定下一冻结版本；无提交推送。


### 2026-10-07 — [EGO1P3] v18目标三轮通过，v19检查多余俯仰代价

- 状态：v18探索三轮安全到达；v19已写入待构建/探索，未宣称正式优胜。
- 目的：修复目标策略连续性后，针对v17旋转25–29秒的额外升降/减速检查候选代价。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：src/pc_gvf/src/omni_depth.cpp、WORK_LOG.md、HISTORY/CHANGELOG.md；performance/omni_20261006下v18差异/三轮分析与v17旋转分段诊断。
- 修改内容：球面成本的相对俯仰tan²附加权重1→2；总角偏差/连续性/净空奖励和安全层不变。惩罚相对操作者3D意图的多余俯仰，不禁止显式升降；固定目标保持v18全视角局部池连续优先。此为待验证假设，不将相关性写成已证明原因。
- 验证：v18主build成功、21/21；三轮目标69.800/72.450/71.467s，3/3独立扫掠重叠/外部保护0，停车均.25s，均耗时71.239±1.340s，未取得新的成对正式结论。v17失败轮16/16状态逐值重放一致，完整audit保留。v17旋转分段均值25–29s：P2/P3投影速度1.417/1.313m/s、实测速率1.801/1.581、绝对垂速.169/.270，其余多数时段差异小。
- 影响与边界：仅候选评分，未改输入、模拟器、场景和安全参数；与P2比较必须保留新探索失败及旧正式失败，不以单轮替代重复统计。
- 回滚/备份：v17_frozen、v18_candidate源码/二进制及v18_diff_vs_v17.patch；恢复俯仰权重1即可回到v18。无提交推送。
- 后续事项：主入口构建；先旋转与扫向探索，若退步撤回，随后新的冻结验收。


### 2026-10-07 — [EGO1P3] v17完整24轮未通过，v18保持固定目标候选连续性

- 状态：v17严格验收失败；v18修正已写入，待构建与动态比较。
- 目的：用户要求比EGO1P2更优；不能把人工扫向显著改善替代固定目标失败。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：depth_angular_controller_node.cpp、README.md、OMNI_ACCEPTANCE.md、EGO1P3_VERSION.md、WORK_LOG.md；performance/omni_20261006下的报告/一致性脚本与全部final_v17证据；HISTORY/CHANGELOG.md。
- 修改内容：固定目标每周期优先四视角world局部候选池，保持各视图掩码更新/选择连续性；候选池无解才使用球面恢复。人工模式保留v17球面优先策略。真实证据、完整运动/制动证明、机体与限速不变。
- v17验证：主build/核心21/21，冻结源码124份/二进制一致性、EGO1P2完整2922文件未改、九个安全/平台/场景文件未改均通过；真实ROS --omni缺前相机/未知拒绝/过期/松手通过。24轮运行均退出0、全部扫掠重叠/外部保护0、速度/实际加速度契约通过，但固定目标只有4/5安全到达，不能写成全部任务成功。
- v17人工比较：扫向P2→P3均角误差31.705→7.580°，RMS .567→.182m/s，进展20.684→28.407m，停车1.011→.250s；旋转均角10.729→9.992°，进展92.309→91.869m退步，jerk3.029→3.833退步，两版停车.5s。恢复单轮停车2.15→1.25s、安全审计0。
- v17目标比较：P2 5/5、70.953±.934s；P3 4/5，成功均值71.092±1.939s、含失败104.873s、平均停车35.68s。第三轮在(50.087,12.233,1.857)机体包络未知而停到240s；回放audit证实.58包络拒绝，表面稀疏采样全覆盖不能替代完整证明。并未证明候选切换是唯一原因；v18是针对可见策略差异的修正，须重测。
- 其他记录：新增完整判据/逐轮表格/源文件核验脚本；图表第一次以未隔离Python运行因用户NumPy2与系统matplotlib ABI冲突失败，改用项目规定PYTHONNOUSERSITE=1 /usr/bin/python3后生成成功，无依赖安装或降级。报告区分成功轮到达均值与超时惩罚，所有失败保留。
- 回滚/备份：v17_frozen含源码包和二进制；final_v17_results.md/acceptance.json/integrity.json完整保留，未混入探索轮；旧validation_notes.json仅代表v5，不改成当前成绩。无提交推送。
- 后续事项：主入口构建v18，先固定目标探索，失败不得删去；随后决定冻结比较范围。真实手柄、GUI/RViz性能、跨场景和真机未验收。


### 2026-10-07 — [EGO1P3] v16扫向失败，v17区分连续意图与固定目标优先级

- 状态：v16正式验收失败；v17已冻结重测，未提前宣称通过。
- 目的：修复原局部候选覆盖连续球面目标而引入的缓慢改向长停顿。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：depth_angular_controller_node.cpp、版本/验收/工作日志；performance/omni_20261006/v17_frozen、候选与正式轨迹；HISTORY/CHANGELOG。
- 修改内容：人工控制优先连续球面候选，仅球面无认证方向时补充局部候选；固定目标沿用v16统一局部候选池优先策略。两种模式始终融合四路当前证据和历史，并使用原完整运动/制动证明。
- 验证：v16扫向三对全部完成，P3停车4.333/4.150/4.433s，最长1.500/2.633/2.900s，明显失败；安全审计仍0。本任务父批次在完整扫向结束后终止，退出143；spin/recovery/固定目标正式组均未开始，不能算通过。v17主build和核心21/21通过；探索sweep角误差7.441°、RMS .179m/s、进展28.432m、停车.25s、jerk2.293，扫掠重叠和保护均0。
- 正式验证：v17_frozen保存124份源码、源码包与二进制/SHA；final_v17扫向/旋转各3对、恢复1对，omni_final_v17目标各5轮正在运行，全部交错。
- 影响与边界：模式区分使用原已有fixed_goal语义，不根据测试标签选择算法；手柄轴映射、物理和场景不改。全部失败完整保留，不拼接最佳单轮。
- 回滚/备份：v16_frozen、v17_frozen、停止批次PID记录与全部原始数据。无提交推送。
- 后续事项：完整冻结比较、最终真实ROS入口复查、归档和最终说明。


### 2026-10-07 00:00 — [EGO1P3] 冻结v16全向候选池，正式对照开始

- 状态：实现与核心回归完成，正式验收运行中，未声明全面胜出。
- 目的：保留原成熟局部选路效率，并实现不依赖四向相机切换的360°规划。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：全向模块、控制节点、回归、ROS探针、版本/验收/WORK_LOG；performance/omni_20261006/v16_frozen及逐轮数据；HISTORY/CHANGELOG。
- 修改内容：v14透视tan偏差成本与已认证保留方向快速返回；v15/v16引入各有效视锥独立局部提案，在world统一比较并使用同一完整证据认证。视锥四角判定、.3s过期像素缓存清除，局部重设阈值.5°；固定目标保留原保持语义。速度/包络/实际制动模型、安全核心和场景未改。
- 验证：v16主build成功、核心21/21；v13真实ROS --omni入口/缺前相机/未知与过期拒绝通过，日志归档；当前局部候选池后的探针待收束复查。v14目标73.200s对同环境P2 69.300s失败；v16探索目标69.183s安全到达、停车.25s/jerk1.676，旋转误差10.011°/RMS .530/进展92.018m/停车.5s/jerk2.737，全扫掠重叠和保护均0。不能用探索单轮宣称正式胜出。
- 正式验证：冻结124份源码、源码包及二进制/SHA；final_v16扫向/旋转各3对、恢复1对，omni_final_v16固定目标各5轮；全部原始轮次保留并共享锁串行，运行中。
- 影响与边界：周期方向图和多视角候选合并不是无盲区球面传感器；大角度跳变仍受实际加速度/制动约束。EGO1P2保留，源码SHA不改，无提交推送。
- 回滚/备份：v14_sources、v15_sources、v16_frozen与全部历史失败组。
- 后续事项：完成正式对照、严格判据检查、最终记录与交付。


### 2026-10-06 — [EGO1P3] v11/v12探索与v13方向保留

- 状态：局部机制完成，最终优胜验收仍进行中。
- 目的：减少绕障方向帧间跳动和多余运动，保持全向输入连续。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：omni_depth.hpp/.cpp、控制节点、omni_depth_check.cpp、tools/ros_vector_guidance_probe.py、版本/验收/WORK_LOG；performance；HISTORY/CHANGELOG。
- 修改内容：v11沿用候选2.5、连续性.4；v12尝试去额外像素膨胀后固定目标变差，已撤回。v13补齐原.35秒方向保留窗口，仍每次重认证，输入累计变化>.05rad取消保留；直达自由时精确跟随不受窗口限制。
- 验证：v11旋转停车.5s/进展92.062m，v12旋转.5s/91.851m；二者安全审计均0。v12固定目标73.683s安全到达，停车1.233s/jerk3.477，未胜出。v13主build与核心21/21通过；新增保留/失效测试；ROS探针和动态测试排队中，不先写为通过。
- 影响与边界：原安全核心/执行物理/地图9项SHA仍一致；EGO1P2不改，全部失败保留。共享锁防止与其他任务仿真并发。
- 回滚/备份：v11_sources、v12_sources；无提交推送。
- 后续事项：完成当前验证后冻结重复比较。


### 2026-10-06 — [EGO1P3] 保留v9/v10失败并继续参数对齐

- 状态：性能验收尚未通过。
- 目的：减少统一球面候选的多余绕行；按完整重复结果筛选。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：全向候选/控制节点/主launch，OMNI_ACCEPTANCE/WORK_LOG；performance/omni_20261006与固定目标实验；HISTORY/CHANGELOG。
- 修改内容：v10启用可选直达通道但退化，已撤回false；v11候选制动前视1.2→原P2的2.5、连续性.12→原参数.4、俯仰附加权重4→1。最终运动认证仍1.2、.58包络；无安全核心或物理修改。
- 验证：v9完整3对旋转均安全且停车.5s，但P3角误差11.551对10.607°、进展89.420对92.385m，失败。v10旋转停车.917s/进展84.196m，固定目标75.233s安全到达，亦未胜出；v11主build成功、核心21/21，探索运行中。v9冻结源码124文件校验无漂移；EGO1P2全部2922文件无变化。
- 影响与边界：真实四相机/原场景/同输入，所有失败保留；图像查看工具因既有bubblewrap故障失败，改从已生成的本地图像base64查看，未改图像内容。
- 回滚/备份：v9_frozen、v10_sources及逐轮SHA；未提交推送。
- 后续事项：稳定候选后重新冻结全套比较。


### 2026-10-06 — [EGO1P3] v9直达分支配置更正

- 状态：仅配置核对；纠正上条默认行为描述。
- 目的：避免将保留的可选分支当成实际启用算法。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：OMNI_ACCEPTANCE、WORK_LOG、HISTORY/CHANGELOG。
- 修改内容：核对两版主launch均设置paper_intent_corridor=false；v9补齐分支但默认未启用，实际启用差异仅俯仰权重1→4。
- 验证：rg逐项核对源码及launch。v9源码/二进制未改，冻结对照继续。
- 影响与边界：不能把v9实跑收益归因于默认未启用的直达分支；旧记录保留并明确纠正。
- 回滚/备份：不涉及算法修改；v9_frozen保留。
- 后续事项：完成重复比较。


### 2026-10-06 — [EGO1P3] v9冻结与交错验收开始

- 状态：验收中。
- 目的：修复全向分支遗漏的原认证直达回归，减少无必要升降。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：omni_depth.cpp、depth_angular_controller_node.cpp、OMNI_ACCEPTANCE及本日志；performance/omni_20261006/v9_frozen。
- 修改内容：恢复原paper_intent_corridor完整通道认证后的精确输入方向优先；俯仰偏差附加权重1→4；完整运动安全核心未改。
- 验证：主build成功，核心21/21；v8三轮全部安全但一次停车1.167s，未胜出，原始结果保留。v9三对旋转测试刚开始，不提前宣称通过。
- 影响与边界：仅候选前端，源目录/地图/物理不变；共享锁串行测试。
- 回滚/备份：v8_sources、v9_frozen源码包/二进制/SHA；无提交推送。
- 后续事项：完成冻结性能与固定目标回归。


### 2026-10-06 — [EGO1P3] 360°融合候选实现、失败对照与恢复修正（验收中）

- 状态：算法实现与核心回归完成，重复性能验收仍在进行。
- 目的：消除四相机选一路的候选瓶颈，按用户要求验证比EGO1P2更优。
- 项目路径：`/home/starry/isaac-data/EGO1P3`。
- 涉及文件：`src/pc_gvf/{include/pc_gvf/omni_depth.hpp,src/omni_depth.cpp,src/depth_angular_controller_node.cpp,CMakeLists.txt,test/cpp/omni_depth_check.cpp,launch/isaac_cloud_navigation.launch.py}`；两主Shell、tools中的对照/分析脚本、测试轨迹；README/版本/OMNI_ACCEPTANCE/WORK_LOG；performance/omni_20261006；HISTORY索引与CHANGELOG。
- 修改内容：四相机采集位姿统一到world的180×37周期球面方向图；精确输入与世界方向连续保留；深度光轴反投影、机体及像素膨胀；真实原始深度认证候选，保留原矢量整形/完整运动与制动证明。v8新增近邻精查和可认证短走廊恢复。候选前视使用1.2制动，相比P2候选2.5更保守；实际运动认证两者均1.2，.48机体/.58包络/速度不变。批量运行可选等待现有共享Isaac锁。
- 验证：经主build构建；v8核心21/21，前期平台15/15、历史重放5帧一致；回归包含721方向、解析射线独立oracle、±180和相机接缝、缺失/过期/未来/撤销/未知、真实通道与运动证明。新增4°窄通道在v7先失败，v8修复后通过，失败日志保留。
- 性能失败：v2/v3旋转工况停车20.9/5.8秒；v6退化已撤回。v5冻结sweep三对平均角误差30.385→8.064°、停车.811→.250s、投影进展21.201→28.293m，均安全；但spin第三轮P3停车3.017s/最长2.517s，故正式稳定性验收失败。v7三轮spin平均进展85.504m/停车1.083s，亦未胜出；v8三轮待完成。每轮数据与二进制SHA保留。
- 批次取消更正：本任务v5父批次在recovery后显式STOP/TERM/CONT终止，退出143。固定目标P2首轮子进程实际已启动并独立到达72.1167s，缺少父进程收尾元数据，孤立轮保留；其余九轮未执行。不得误写成全组未开始或五轮通过。
- 影响与边界：360°是水平方位统一规划，四相机上下及近场仍有真实盲区。未改EGO1P2、安全核心、场景、手柄轴映射或执行物理；旧收益标签保留。原source_manifest再次逐文件验证P2无变化。
- 回滚/备份：EGO1P2原目录、source_manifest；v2至v7源码备份和v5冻结二进制保留。未提交/推送。
- 后续事项：完成稳定候选、冻结同条件重复对照、固定目标回归和最终记录；当前不能宣称整体优胜。


### 2026-10-06 — [EGO1P3] 建立副本并启动连续360°感知与控制优化（进行中）

- 状态：复制完成，算法与对照验收进行中。
- 目的：用户确认将EGO1P2完整复制为EGO1P3，消除按最近四相机选一路生成候选的方向限制，完成后与EGO1P2同条件验收。
- 项目路径：`/home/starry/isaac-data/EGO1P3`；来源EGO1P2保留。
- 涉及文件：完整新增EGO1P3；两个主Shell、benchmark launch、四份tools、AGENTS/README/WORK_LOG/版本说明；HISTORY索引和状态。
- 修改内容：2922文件、3257条目完整复制，含隐藏文件和Git；建立ego1p3独立/tmp构建、安装、日志与重放路径。旧版本历史报告保留。
- 验证：复制前后SHA256/模式/符号链接逐项一致，无共享文件inode；证据在performance/omni_20261006/source_manifest.json及copy_verification.json。新构建、测试、仿真尚未运行。
- 影响与边界：当前仅目录建立；尚未证明360°优化优于EGO1P2。原.48m机体/.58m认证与完整运动制动证明继续保留。
- 回滚/备份：EGO1P2完整来源及逐文件清单；未提交推送。
- 后续事项：实现统一世界方向候选、跨接缝回归、原场景交错重复对照并记录全部失败。

### 2026-10-06 21:37 — [EGO1P2] 静态历史局部冲突修复与防卡停回归

- 状态：完成一处可复现卡停机制的修复、默认启用和回归；原人工轨迹未完整复现，任意状态不卡停未验收。
- 目的：响应用户“修复”“尽可能避免卡住”，避免远处小范围深度冲突抹掉整幅历史观测，丢失近场起步通道。
- 项目路径：`/home/starry/isaac-data/EGO1P2`。
- 涉及文件：`src/pc_gvf/src/paper_guidance.cpp`、`include/pc_gvf/paper_guidance.hpp`（位于src/pc_gvf下）、`src/pc_gvf/src/paper_replay.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/history_repair_check.cpp`、`src/pc_gvf/test/cpp/intent_response_check.cpp`、`src/pc_gvf/test/fixtures/paper/isaac_stall_repair_trace.json`、`scripts/isaac/run_fov_gvf_navigation.py`；`README.md`、`EGO1P2_VERSION.md`、`WORK_LOG.md`、`performance/stall_fix_20261006/`；`HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/CHANGELOG.md`。
- 定位：原frame_1历史仅9/16幅，关键前向旧观测缺失；原frame_4前进0.1m的机体上下边界存在无证据采样点。代码遇任一原始深度冲突会撤销整幅旧图，合成同条件回归证明会导致旁侧自由起步通道丢失。原快照不含被撤销图和完整原始输入，因此不能声称已还原每次历史撤销事件或人工操作轨迹；采样仅定位，不参与安全认证。
- 修改内容：新增`paper_local_history_repair`，纯核心默认false，主launch以`FOV_GVF_LOCAL_HISTORY_REPAIR=1`默认启用；且必须满足静态历史（retain_verified_travel且history_uncertainty_rate=0）。对每个冲突原始命中投影完整不确定性球，含保守边界像素，只缩短对应深度；撤销借用旧图并发布不可变缩小副本，其余历史自由列保留。动态历史/开关关闭仍整图撤销；球、管、待执行证明原有撤销保留。最后强化为所有命中均相对原图判断，避免前一个命中仅裁掉后一命中中心却漏掉其不确定性邻域。
- 配套：重放格式V17→V18（EGOPAPRI）保存新开关，旧格式继续读取且关闭新开关；离线audit增加未覆盖表面样本。trace测试可选独立yaw_rate，旧trace保持原自动朝向，真实手柄读取/轴映射不改；新72秒轨迹是近似场景测试，不是原摇杆录制。
- 验证：均经主build脚本构建。最终`PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p2_isaac_build/pc_gvf --output-on-failure`20/20，`/usr/bin/python3 -m pytest -q src/pc_gvf_platforms/test`15/15。新增回归覆盖旧模式同场景无法起步、修复后松手再输入输出非零完整认证命令、障碍与不确定性邻域拒绝、所有像素只减不增、旧引用撤销、采样间新冲突和动态整图撤销。最终原用户5帧+历史82–85轮16帧，21/21逐值重放一致；这证明旧状态兼容，不能补回已丢失历史。修改Python AST和git diff --check通过。
- 实跑：串行通过主运行脚本。初稿72秒手柄等效开关对照停车均1.250s/最长0.250s，路程88.896/89.046m；初稿仍有COMPUTE_DEADLINE峰值571.186/455.934ms。最终代码94秒恢复开关对照停车1.183/1.200s、最长均0.250s、路程73.320/73.271m；最终72秒独立航向再测停车1.250s、最长0.250s、路程90.151m。三组最终运行全部5段平移输入有位移，完整.48m机体扫掠重叠及外部保护均0，最小保守间隙分别.196/.177/.143m；最终独立航向避障max7.272ms，但控制周期间隔max93.334ms，不声称逐周期实时。五组均按手动模式时间上限正常结束MANUAL_TIMEOUT、退出0，不是固定目标到达验收。
- 失败与限制：格式升级后的第一次CTest19/20，因intent_response_check仍硬编码V17；更新测试预期后20/20，失败日志ctest_ab.log保留。两条旧模式轨迹本来就无>1s长卡停，不能据其宣称真实卡停频率下降；人工原轨迹/真实手柄动态/实物/跨地图未验收，原五帧仍输出其原决定；新仿真没有可保存失败快照，不写成新实跑V18快照通过。沙箱mountinfo启动失败，所有后续命令经升级审批执行。
- 影响与边界：只修复静态历史冲突处理与测试诊断；机体.48m、认证.58m、速度/动力学、完整运动制动证明和未知拒绝不变；不新增在线全地图许可、不执行未经认证脱困。旧EGO1P1/EGO1P0/user_ego与场景未修改。新逻辑不能恢复旧进程已经删除的历史，应重启仿真以重新积累观测。
- 回滚/备份：`performance/stall_fix_20261006/before_sources.tar.gz`、before_sha256.json及旧paper_replay；精确差异implementation.patch、最终changed_sources_sha256.json、测试/CSV/审计/所有失败与completion_summary.json已保存。临时对照或关闭修复用`FOV_GVF_LOCAL_HISTORY_REPAIR=0`加原运行命令；彻底回退仅恢复本轮相应文件后主入口构建，不用旧Git HEAD覆盖继承未提交改动。无提交推送。
- 后续事项：原人工场景需用户实际操作复验；继续出现停滞时利用新快照定位剩余证据缺口。真实未知边界仍应停车，本次不承诺所有环境永不卡住。

### 2026-10-06 21:01 — [EGO1P2] 实际手柄空地停滞诊断

- 状态：仅检查并归档，原因定位到自由空间认证边界；未修复算法。
- 目的：解释用户20:57截图中空地无法前进的问题。
- 项目路径：`/home/starry/isaac-data/EGO1P2`。
- 涉及文件：新增`performance/stall_20261006_205620/`（5组原始快照、日志、审计、重放结果与SHA）；`EGO1P2_VERSION.md`、`WORK_LOG.md`、`HISTORY/CHANGELOG.md`、`HISTORY/FOV_GVF_FAMILY.md`。
- 检查结果：用户运行20261006_205620已退出，不能实时读取；日志反复NO_REGULAR_FIELD/NO_CERTIFIED_DIRECTION，夹杂CERTIFIED_TRACKING和COMPUTE_DEADLINE。frame_4的40.333s位置约(-43.6852,-10.0982,2.33919)，实际速度0，手柄意图(1.92717,.534791,0)，候选0。当前.58m包络认证通过、增加.005m失败；后备构场需要.01m前缀，近意图方向仅认证.00146484m；frame_3接受约.02m/s小指令，解释微动/停车反复。采样前半球最大.02173m，后方约.3728m；这些是保守认证长度，不是实际障碍距离。
- 验证：现有paper_replay --audit读取frame_3/4；verify_navigation_replays.py共5/5决策一致。按既有occupancy体素AABB约定做独立离线诊断，当前.48m机体表面近似净距1.281m，沿输入方向前进.5m整段保守扫掠净距仍约1.115m；地图仅离线审计，不喂给在线规划器。运行外部保护介入0，计算峰值111.790ms。截图与快照并非严格同步；只诊断同一运行已复现的停滞，未执行新仿真/构建/测试。
- 修改内容：只保存证据、追加状态与日志，未改源码/参数/运行方式。
- 影响与边界：主要是已观测自由空间/历史认证衔接导致可通行空地无法生成可认证前进通道；细分到具体盲区或证据遗漏仍需进一步定位。手柄仍有输入，不能归咎手柄未解锁。motion_scale诊断使用previous_command=0，三个比例均通过不代表前进可行。前一轮净空计算优化未解决此问题。
- 回滚/备份：原/tmp日志和快照完整保留，项目内副本另有SHA；无需代码回滚。无提交推送。
- 后续事项：围绕这5帧定位近场/上下盲区与历史体积衔接，再验证受完整安全认证约束的恢复动作；不得以缩小机体或放行未知空间掩盖停滞。

### 2026-10-06 20:51 — [EGO1P2] 精确净空距离线性化与首轮实测

- 状态：完成本轮计算优化与验证；导航提速及全面领先未通过。
- 目的：阅读代码和历史进展后，降低默认深度候选重复净空查询的计算成本，保持原安全契约。
- 项目路径：`/home/starry/isaac-data/EGO1P2`。
- 涉及文件：`src/pc_gvf/src/depth_angular_core.cpp`、`include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/clearance_check.cpp`（include路径完整为src/pc_gvf/include/pc_gvf/depth_angular_core.hpp）；`EGO1P2_VERSION.md`、`README.md`、`WORK_LOG.md`；`performance/clearance_20261006/`；`performance/navigation_benchmark_20260929/edt_before_20261006_*`、`edt_after_20261006_*`及其PERFORMANCE.md追加；`HISTORY/README.md`、`FOV_GVF_FAMILY.md`、`CHANGELOG.md`。
- 修改内容：穷举障碍最近距离改为两遍精确欧氏变换，O(N×B)→O(N)；提供可测试净空接口、全自由/全阻塞快速分支、边界验证；不改候选代价/排序、0.48m机体/0.58m认证、2m/s速度、实际1.2m/s²和所有运动/制动证明。
- 验证：两次主入口构建均成功（初始优化21.6s，测试修正另见build_test_fix.log）；修改前核心18/18；`PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p2_isaac_build/pc_gvf --output-on-failure` 最终19/19；加载Humble与新local_setup后`/usr/bin/python3 -m pytest -q src/pc_gvf_platforms/test` 15/15；clearance_check穷举65536种4×4掩码、192矩形样例逐值一致，候选排序及超长一维/非法尺寸通过。`clearance_check --benchmark`七轮交替顺序取中位数，64×48/50%障碍754.388→13.933us，54.1倍。历史82–85轮16帧`verify_navigation_replays.py`全部一致；本轮新运行无可验证快照，不声称新快照通过。源码检查确认3个既有源码/构建文件变化、1新测试；其余归档110文件当时未变，后续仅追加文档；git diff --check通过。
- 实跑：`tools/run_navigation_benchmark.py edt_before_20261006 ego1p2 --repeats 2`与`edt_after_20261006`同命令，通过主运行Shell串行；`tools/analyze_navigation_benchmark.py`分别审计。前69.300/69.317、后71.433/69.283仿真秒，前后各2/2安全到达、全部扫掠重叠/保护0；均值69.308→70.358s，墙钟68.827→70.573s，停车0.250→0.242s，jerk1.710→2.243。避障p95 1.500/1.705→0.970/0.994ms，但后组首轮max319.867ms、COMPUTE_DEADLINE/TARGET_HOLD；不隐去退化，不把两轮写成五轮或跨地图验收。
- 失败与限制：初次新增测试误用输入源像素所属连通域，未考虑既有历史偏向重捕获，导致18/19；修正测试为实际选择源连通域后19/19，算法未为测试放宽，失败保留ctest_after.log。沙箱启动因mountinfo路径失败，使用获准升级命令；会话中断令前后组隔时，非交错对照，不能用两轮归因普遍导航性能。真实手柄/实物/跨地图未运行。
- 影响与边界：净空函数和局部计算性能改善；平均导航时间、波动与jerk并未改善。默认保留精确计算优化，不宣称全面优于旧版。EGO1P1/EGO1P0/user_ego、地图、launch、安全认证与共同执行代码未修改；HISTORY规则未改。
- 回滚/备份：`performance/clearance_20261006/before_sources.tar.gz`及113文件SHA、before_git_status；旧控制器/重放/静态库保留，`implementation.patch`是本轮实际源码差异。可仅恢复3个原文件并移除新增测试后经主入口重构建；不要用旧Git HEAD覆盖继承未提交改动。未提交、未推送。
- 后续事项：以真实计算截止/观测边界事件为目标，继续联合分析候选可执行性与后备构场峰值；再冻结交错多轮验证。当前不宣称导航提速或全工况连续可控。

### 2026-10-05 19:43 — [EGO1P2] 完整复制 EGO1P1 并建立后续版本状态说明

- 状态：完成复制、路径适配、状态整理和构建/回归；未进行新导航性能验收。
- 目的：按用户要求保留目前 EGO1P1，完整复制为 EGO1P2，并说明代码算法进度和未达标内容。
- 项目路径：`/home/starry/isaac-data/EGO1P2`；保留来源 `/home/starry/isaac-data/EGO1P1`。
- 涉及文件：新建完整 EGO1P2 目录；其内有意适配 `AGENTS.md`、`README.md`、`WORK_LOG.md`、`scripts/build_isaac_ros_workspace.sh`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf/launch/navigation_benchmark.launch.py`、`tools/run_navigation_benchmark.py`、`tools/analyze_navigation_benchmark.py`、`tools/verify_navigation_replays.py`、`tools/ros_vector_guidance_probe.py`；新增 `EGO1P2_VERSION.md`、`performance/branch_copy_20261005/` 清单/校验脚本/测试证据。同步更新 `HISTORY/README.md`、`PROJECT_INDEX.md`、`FOV_GVF_FAMILY.md` 和 `CHANGELOG.md`。
- 修改内容：用 `cp -a --reflink=auto` 全量复制 2,789 文件、321 子目录（含隐藏文件、Git、未提交改动、全部失败实验），复制前后 SHA256/类型/模式一致，无源目标文件硬链接。新目录默认使用 `/tmp/fov_gvf_ego1p2_isaac_{build,install,log}`、独立 ROS 日志/重放路径；实验新增 ego1p2，保留三个旧版标识。共用 ROS 域和单实例锁继续防止重复控制。原算法、主 launch 参数、场景与历史实验不改。
- 状态说明：明确默认是深度候选→单三维矢量整形→完整运动/制动认证的工程混合实现；静态证据、0.48m机体/0.58m认证、实际1.2m/s²模型不改。继承 final_v4 的 EGO1P1 69.893s 对 user_ego 68.980s，仍未全面领先；主动信息增益/系统回退、跨地图/当前手柄动态验收等缺项写入版本说明。不得把继承成绩改名成 EGO1P2 新成绩。
- 验证：执行 EGO1P2 主 build 脚本，3 包构建成功（23.2s）；`PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p2_isaac_build/pc_gvf --output-on-failure` 18/18；加载 Humble 与 EGO1P2 local_setup 后 `/usr/bin/python3 -m pytest -q EGO1P2/src/pc_gvf_platforms/test` 15/15；43 Python AST、3 Shell `bash -n`、`git diff --check` 通过。只生成 benchmark launch 描述，ego1p2 二进制/3包前缀均指向新安装目录；没有启动仿真节点。完整清单再审计证明 EGO1P1 未变、无继承文件删除、复制的 Git/场景/历史实验一致。Isaac GUI、手柄动态、导航和新性能对照本次未运行。
- 校验问题与处理：首次保留审计因 Python 3.10 不支持 `hashlib.file_digest` 失败，改分块 SHA256；再次检测副本 `.git/index` 字节刷新，核对 `git ls-files --stage -z` 暂存内容相同且来源 index 未改后恢复副本 index，完整审计通过。过程见 `audit_notes.json`；未省略失败，未改来源。
- 影响与边界：EGO1P1 完整保留，EGO1P2 可独立构建；本轮是目录版本建立，不是算法提速或论文复现完成。旧报告/日志的 EGO1P1 名称与旧命令保留历史含义，新运行使用 README/EGO1P2_VERSION。复制的 Git 仍为 ego1p1、HEAD 50f7c5500af86e31d0d9af03de8ec9a29ffb10e1；工作区未提交改动完整继承，本次没有建 Git 分支、提交、推送。
- 回滚/备份：原 EGO1P1 与复制前逐文件清单保留，可随时从来源重新建立另一副本；不得以早期 Git HEAD 单独代替当前来源工作区恢复。
- 后续事项：后续工作在 EGO1P2 进行，算法全面领先、主动探测/回退与真实手柄等仍需单独实现和验收；本次目录任务已完成。


### 2026-09-30 03:35 — [EGO1P1] 收束验证完成，全面优胜目标仍未达标

- 状态：部分完成，非性能验收通过；不标记用户要求的全面优胜已完成。
- 目的：确认退化配置已撤回、当前源码与实际运行对应，并完整保留比较证据。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：保留的控制/配置与测试、README/NAVIGATION_BENCHMARK/PAPER_IMPLEMENTATION、performance/navigation_benchmark_20260929、HISTORY README/FOV_GVF_FAMILY/CHANGELOG及WORK_LOG。
- 修改：主launch SHA与frozen_v4完全相同。相对v4仅四个源码/测试文件包含默认关闭的局部路径完整包络证明；retained_sources归档114文件、源码包、控制器及重放二进制。未修改旧算法、公共协议/运行端或地图。所有79–85退化试验原始数据、归档及参数保留，未覆盖冻结均值。
- 验证：最终3包构建成功，核心CTest18/18，平台整目录15/15（先math子集7/7，后完整目录）；22份Python AST、3份Shell bash -n和git diff --check通过。82/83/84/85共11+2+1+2=16个已保存候选的核心快照用当前二进制重放全部一致，不重建ROS候选或墙钟截止。user_ego44、EGO1P0 46源码SHA未变，原USD/occupancy及共同运行源码SHA未变；初次完整性脚本误把共同文件相对路径按cwd读取报FileNotFoundError，修正基目录后审计通过，未改被检文件。
- 恢复后实测：retained_check完整协议单轮安全到达69.266667仿真秒/69.906228墙钟秒；停车0.250秒、jerk1.685936、路径133.264308m、最小保守机体净距0.157082m，外部保护和扫掠重叠均0。源码冻结后未改、实际二进制和launch与归档一致；单轮不计入v4五轮均值，也不声称新五轮验收。
- 性能结论：最近完整五轮仍为v4，新版69.893仿真秒/70.981实际秒，user_ego68.980/69.106；新版停车/平均jerk/净距更好，完成时间与波动更差。EGO1P0旧五轮均240秒超时并有保守扫掠重叠；v3新版曾4/5，失败不隐去。全面优于两个旧算法尚未成立。
- 影响与边界：静态场景、固定起终点和统一实际物理；旧内部规划制动2.5、新完整认证1.2这一不对称已披露。手柄/实物/跨地图未验收，当前工程混合方法不是论文全部公式原样实现。未来需要联合优化候选选择与动态可执行性，不能承诺相同调参必然领先。
- 回滚/备份：任务开始19:29全备份、frozen_v1/v2二进制与哈希、frozen_v3/v4源码包和当前retained归档；全部失败保留，未提交或推送。
- 后续事项：用户要求的速度与稳定性全面优胜仍未完成；禁止将本条误读为该目标验收通过。


### 2026-09-30 03:27 — [EGO1P1] 撤回退化候选并收束当前阶段验收

- 状态：部分完成；同地图实测与多轮优化已执行，“全面胜过两旧版”未完成。
- 目的：保留已验证主策略，明确未达标项，不用最快单轮或删失败制造优势。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主launch、paper_guidance.hpp/cpp、控制节点、motion_consistency_check、验收/方法/入口文档、性能归档及HISTORY。
- 修改：85旧48×36角域到达72.3833秒，恢复64×48。当前默认执行策略恢复final_v4：保守深度目标候选、2.4指令斜坡/jerk6、真实1.2物理与.58认证、固定目标前向+历史、手动四路、10Hz完整接纳/60Hz控制；局部网格、前馈、取样候选、调和建议均关闭。83新增完整路径机体几何检查及回归保留，仅影响关闭的局部路径选项，不作为默认提速成果。
- 当前正式证据：final_v4新旧user各5/5安全到达，新69.893±1.336仿真秒/70.981墙钟，旧68.980±.403/69.106；新停车.25对旧.497，jerk1.903对2.001，但速度慢、完成时间波动大，所以未全面领先。原final_v1 EGO1P0五次240秒超时且保守机体扫掠审计不通过，原样保留。v3一次超时与所有退化配置均保留。
- 验证：新完整路径回归包含于83的18/18；平台15/15为既往冻结结果。当前收束重构建/全量回归/旧源码与地图SHA/语法差异核对待执行，以后续记录为准，不预先写通过。当前源文件含关闭分支的几何修正，不能声称与v4二进制逐字节一致。
- 影响与边界：论文原调和场/流盒为后备，主链为工程混合；真实手柄、实物、多地图未验收。没有放松未知空间/物理边界，也没有改旧算法或共同平台来制造优胜。单条路线优化不能保证任意地图领先。
- 回滚/备份：19:29全备份、各冻结源包/二进制、全部参数/轨迹；本轮当前源将另外归档retained_sources。
- 后续事项：动态可执行性与候选排序仍需联合优化，宽角度分支选择和极端计算延迟尚未根治；不能承诺单纯继续调参必然胜出。未提交推送。


### 2026-09-30 03:24 — [EGO1P1] 前馈未提速，核对旧版候选角域尺度

- 状态：全面领先尚未通过。
- 目的：区分角域离散选路变化与实际安全约束。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主launch、性能及双日志。
- 结果：84前馈到达71.0833秒，未提速，已关闭并恢复jerk6；完整指标见iteration84_analysis。
- 修改：85仅将保守角域64×48改为旧user_ego默认48×36，净空奖励已有48列归一化；前向固定目标/手动四路、10Hz完整接纳、60Hz控制、原始点矛盾检查、.58m认证及全部物理参数不变。局部路径保持关闭；新增完整路径几何检查与测试保留为可选模式的正确性修正。
- 验证：待构建和实际运行；分辨率改变不等于降低安全标准，粗化仍由保守深度下采样执行，不能以成功单次替代冻结重复。
- 回滚/备份：恢复64×48回到v4默认执行策略；全部冻结/探索/失败保留。
- 后续事项：核对耗时和安全后形成当前阶段结论，未提交推送。


### 2026-09-30 03:21 — [EGO1P1] 保留路径几何修正，撤回默认局部路径

- 状态：全面领先未通过。
- 目的：保留正确的候选空间证明，避免性能退化；重新核对低延迟链路下的响应补偿。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主launch、完整路径证明/回归、实测与双日志。
- 结果：83到达74.9333仿真秒，多个TARGET_HOLD和计算截止事件；核心18/18通过，但实际导航性能退化，局部路径恢复关闭。新增proposalPath及中心/边缘/中段未知测试作为可选路径的正确性修正保留，不把它写成已提速。
- 修改：84仅在现有前向消费、60Hz控制/10Hz接纳且局部路径关闭的基础上，启用已有独立速度参考前馈、jerk3。此前前馈在四相机和局部路径开启时实测不够平滑，本轮检验低延迟链路下的效果。前馈模式仍使用真实1.2m/s²限制，proposal_command_accel参数不在该模式生效；完整实际状态制动证明和2m/s上限不变。
- 验证：83实际结果/完整几何审计保留，84待构建及实跑；不将既往前馈失败删除或声称理想同步单元测试等于真实平滑。
- 影响与边界：旧源码/共同平台不变，未减少实际安全包络。
- 回滚/备份：关闭feedforward、jerk恢复6即回到v4执行策略（可选局部路径新增更严格几何检查）；所有冻结与失败保留。
- 后续事项：真实导航/重复验证，未提交推送。


### 2026-09-30 03:16 — [EGO1P1] 对局部候选路径补齐完整机体几何证明

- 状态：新方案验证中，全面领先仍未通过。
- 目的：修正局部候选只证明中心点可见、机体边缘可能伸入未知的问题，减少被吸引到认证边界。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：paper_guidance.hpp/cpp、控制节点、主launch、motion_consistency_check及双日志/实测记录。
- 修改：82取样候选最终安全到达130.30秒/126.995墙钟，停车56.483秒、jerk9.8003；性能失败，撤回取样。83局部路径每次重规划后必须调用proposalPath，以原.58m包络逐段证明整条路径在真实深度+认证历史中，不能只查中心或顶点；只有通过后才允许替代较长深度目标分支，原15°任意夹角限制由完整路径几何证明取代。当前状态到新方向的动态过渡和全程制动仍逐周期独立认证。路径几何查询不记录新自由证据、不授权执行。
- 验证：新增回归覆盖弯折路径已知、中心已知但机体外缘未知、两端已知而中段未知、陈旧观测和查询不产生证据；正在构建执行，不提前写通过。82全部扫掠重叠0，不能将长停低速写成性能优化。apply_patch因已有沙箱mountinfo故障无法执行，改用已授权Shell精确编辑。
- 影响与边界：不放松未知/尺寸/物理/旧基线；只加强候选路线筛选，不能保证未来动态可行或持续通行。非论文新增完整复现声明。
- 回滚/备份：frozen_v4源码包；关闭局部路径即可恢复v4执行策略，全部探索数据保留。
- 后续事项：回归及真实场景后决定是否冻结，未提交推送。


### 2026-09-30 03:11 — [EGO1P1] 撤回调和建议，区分候选取样与安全证据

- 状态：全面领先仍未通过。
- 目的：检验保守深度缩图是否在方向候选阶段放大通道封闭。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点、主launch、性能与双日志。
- 修改：81调和场建议耗时71.2333秒，未优于v4，节点与launch恢复冻结v4原文件；试验节点另存rejected_harmonic_proposal_node.cpp。82启用已有paper_sampled_proposal，只给候选规划传近邻采样深度，实际空间认证/走廊认证仍用原保守缩图，每个接纳帧全部原始障碍点仍参与矛盾撤销，.58m执行包络不变。与此前稀疏原始点锥方案不同，此处仅生成固定64×48方向候选。
- 验证：81构建、核心18/18通过并实际到达；独立审计另存iteration81_analysis，统计不与v4混合。82待构建和实跑，不能预先写改善。
- 影响与边界：候选不是运动授权，未知区域不能由取样变为可通行；旧源码、场景、物理和共同基准不改。
- 回滚/备份：去除paper_sampled_proposal即还原v4，原冻结源包和全部失败保留。
- 后续事项：实测审计与最终重复，未提交推送。


### 2026-09-30 03:08 — [EGO1P1] 调和场建议方向对照

- 状态：优化中，全面领先未通过。
- 目的：检验直接追安全目标方向与沿角域调和场转向的差异。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点、主launch、实验及双日志。
- 修改：80提高接纳至30Hz安全到达71.75秒/74.4347墙钟、jerk2.3784、停车.25秒，未胜出，恢复10Hz。81显式paper_harmonic_proposal=true，computeGuidance保留direction_proposal_only但计算调和场并使用其command_world作建议；最终3D整形与完整制动认证不变。选定安全目标与场给出的中间建议方向分别保存，避免拿中间切线覆盖下一帧目标记忆；局部路径仍关闭，若启用则记忆被替换的真实目标。
- 验证：80独立扫掠重叠0、构建通过。81使用已有核心调和场/方向候选接口，正在构建回归，未提前写实测结果。
- 影响与边界：此为工程混合控制路径，不能称论文逐式完全复现；物理、速度、包络、未知规则、旧算法与地图不变。
- 回滚/备份：关闭paper_harmonic_proposal可恢复目标方向模式，frozen_v4源包与二进制保留；所有探索失败保留。
- 后续事项：单轮筛选后冻结重复；未提交推送。


### 2026-09-30 03:04 — [EGO1P1] 撤回候选缩小，检验前向观测时效

- 状态：未达全面领先，继续探索。
- 目的：减少慢速障碍释放和过时目标带来的绕路。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点、主launch、实测记录及双日志。
- 修改：79候选半径.50结果75.9167秒，较v4明显退化，撤回为.58；80仅将固定目标前向消费下的完整帧接纳10→30Hz，控制60Hz、真实相机发布60Hz、完整安全证明不变。三帧障碍释放原本随10Hz接纳变慢，该试验检验提高时效是否改善选路，尚不预判结果。
- 验证：79构建成功；第一次CTest漏设PYTHONNOUSERSITE造成NumPy2与系统扩展冲突，两Python项失败（C++全通过），修正环境后完整18/18通过。79实测到达，独立几何审计正在执行，不提前写安全通过。80待构建实测。
- 影响与边界：没有减小执行包络或改变旧算法/公共物理；保留79失败的性能结果。未做真实手柄/实物试飞，未提交推送。
- 回滚/备份：frozen_v4源码包；将depth_processing_rate恢复10即回到v4。
- 后续事项：单次探索之后还须冻结重复，不混合不同配置平均。


### 2026-09-30 03:01 — [EGO1P1] 第四组完整结果及候选膨胀对照

- 状态：部分完成，全面领先仍未通过。
- 目的：核对全部重复结果并降低候选筛选的额外保守性。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：depth_angular_controller_node.cpp、performance/navigation_benchmark_20260929、WORK_LOG/HISTORY。
- 验证：final_v4新旧各5/5安全到达、扫掠重叠0；新版69.893±1.336仿真秒、70.981墙钟秒、jerk1.903、停车.250秒；user_ego68.980±.403、69.106、2.001、.497。新版一轮72.283秒且有.1秒计算超时，不能删除该轮或用最佳结果替代。114源码哈希未变、10轮无提前中止、公共SHA与v1相同，二进制/launch各算法唯一；核心18/18及平台15/15冻结前通过。已目视检查路线图。v4没有保存到拒绝重放帧，未声称该组重放通过。第一次完整性脚本读错exploration_stall_s层级报KeyError，修正为结果顶层后完整性通过。
- 修改：下一探索79只把深度方向候选的锥膨胀由.58改为实际body+safety=.50m。候选必须经过原.58m走廊认证，最终指令仍经原.58m完整过渡/制动认证；未改任何自由空间证据、未知规则、真实机体或物理参数。考察是否减少更短支路被过早丢弃，不能先验宣称有效。
- 回滚/备份：frozen_v4_sources.tar.gz及114文件SHA、二进制/launch；恢复候选body_radius=paper_->envelopeRadius()、safety_margin=0可撤回79。
- 后续事项：构建/回归/实测79，全部失败保留，未提交推送。


### 2026-09-30 02:39 — [EGO1P1] 冻结第四组，前向目标模式与旧版交错复测

- 状态：正式验收中，尚未达成全面领先。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：performance/navigation_benchmark_20260929及本地/全局记录、HISTORY入口状态。
- 冻结候选：以78为准，指令斜坡2.4/jerk6，真实1.2m/s²不变；固定目标只消费前相机+认证历史，手柄保留四相机；相机仍60Hz发布，算法接纳10Hz完整帧、控制60Hz；局部网格覆盖关闭，净空奖励按旧48列尺度归一化，有效目标记忆一致；.58m认证不变。
- 78实测：安全到达69.30仿真秒/69.1771墙钟，jerk1.7185、停车.25秒、路径133.3622m、净距.1906m、扫掠重叠0。77对应69.1833/71.2141/1.6355，墙钟存在波动，因此必须重复，不能从单次得出普遍胜出。
- 验证：78加速指令回归通过；冻结前完整核心18/18、平台15/15正在执行或已由日志给出结果，通过后串行启动final_v4。实际最终测试结论以ctest_frozen_v4/platform_frozen_v4日志和该组结果为准。此前真实模式4/1路切换通过。正式5轮与user_ego交错，不提前中止、不并行构建/测试，保持公共原协议及旧源码。
- 归档：frozen_v4_sources.tar.gz/114文件SHA、主launch、控制器/重放二进制、逐轮参数/公共SHA；先前所有失败和v1/v2/v3数据保留。
- 影响与边界：消费策略是新算法自动目标模式的变化，必须披露；未改旧算法或公共物理来制造优势。HISTORY README/FOV_GVF_FAMILY已同步截至v3失败及后续试验，规则本身未修改。
- 后续事项：完整重复和独立安全审计结束后形成结论，未提交推送。


### 2026-09-30 02:35 — [EGO1P1] 归一化后路线试验完成，继续核对指令响应上限

- 状态：尚未全面达标。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主launch、motion_consistency_check、性能记录。
- 77结果：安全到达69.1833仿真秒，详细墙钟/jerk/几何见iteration77_analysis.json；较74单次69.4833有所改善，仍不能替代重复均值。77完整18/18通过。
- 78修改：指令斜坡1.8→2.4m/s²，jerk仍6；真实物理与安全预测的加速度上限仍1.2m/s²。回归夹具同步验证更高指令上限，实际执行上限、完整制动、发布覆盖、松手与回放约束不变；不改变公共车辆动力学。
- 验证：正在构建与测试，若jerk/认证失败不得启动实际试验；不预先写通过。
- 影响与边界：保持前向固定目标感知、手动四相机、原场景及旧源码不变。未知拒绝和.58m认证包络不变。
- 回滚/备份：将paper_proposal_command_accel恢复1.8即回到77配置，所有原始数据保留。
- 后续事项：冻结重复确认收益，未提交推送。


### 2026-09-30 02:31 — [EGO1P1] 按旧版48列基准归一化像素净空奖励

- 状态：继续实测，全面领先未通过。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点/主launch、性能记录。
- 76结果：安全到达70.2833仿真秒/73.0215墙钟，jerk2.3889、停车.25秒，未胜出。已再次关闭局部路径修正；保留有效选定目标的记忆修复。
- 原因与修改：旧user_ego源码默认48×36且clearance_reward=.018；目标/连续性成本按图像宽高归一化，但净空使用原始像素距离。新版64×48以64为奖励锚点，对相同角域空隙有额外约33%偏好。77改为48/当前宽度归一化，即当前64列的有效奖励.0135，保留旧版对应角域的偏好；并非降低真实机体、安全半径或制动要求。
- 验证：真实旧源码只读核对，未修改；77正在构建/回归与试跑。整体到达、碰撞与时间仍待实测，不能仅凭奖励归一化宣称性能提升。
- 影响与边界：paper_goal_front_only继续仅用于固定目标；手动4路订阅保持。公共地图、协议、.48物理/.58认证不变。
- 回滚/备份：将归一化锚点48恢复64即还原旧偏好，全部冻结与探索数据保留。
- 后续事项：冻结前必须重复验证，未提交推送。


### 2026-09-30 02:26 — [EGO1P1] 修正局部候选与下一帧目标记忆不一致

- 状态：性能未达标，继续验证。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：depth_angular_controller_node.cpp及性能记录。
- 75结果：到达69.85仿真秒；完整数值见iteration75_analysis.json，尚未胜出，不作为最终成绩。
- 原因与修改：局部路径覆盖proposal后，旧逻辑仍把被覆盖的primary目标写回previous_goal_direction_world_，下一周期的滞回目标与实际选定候选不同；goal_valid=false时也把默认像素写成“已有目标”。76改为只有有效候选才保存其真实世界方向，无效候选清空目标有效标志；可视化目标像素同步所选方向。保留15°候选修正限制与完整运动认证，不引入新速度合成。
- 验证：正在构建/实际运行；不能凭代码原因直接断言已经解决平均时间或稳定性。原始75和全部失败数据保留。
- 影响与边界：候选连续性状态变化，实际机体/物理/未知约束、公共协议与旧版源码不变。
- 回滚/备份：frozen_v3源码及本轮逐次参数/二进制SHA；原旧赋值可由冻结控制节点恢复。
- 后续事项：复测并冻结重复，未提交推送。


### 2026-09-30 02:23 — [EGO1P1] 前向固定目标实测提速，局部路径改为受限修正

- 状态：优化中，全面领先尚未通过。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点/主launch、性能证据与记录。
- 74实测：安全到达69.4833仿真秒/70.4196墙钟、jerk1.8159、停车.25秒，扫掠重叠0。相比73同系列四路消费墙钟73.2331秒有改善，但单次仍未全面优于旧版。控制回调到发布约1.296ms，实际命令59.32Hz；不以名义60Hz写成所有周期满足。
- 模式验证：真实ROS独立域87启动同一节点，use_fixed_goal=false时实际创建4路订阅，true时1路；两项通过，日志见goal_perception_policy_probe.json。无Isaac导航并行干扰该测量。
- 75修改：局部路径仅在深度主建议偏离期望超过2°时求取，且只能替换成与主建议夹角≤15°的局部修正；主建议无效时不能由局部网格单独接管。保留成熟深度目标选路的绕障支路，尝试利用局部路径缩短绕行；所有建议仍独立完整认证。此为工程候选筛选，非论文原始公式；没有放宽安全条件。
- 验证：75正在构建/实际运行，未提前写通过；物理上限、协议、四相机发布和旧源码不变。
- 回滚/备份：paper_local_path_proposal=false可恢复74候选链，逐轮参数与源码冻结备份齐全。
- 后续事项：实测与冻结重复，未提交推送。


### 2026-09-30 02:19 — [EGO1P1] 验证固定终点前向感知策略，保留手动四相机

- 状态：实验中，目标尚未通过。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点/主launch、性能证据及记录。
- 73结果：安全到达69.4333仿真秒/73.2331墙钟，jerk1.6972、停车.25秒；接纳20→10Hz使证据维护约3.5→1.9ms/周期、墙钟减少约.88秒，不能声称完整运行更快。所有完整机体扫掠重叠0。
- 74修改：新增显式paper_goal_front_only，仅在paper_enabled且use_fixed_goal同时成立时只订阅前向深度；手动模式仍按horizontal_360配置四相机。适用于平台机头随固定目标方向转动的自动导航；当前共同平台已有该规则。仍使用同一完整机体、历史自由体积、完整过渡和制动认证，不能把未观测侧后区域当自由；机头不对准时可能停车。相机发布配置及公共物理不变；与原user_ego仅消费前相机类似，算法消费策略必须公开披露，不能伪称四路当前证据仍在使用。
- 验证：73实际完成且离线审计通过；74正在构建/试跑，未写通过。旧源码未修改，公共起终点/场景/判据不变；不是改变旧算法来制造比较优势。
- 回滚/备份：参数设false恢复全部四路订阅；保留全部冻结及逐轮记录。
- 后续事项：检查完整认证和到达/计算耗时，再决定是否采用；未提交推送。


### 2026-09-30 02:15 — [EGO1P1] 指令斜坡回归通过，降低重复静态观测计算继续验证

- 状态：继续优化，尚未全面优于旧版。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主launch、intent_response_check版本断言、性能记录。
- 72实测：安全到达69.35仿真秒/74.1092墙钟，jerk1.5671、停车.25秒、路径133.4174m、净距.1962m、扫掠重叠0。物理加速度仍1.2m/s²；平滑性改善，速度未通过。
- 验证：第一次核心17/18，intent_response仍断言旧G格式失败；更新至H格式后18/18通过，其余累计意图/重放等断言保留。72未启动在失败测试之后；实际运行使用通过回归的二进制。
- 修改：73仅将静态深度完整帧接纳20→10Hz，控制60Hz及原60Hz相机发布不变；完整新像素矛盾检查不抽样，未接纳图像不刷新有效时龄；max_depth_age=.3及全机体认证不变。旨在减少约3.5ms/控制周期的证据维护开销，实际收益待运行，不假定必然改善。
- 回滚/备份：全部逐轮参数和冻结源包；恢复paper_depth_processing_rate=20可还原72配置。
- 影响与边界：旧源码、公共场景/物理/协议不变；当前关闭局部路径覆盖，前馈关闭。只适用于已声明的静态场景证据前提，不能外推动态障碍性能。
- 后续事项：验证实际耗时与稳健性后再冻结重测，未提交推送。


### 2026-09-30 02:10 — [EGO1P1] 撤回局部宽度试验，区分指令斜坡和真实动力学

- 状态：全面领先未通过，持续优化。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：paper_guidance.hpp/cpp、paper_replay.cpp、控制节点/主launch、motion_consistency_check、局部路径三个文件、性能证据及记录。
- 71结果：安全到达72.8667秒/77.6487墙钟，jerk3.8069、路径133.8227m、停车.55秒、COMPUTE_DEADLINE累计.4333秒，退化。局部宽度试验已按v3源码包精确恢复三个文件，拒绝实现另存rejected_visible_width_local_path.cpp。当前关闭局部网格路径覆盖，回到已有冻结5/5的深度目标建议主链；当前核心/帧率已有变化，不能直接继承v1成绩，必须重新实测。
- 72修改：速度指令斜坡与真实加速度分开配置。指令斜坡上限1.8m/s²、jerk6，真实plant和完整运动/制动模型仍1.2m/s²；意在减少“指令先慢变、机体再慢响应”的额外延迟。模型反演/前馈继续关闭，输入松开仍立即输出零。V17(EGOPAPRH)保存新参数且兼容3–16；新增真实加速度不变、完整认证、指令jerk/限速、重放和松手回归。
- 验证：71完整18/18后实际性能退化，不能只用合成分支选择通过认定有效。72正在构建，未写成通过。固定公共协议、旧源码与地图不变。
- 回滚/备份：frozen_v3源码包、旧冻结二进制、全部探索参数与拒绝版本；paper_proposal_command_accel=0恢复原指令斜坡。
- 后续事项：先验证72，再决定冻结配置；不以单次替代重复，不提交推送。


### 2026-09-30 02:04 — [EGO1P1] final_v3失败如实保留；候选增加观测空间宽度代价

- 状态：未达到全面领先，继续修正。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：local_depth_path.hpp/cpp、local_depth_path_check、性能目录、方法与验收文档。
- 正式结果：final_v3新版4/5安全到达，成功均值69.1417±.9295仿真秒/73.6159墙钟秒，1轮240秒超时；含失败惩罚均值103.3133秒。旧user_ego5/5，69.0767±.6221仿真秒/69.5436墙钟。不能用失败轮静止稀释后的jerk1.8534声称更稳定；本组不通过且不得删除失败。
- 一致性：10轮完整协议，无探索中止，114源码SHA未变，公共SHA与final_v1相同，两版各只有一组二进制/launch。源码包、二进制与全部原始数据冻结保存；后续修改不追溯计入v3。
- 失败审计：停在(28.0807,3.0941,1.4822)，.58m当前机体可认证，但加.005m余量不可认证；沿意图只有约.0015m认证前缀，后向也仅约.058m。上一命令为零，因此motion_scale0/.5/1均为刹停检查，不能当作能继续飞的证明。证据支持困在认证边界，不能说是碰撞；该轮保守净距仍.2096m。
- 修改：71候选的已观测中心/障碍网格增加距未知或阻塞单元的Manhattan距离代价，权重.03，优先较宽的可见绕行支路；直达中心线畅通时不额外偏离。它只是候选排序，不是假冒欧氏净距或完整体积认证，不改变未知拒绝、真实机体/制动模型。新增窄观测边缘与宽支路对照，待构建实测。
- 验证：v3核心18/18、平台15/15；独立扫掠审计所有新版轮重叠0。71尚未运行；V16、帧处理20Hz/控制60Hz等均保持。正式计时中未构建/测试。
- 71补充验证：首次局部路线回归失败，等代价分支在世界旋转后被微小浮点误差翻转；增加1e-10等价容差后，完整18/18通过。宽观测支路夹具中，长度优先方向约(.862,-.359,-.359)，宽度偏好为(.862,-.359,+.359)。实际导航正在运行，不能用该合成结果替代实测。final_v3另完成17帧同冻结二进制重放一致；最初失败帧因环形缓存覆写不在末尾16帧中，未声称已重现最初瞬间。
- 回滚/备份：完整frozen_v3_sources.tar.gz及所有旧冻结数据；局部visibility_weight设0恢复原候选代价。
- 后续事项：验证候选宽度修正，不以一次成功宣称全面胜出；未提交推送。


### 2026-09-30 01:48 — [EGO1P1] 冻结60Hz控制/20Hz观测候选，完整重复验收

- 状态：验收进行中，全面优于旧算法尚未达成。
- 目的：不以单次最佳值代替重复稳定性，保留失败证据。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主launch、paper_guidance.cpp、性能证据及记录；EXPLORATION补录55–68。
- 实测：69关闭补偿、jerk6、内部两线程、60Hz控制/20Hz完整帧处理，安全到达69.40仿真秒/74.4248墙钟秒，jerk1.6671、停车.25秒、路径133.0732m、净距.1925m、重叠0，平滑性改善但仍未更快。70仅改30Hz帧处理，在34.2667秒因持续停滞中止；失败惩罚240秒，不算快速到达。已恢复69参数；不能声称高观测频率必然改善。
- 验证：69构建、核心CTest18/18通过；70运行失败及独立轨迹分析保留。冻结v3将重新构建、核心和平台回归后进行5次完整240秒上限测试，且与user_ego交错重测，不使用探索早停。
- 影响与边界：原地图、旧源码、共同物理及相机发布频率不变，安全认证未减弱。仅固定路线验收，不推断任意地图优越性。
- 回滚/备份：任务起始tar、frozen_v1/v2和所有探索日志；69参数可直接由保存的parameters恢复。
- 后续事项：如重复失败必须报告，不能剔除失败轮或写全面通过；未提交推送。


### 2026-09-30 01:39 — [EGO1P1] 撤回高jerk前馈，分离观测处理与控制频率继续验证

- 状态：性能目标未完成，继续实验。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点、paper_guidance.hpp/cpp、paper_replay.cpp、motion_consistency_check、主launch及性能数据。
- 实测：62路线保持在62.25秒停滞中止，已撤回；63独立执行器70.0333秒/77.5827墙钟/jerk2.6352，已撤回；64软路线方向69.05/74.8497/2.1893；65原生候选采样73.1/80.0726/3.5575，已撤回采样；66直接模型补偿68.65/75.8556/4.1972；67独立参考前馈68.4333/75.4545/4.5897。66/67均速度略快但平滑性退化，不能算通过。68前馈下60Hz控制、30Hz深度处理到达68.5167/73.7928/3.4688，仍不通过平滑性；上述成功轮机体扫掠重叠均0。
- 修改：67独立积分速度参考、加速度/jerk约束和响应前馈及发布覆盖复位，V16保存参考状态、兼容旧重放；理想同步单元测试通过不等于真实异步系统平滑。68新增可配置控制频率和按图像时间戳限流；未接纳帧不认证空间也不刷新证据时龄。原相机发布仍60Hz、所有接纳帧原始像素仍完整安全检查。69关闭前馈恢复jerk6，保持60Hz控制，将接纳频率30→20Hz、内部OpenMP4→2，减少与仿真CPU竞争，待实测。
- 验证：67完整CTest18/18；68构建、相关2项通过且实际完成导航。69尚未运行，不提前写通过。真实67高jerk否定了仅凭理想plant单元测试的性能判断。
- 影响与边界：旧算法、公共物理/相机发布/地图/起终点不改，完整.58m体积与制动认证不放松。新限流属于算法计算策略，必须在比较中披露；不声称处理了全部60Hz帧。未提交推送。
- 回滚/备份：任务起始备份、冻结v1/v2、逐轮参数与二进制SHA；拒绝执行器/采样/保持代码单独归档。
- 后续事项：实测69，再冻结重复；全面领先尚未通过。


### 2026-09-30 01:06 — [EGO1P1] 验证惯性前视与历史几何加速；仍未通过全面领先

- 状态：持续优化，未完成全面领先验收。
- 目的：减少绕行和重复几何检查，保留所有退化实验。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：local_depth_path.hpp/cpp、paper_guidance.hpp/cpp、控制节点/主launch、局部路径/观测测试、PAPER_IMPLEMENTATION、实验目录及记录。
- 结果：56去局部回轴惩罚安全到达68.9833秒/74.5678墙钟/jerk2.0446；57仅缩候选余量至.10在33.8167秒停滞中止。58候选障碍沿实测速度反方向延长两个响应时间，保留.10候选余量，安全到达68.70/75.0447/jerk2.1705、路径132.0478m、最小净距.2195m、重叠0；单次略快于旧均值，但墙钟和平滑性未通过。59将历史16→4后46.7667秒停滞中止，已恢复16。60原始光学坐标点组+jerk4到达70.0833/76.8484/jerk3.0747、重叠0，已恢复jerk6。61增加近等代价路线保持，到达70.0/77.0268/jerk2.9171，未胜出。
- 修改：点组先在输入光学坐标中包围，再一次变换到旧相机，避免两次轴对齐放大；只剔除确定不冲突组。62新增小投影矩形的完整深度上界检查，最多128像素逐个检查，任何未决组仍逐个原始点检查；不使用地址缓存，不抽样。局部路线代价采用剩余弧长，意图变化或完整运动失败取消保持；手柄改向阈值以被保留路线的原始意图为基准，避免累计变化漏重设。
- 验证：60第一次编译Eigen条件表达式存储类型不一致失败，改为显式赋值后构建、18/18通过；再加实测迎障速度提前转向及无效时间测试通过。62构建、完整18/18通过，含投影区域中单个深像素不得被批量排除、60组独立穷举比较。61已运行二进制尚是逐次更新路线意图基准的版本；累计基准修正属于62，不能追溯算给61。62实际导航正在运行。
- 计时证据：历史维护总耗时约7ms/周期，记录到达几何约.02ms；细分每相机球处理约.4、胶囊约.3、历史检查约1.1ms。不能把记录到达几何说成主要瓶颈。减历史导致的失败不作为更快成绩。
- 影响与边界：公共物理、场景、起终点、.48机体/.58认证及旧源码不变。惯性候选与路线保持是工程扩展，最终完整实际过渡和制动检查仍授权唯一三维命令。未提交推送。
- 回滚/备份：任务起始备份、冻结v1/v2二进制/源码SHA和逐轮参数/轨迹完整保留。
- 后续事项：验证几何加速与路线连续性，再冻结重复统计；全面领先尚未达成。


### 2026-09-30 00:45 — [EGO1P1] 认证记忆回归通过；历史采样改为独立时钟，继续比较

- 状态：部分完成，全面领先未通过。
- 目的：保持已认证自由体积并消除历史淘汰引起的重复处理；分别检查行程和计算开销。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：paper_guidance.hpp/cpp、控制节点、主launch、local_depth_path.cpp、continuous_recovery/observed_space测试、实验目录/记录。
- 结果：53完整CTest18/18，安全到达69.05仿真秒/76.4696墙钟秒/jerk2.7092，原4.7秒停车消失；21.17秒执行端一次零命令脉冲仍增加jerk。54历史32→8时安全到达70.7167/76.6231/jerk2.6946，历史淘汰超过2000次，未胜出。55采样时钟独立于淘汰/撤销并恢复16幅历史，安全到达69.1333/74.8157/jerk2.0764，扫掠重叠均0，仍未超过user_ego均值。
- 修改：历史采样时钟不再随覆盖淘汰或矛盾撤销重置；所有新帧仍逐点检查冲突。查询时先检查同一几何联合中的最近已认证球，未放松包含判据。第56轮撤销局部路线中间点到意图轴的额外距离惩罚，仅保留长度、转向和终点距离代价，以减少局部回轴再绕开的弯路；增加入口证据/记录耗时统计定位计算热点。
- 验证与失败：55首次18项中observed_space旧夹具假设撤销即重置1000秒采样间隔失败；改为0.5秒独立采样夹具，并增加撤销后0.2秒不得重采、反复淘汰不突破时钟测试，18/18通过。56构建及局部路线/观测/连续恢复3项通过，完整运行进行中。54统计首次环境变量拼写错误导致NumPy ABI绘图失败，正确PYTHONNOUSERSITE=1重跑成功；原数值及失败未掩盖。
- 影响与边界：旧代码、地图、共同物理/起终点不变，完整安全认证、未知拒绝和0.58m包络不变；新几何记忆限静态世界。尚未冻结新一组平均值，无提交推送。
- 回滚/备份：任务初始备份、final_v1/v2冻结数据及所有探索参数保留。
- 后续事项：继续比较并冻结重测；阶段结果不能写为最终达标。


### 2026-09-30 00:33 — [EGO1P1] 定位认证空间交接丢失并增加静态已证体积保留（验证中）

- 状态：实施/验证中；全面优于旧算法仍未通过。
- 目的：解决第52轮起步后约1.08秒的NO_REGULAR_FIELD/COMPUTE_DEADLINE，避免仅靠继续调参数。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：paper_guidance.hpp/cpp、paper_replay.cpp、控制节点/主launch、continuous_recovery/intent_response测试、local_depth_path.cpp、实验目录和记录。
- 前轮结果：51可见短前缀74.8833秒停滞中止，转向权重.4也未通过局部路径单元测试，已恢复固定8m/权重.04；52历史观测+10Hz世界路径跟随安全到达69.95秒/墙钟75.5927秒/路径131.9006m/jerk2.4578，扫掠重叠0，仍不全面领先。52全量CTest18/18通过。
- 因果证据：52的4.7秒失败快照含13幅历史、4幅当前及83个已接受运动的认证球。原证据连完整刹车都拒绝；只将未撤销的83个认证球加入几何查询后，完整刹车可认证。审计本来已包含历史，不能误称历史没加载。此时上一已发布命令为零，所以scale0/.5/1测试都是停车，不证明原速继续可行。
- 修改内容：新增显式paper_retain_certified_volume，仅在已有静态场景/已验证运动记忆/零历史增长前提下复用上一条已接受完整运动检查产生的几何球；并非把预测的机体位置当观测。球必须原先有完整体积包含证明，所有新原始点仍撤销冲突球，不扩张未知。V15(EGOPAPRF)保存开关并兼容旧版本；局部路径加入非有限/远端范围检查。
- 验证：新增未来认证体积保持、未知不扩张、多次继承不生造空间、新障碍撤销、旧模式及重放测试；正在构建，未提前写通过。审计第一次构建因unique_ptr类不可复制失败，改为临时追加/恢复向量后构建成功；直接执行无执行位Shell失败，使用bash主入口成功。
- 影响与边界：旧源码、地图、公共起终点和物理协议不变，完整运动/制动检查不削弱；实验路线候选属工程扩展，非论文逐式原样复现。没有提交或推送。
- 回滚/备份：主launch将paper_retain_certified_volume改false恢复旧语义；任务初始完整备份和冻结版证据保留。
- 后续事项：先回归与单次探索，再冻结重复比较，不能以单次速度证明全面领先。


### 2026-09-30 00:12 — [EGO1P1] 引入深度局部短路径候选；领先目标尚未通过

- 状态：继续研究优化，未完成用户要求的全面领先验收。
- 涉及文件：新增local_depth_path.hpp/cpp及local_depth_path_check；CMake、控制节点、主launch、实验目录和记录。
- 撤回项：46同侧目标收近69.2667秒但jerk3.4537，关闭投影；47图像并发71.5333/jerk3.4373且出现349ms峰值，恢复两线程/原传感组。48真实夹角/4秒前视/静态.55m组合81.0333/jerk5.5668，恢复3秒、原代价、.58m完整包络。当前没有使用这些退化参数。
- 新方法：只用当前真实深度，在随q旋转的局部三维网格中比较前方最多8m的路径长度和转向代价；只作为未经完整认证的方向建议。最终仍先做已知体积走廊检查，再执行原完整实际过渡/制动认证，输出一条三维速度；在线不读全图路线。这个路径选择是工程扩展，不是论文原公式的逐项复现。
- 验证：49首次编译缺Eigen/Geometry链接失败，补齐后构建及3项相关回归通过；空旷直行、未知拒绝、有限障碍绕行、世界坐标旋转/平移通过。49实际候选约9成被采用，安全到达69.0833秒、墙钟75.2529秒、路径132.745m、净距.1853m、jerk2.1272、扫掠重叠0，尚未快于旧均值。50把候选额外网格膨胀.18降至.05并设jerk6后52.2秒因停滞中止，保留失败且惩罚均值240，不计到达；已恢复.18。
- 当前51：转向代价加大，并允许遮挡时使用最深可见短前缀，短前缀本身不授权执行；完整制动走廊不足时仍不能走。相关测试/实测进行中，尚未写成通过。原始数据和所有被拒配置保存。
- 边界：旧源码、地图、起终点和公共物理/到达协议不变；原先final_v1三版各5次仍是有效冻结对照，其未全面领先的结论不被探索单次结果覆盖。未提交推送。


### 2026-09-29 23:49 — [EGO1P1] 完成额外对照并撤回无收益加速，继续验证连续方向

- 状态：性能优化继续；此前final_v1/final_v2冻结组仍未满足全面领先，不以单次最优替代均值。
- 涉及文件：深度核心/控制节点、paper_guidance/paper_replay与头文件、主launch、观测/连续恢复/方向回归、实验目录及记录。
- 结果：37批量深度排除69.65秒/jerk2.2221，38奖励.014为69.3667/2.2313，40奖励封顶6为71.0167/2.9549，41分层点组为69.3833/3.1299，42缩短保持为72.6833/3.1290，43历史16幅为69.4/2.1072；均安全到达但未获总体收益。39主动中止：复核发现原始地址缓存可能在观测释放后复用，修正为持有观测所有权才重跑40。复杂深度缓存、分层分组和临时计时最终均撤回，拒绝实验代码另存rejected_history_tree.cpp。
- 修改：历史容量改为显式参数，默认/当前32，试验16的近场关键观测保留通过；V14(EGOPAPRE)记录容量并兼容V3–V13。候选亚像素二次细化只在同一自由3x3邻域内进行，旧目标保持时不再重新取整；相机/机体原点统一保留。44细化但保持仍取整为70.6167/jerk2.6017；45补齐连续保持并试验静态时龄增长0为69.3833/jerk1.9448，时间仍未胜出。现恢复.58m全包络并开始46：在同一绕障侧将候选向q收近，额外2像素空余且须完整观测走廊重认证，最多4次查询，最终运动/制动仍独立认证。
- 验证：40/41/43完整CTest17/17；42相关4项通过。44初始解析测试缺少真实视场阻塞边界，目标落在图像边缘导致细化不启用，补齐边界后3项通过；45相关4项通过。46尚在构建/试验，不能写通过。所有运行的逐段.48m机体扫掠审计均为0重叠；这仅指以上新版成功轮，不包括旧EGO1P0。
- 边界：旧源码/场景、输入、物理速度和到达协议不变。最终统计仍需冻结复测；当前保留所有失败和中止数据，未提交推送。


### 2026-09-29 23:24 — [EGO1P1] 复测拒绝退化参数；统一候选坐标并验证等价历史加速

- 状态：仍在优化，尚未达到同时更快、更稳的验收目标。
- 涉及文件：深度核心头文件/实现、控制节点/主launch、paper_guidance、guidance_composition/observed_space测试；基准目录及项目/统一日志。
- 结果：34净空奖励封顶试验95.72秒停滞中止；35历史间隔4秒到达70.333秒/jerk2.2737。final_v2仅历史间隔2秒的五次完整复测全安全到达，均值69.3667秒、标准差.7827、墙钟74.4647秒、jerk2.1210，稳定性不如final_v1，已恢复1秒间隔。这些失败/退化不删除，不混入冻结版均值。
- 修改：候选碰撞锥把观测点从拍摄相机原点平移到当前机体中心，保持光学轴不变，补偿安装偏移和取帧后的平移；最终物理视锥与完整制动认证不变。36安全到达69.35秒、墙钟75.7795秒、jerk1.9727，单次不能证明总体胜出。另加历史128点组的保守投影矩形深度上界排除，只跳过逐点检查必然全不构成冲突的组；其余仍逐点检查。
- 验证：36首次测试编译缺Eigen/Geometry失败，补齐后构建和3项相关回归通过；旋转/平移/取帧后移动及原始点两条路径的平面净距与解析解一致。37全量17/17通过，新增60组独立逐点真值对照和原有5个分组边界单点入侵覆盖；37实际导航正在运行，未写成通过。
- 边界：旧源码和地图不改、未知拒绝、机体.48/认证.58m、速度2m/s及公共到达判据不变。没有提交推送。final_v1/final_v2参数、源码SHA、二进制及所有原始轨迹已保留。
- 后续：验证加速的实测收益，定位额外绕行，冻结后再重复验收；不能以单次最优取代平均结果。


### 2026-09-29 22:55 — [EGO1P1] 完成15轮冻结对照；全面领先未通过，继续针对性优化

- 状态：部分完成；对照实验完成，性能目标未全部通过。
- 目的：报告真实平均时间与稳定性，继续处理测量明确的计算和绕行开销。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：NAVIGATION_BENCHMARK.md、实验目录；主launch、depth_angular_core.hpp/cpp、controller_node与guidance_composition测试；WORK_LOG/HISTORY。
- 验证：final_v1三版各5次交错、完整240秒超时，源文件/公共端SHA及每版二进制/launch完全一致。user_ego5/5到达，69.003±.474仿真秒、69.646墙钟秒；EGO1P15/5，69.297±.071、75.643秒。EGO1P0全超时且0.48m包围球离线相交，不能把保护计数0写成安全。新版最小净距.1534m，旧user_ego.0294m；新版停车均值.2167 vs .4267秒、jerk2.0504 vs2.0123。新版导航慢.43%、墙钟慢8.61%、jerk略高1.89%，未通过全面领先。冻结版三包构建、CTest17/17、平台15/15、语法和diff检查通过。
- 后续修改：第33轮历史保留采样1→2秒，仍每帧检查所有新原始点，安全到达69.25仿真秒/74.218墙钟秒/jerk2.0707；计算开销稍降。第34轮进一步将无上限的额外净空奖励封顶于2个基准像素，保留靠近障碍处的奖励斜率、.58m认证及真实夹角，避免越宽越值得偏离q；已构建且相关3项回归通过，正在运行，尚未得出成功结论。
- 影响与边界：固定组及失败原始数据不覆盖，后续版本另编号。没有扩大已知空间、减少新障碍检查或修改旧算法源文件/场景；当前工程混合主链不是论文完整原样复现。图像预览工具因环境沙箱故障失败，改用只读方式检查已生成图像，不影响运行证据。
- 回滚/备份：19:29整包备份；final_v1完整参数与frozen_sources.json、frozen_v1_launch.py及控制器/回放二进制保存，可还原该冻结组。
- 后续事项：验证有界净空偏好及历史去重；只有新版本重复数据才可支持新性能结论，未达标必须继续明确标注。


### 2026-09-29 22:15 — [EGO1P1] 冻结重复对照版本；所有探索保留

- 状态：部分完成，进入五轮实际对照；尚未证明快于user_ego。
- 目的：停止混用不同版本单次成绩，测量可复现的均值与稳定性。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：控制节点/主launch、深度核心/认证核心及回放、测试、README/PAPER_IMPLEMENTATION、实验目录和工作区记录。
- 修改内容：冻结64×48保守深度候选、旧版像素代价、净空奖励.018、左偏.012、保持.35秒/权重.4、前视3秒、参考响应.22秒、矢量jerk8；候选走廊与完整运动/制动证明、有限终点、发布覆盖同步、精确投影和历史分组几何加速保留。原始稀疏点、真实夹角、短走廊意图抢占和模型反演默认关闭，均曾测试但未优于稳定配置。诊断目标改为实际候选矢量，避免相机截断导致显示错误。
- 验证：29原始稀疏点69.47秒停滞中止；30高分辨率69.433秒/jerk2.2462，31去左偏69.55秒/jerk2.2915；32完整短走廊意图优先在95.48秒停滞中止，淘汰。29构建后17/17完整回归通过；冻结版全量回归正在执行，不能提前写通过。原始user_ego44个、EGO1P0 46个文件SHA复核全部未变。
- 公平性边界：保留两版旧算法硬编码的2.5m/s²规划制动假设（无法通过公开ROS参数覆盖），共同物理执行仍1.2m/s²；这是现有算法同平台适应性对照，不能写成内部安全假设完全相同。`speed_brake_accel`是旧命令整形参数，不能误称为规划模型覆盖。此项未暗中修改或调低旧算法成绩。
- 影响与边界：仍.48m机体/.58m认证、未知拒绝、相同地图起终点与2m/s。新版并非完整论文原样主链；旧源码和地图未修改，未提交推送。
- 回滚/备份：19:29备份；冻结源码SHA见frozen_sources.json，各探索完整原始轨迹和参数保留。
- 后续事项：三版各五次交错运行，240秒完整超时，不启用探索早停；如未同时更快更稳定，必须报告验收未通过。


### 2026-09-29 22:01 — [EGO1P1] 三维方向度量与原始候选优化（阶段）

- 状态：实施中；最终均值和优于旧版的验收尚未完成。
- 目的：减少绕障中段的折返，保持完整观测/制动认证并降低历史检查开销。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：depth_angular_core.hpp/cpp、paper_guidance.hpp/cpp、paper_replay.cpp、controller_node、主launch；guidance_composition/observed_space/motion_consistency/paper_guidance测试；README、PAPER_IMPLEMENTATION及实验记录。
- 修改内容：V13独立记录参考响应时间；三维候选用真实射线夹角衡量接近意图，默认旧像素代价接口保持兼容。历史点按128点连续组建立保守AABB，只剔除整个组位于旧视锥外的情况，余下每个原始点仍检查。正在试验原始点每8个抽取一个仅生成方向候选，完整保守深度和所有原始点仍用于执行授权及矛盾撤销，不将稀疏候选当自由证据。
- 验证：25（响应.10/前视2.5）69.383秒、jerk2.3365；26（净空奖励.004）111.767秒/停车14.083秒/jerk9.3257，撤销该配置；27真实夹角/前视3为69.60秒、jerk2.3308；28前视4为69.40秒、jerk2.149。均到达且新版离线扫掠重叠0。29当前恢复响应.22、前视3、净空奖励.018并加入原始候选，运行尚未结束。新17/17完整回归通过，包括5个跨原始点分组边界的小障碍撤销位置和长宽比不等视场的真实转角排序。
- 失败与更正：27首次编译因局部替换误落入nearestFreePixel失败，其后执行的旧测试无新代码验收意义；修复后重构建和相关2项通过。29首次编译因ROS整数参数返回类型不匹配失败，显式转换后构建及17/17通过。早期“完成轮重叠0”只适用于新版，旧EGO1P0 pilot离线扫掠存在负净距（约-.054m），不能推广到三版；最终须按统一独立审计判安全到达。
- 影响与边界：.48m机体、.58m认证、安全未知拒绝、2m/s上限、1.2m/s²及物理.22秒保持；当前混合候选工程路径与原论文差异已显式写入说明。旧源码未修改，无提交推送。
- 回滚/备份：沿用19:29备份，每轮参数和二进制SHA/原始日志完整保留。
- 后续事项：确定稳定有效版本后冻结做五轮交错复测，均值、成功率、平滑度及墙钟开销全部报告。


### 2026-09-29 21:45 — [EGO1P1] 固定目标候选收敛与矢量响应调优（阶段）

- 状态：实施中，仍未完成五轮最终验收，也未证明快于user_ego。
- 目的：减少候选方向与完整安全执行不一致、远端障碍干扰终点收敛及重复参考滤波。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：depth_angular_core.hpp/cpp、paper_guidance.hpp/cpp、paper_replay.cpp、depth_angular_controller_node.cpp、主launch；guidance_composition/motion_consistency/paper_guidance测试；主Shell、benchmark launch/driver/统计与实验目录。
- 修改内容：安全目标最多12次完整走廊验证后直接作为方向候选，target-only模式不计算也不伪造调和场（goal_valid与field_valid分开）；最终实际过渡/制动仍完整认证。固定终点限制规划距离，目标自然方位变化不作为手柄突变重置。增加精确证据AABB快速剔除；统一OMP_WAIT_POLICY默认PASSIVE。V12记录模型整形开关与jerk，正在V13中独立记录候选响应时间，物理tau仍.22。最新探索前视2.5秒/参考响应.10秒，全部速度、机体、认证和加速度边界不变。
- 验证：iteration17/18停滞中止59.32/68.25秒；19到达70.82秒；20到达69.25秒/jerk2.0566/净距.1571m，为当时最优但仍慢于user_ego68.75秒。模型反演21/22为69.33/69.65秒且jerk2.55/2.83，未采用默认；23静态帧龄余量去除至.55m后79.07秒，已恢复.58m；24前视2秒在68.12秒因停滞中止。完成轮离线碰撞重叠0、外部保护0。完整CTest17/17在target-only之前通过，target-only相关2项已通过；当前V13修改正在构建，尚未验收。
- 校准边界：原始静止起点有限深度与独立voxel+真实地板(-.2m)交线最大误差<.1mm，只支持该样本；初次错误使用地板0的统计单独保留，不能视为传感器误差。未据此宣称全运动误差界，原.03m深度余量及.58m总包络保留。
- 影响与边界：主链变为“安全深度目标候选+完整运动认证+矢量整形”，原论文调和场/流盒仍作失败后备，不能称论文公式完整原样实现。旧算法源文件未改，探索版本不同不能混算均值；最终须冻结版本、独占计算资源重跑。没有提交/推送。
- 回滚/备份：沿用19:29完整备份和每轮参数/二进制SHA；失败试验全部保留。
- 后续事项：完成针对响应延迟的试验，冻结后执行各五轮并报告真实均值、成功率和稳定性。


### 2026-09-29 21:10 — [EGO1P1] 公共回调修复、等价几何加速与候选/执行失配诊断（阶段）

- 状态：实施中，未达到同时优于user_ego的目标，未做最终五轮验收。
- 目的：区分虚假失联、重复速度限幅与真实已知自由空间边界停车。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：Isaac运行器/navigation_benchmark.py、benchmark launch/driver/统计；depth_angular_core.hpp/cpp及节点；depth_geometry/guidance_composition/motion_consistency/protocol测试与CMake；实验目录和工作区记录。
- 修改内容：公共端每帧最多8个非阻塞回调，防诊断饿死Twist新鲜度；碰撞锥用精确球投影范围剔除不可能相交的像素，窄相仍为原二次交公式。候选安全角段可行时收回旧目标，固定终点速度考虑执行响应。新增direction_proposal_only接口，候选不再经过旧单相机速度限幅，最终完整运动认证唯一授权执行。第17轮改用实测速向及双tau响应预览，尝试解决虚拟方向先转、机体仍滞后的问题。探索可显式启用30秒停滞中止，标ABORTED_EXPLORATION_STALL而非到达/超时；最终默认0、仍完整240秒。
- 验证：第12轮统一回调修复后user_ego到达68.75s/jerk2.0516，新版69.8667s/jerk2.6148；新版最小净距.1472m、停车.2167s。第13轮退化至81.9667s；第14–16轮均240秒超时，全部保留。真实停车审计显示机体包络已知，但向前认证长度为0，部分后向只有厘米级可认证段。7+16个失败快照重放一致。精确投影与穷举随机500点×4半径×2内参逐射线对照及原固定夹具通过；完整CTest17/17通过（含协议首4例），追加探索中止后协议5/5通过。第17轮运行中，未完成。
- 更正：第14/15轮传入native_points接口时，当前观测此前已清空原始点指针，实际仍使用保守缩图，不能记为原始点方案实测。第16轮修复指针保留并打印真实点数，确认原始点实际启用，但计算变慢且仍停在相近位置；第17轮恢复保守缩图候选。原始点只用于候选试验，安全证据一直是保守深度、全部新鲜点矛盾撤销与运动认证。
- 影响与边界：旧版源码、地图、速度/动力学、.48/.58m尺寸、未知拒绝及计算截止不变。候选方向/完整证明分工属于工程扩展，不能称论文原样实现。失败样本的低全程jerk受长时间静止影响，不能据此宣称更平滑。
- 回滚/备份：沿用19:29备份与各轮参数/二进制SHA；公共执行端源码SHA从第15轮起保存；没有提交推送。
- 后续事项：继续验证状态匹配、冻结有效版本后做全量回归及各5轮公平统计；若尚未胜出必须如实说明。


### 2026-09-29 20:45 — [EGO1P1] 深度建议加完整运动认证；首次全程到达，性能仍在优化

- 状态：实施中；未完成最终五轮对照，尚不能宣称优于user_ego。
- 目的：解决原完整角域构造反复超时/停车，同时保持三维包络与制动认证。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：paper_guidance.hpp/cpp、paper_replay.cpp、depth_angular_core.hpp/cpp、depth_angular_controller_node.cpp、主launch、motion_consistency/intent_response测试、Isaac运行器、benchmark统计及协议。
- 修改内容：沿用旧版深度调和场生成候选，再由新版完整观测空间认证实际执行及制动全过程；增加矢量加速度/jerk整形，发布覆写同步状态。观测联合细分3→5层以减少数值误拒，必要端点/视锥AABB剔除保持保守性，搜索限12候选/周期。V11(EGOPAPRB)保存候选与加速度状态。96×72试验后准备64×48折中；安全可回正时取消旧目标滞留。发现公共仿真每帧单次spin_once可能让新版诊断消息挤占Twist新鲜度回调，改为统一每帧最多8次非阻塞处理，三版最终都采用相同修复。
- 验证：iteration3–8均240秒超时；iteration9首次安全到达71.483秒、净距.2122m、停车.167秒、jerk RMS9.7245，仍慢于user_ego首测68.733秒。iteration10放宽候选采样且缩小候选余量后再次超时，已撤销该配置；iteration11保守96×72加矢量整形安全到达73.55秒、净距.1347m、jerk4.9702，路径137.459m仍偏长。完成轮的离线完整扫掠均未发现相交。联合细分改动后CTest16/16通过；最新混合候选/公共回调改动尚待完整回归，不能沿用旧通过结论。
- 影响与边界：机体.48m、完整包络.58m、未知拒绝、四相机、80ms计算截止、速度2m/s及公共动力学不放松。混合候选与回正属于工程扩展，不是论文原公式逐项不变。前11轮均为探索数据，含不同控制配置和早期公共回调缺陷，不作为最终均值。旧版源码未修改；所有失败原始数据保留。
- 回滚/备份：沿用19:29整包备份；benchmark目录保存协议、每轮实际参数/可执行SHA及部分中间二进制，未提交或推送。
- 后续事项：修复后重新比较，继续优化并冻结版本，执行统一五轮、全量回归、回放及安全审计。


### 2026-09-29 20:00 — [EGO1P1] 起终点对照首测失败，继续迭代（阶段记录）

- 状态：实施中，尚未达到优于两版的目标；不作为最终验收。
- 目的：固定同地图起终点，查明旧版与新版的实际性能差距。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：主运行Shell/Isaac运行器、新增`navigation_benchmark.py`、`navigation_benchmark.launch.py`、`tools/run_navigation_benchmark.py`、`tools/analyze_navigation_benchmark.py`、paper_guidance.hpp/cpp、paper_replay.cpp、controller_node、主launch和motion_consistency/intent_response回归；协议及原始证据在`performance/navigation_benchmark_20260929/`。
- 修改内容：三版单独构建，统一4倍Cloud、起点(-64,-11.2,2.8)/终点(64,11.2,2.8)、2m/s、.48m机体、.22s响应/1.2加减速和到达判据；建立JSON/CSV与完整扫掠离线统计。新增响应预览和命令角速度限制；补全8m到1cm之间被漏掉的前视尺度；端点必要检查减少无效认证；修复静止重复认证不一致，静止允许从选出的安全目标重新初始化。完整走廊与实际制动均可证明时才使用直达参考。正在修正短期状态域与长期目标距离绑在一起造成的过晚绕障。
- 验证：三版构建成功；首轮user_ego安全到达68.733仿真秒/65.586墙钟秒，路径131.933m，机体扫掠重叠0、最小净距.0752m；EGO1P0和初始EGO1P1均240秒超时，有输入停车228.23/218.67秒。iteration1/2也超时，全部失败保留。响应预览空旷90°测试由10.98秒到1.42秒，180°为.96秒。最初全CTest14/16：静止重复认证及回放版本断言两项失败，修复后相关三项回归通过；尚未做最终完整回归和5轮最终统计。统计工具首运行NumPy int64 JSON序列化失败，已转原生整数并重跑成功。
- 影响与边界：旧算法源码不修改；无人机仍执行一条三维规划矢量；未知拒绝、.58m认证包络和80ms截止时间保留。调参试跑期间发生过并行构建/测试，因此探索轮墙钟数据不能当作最终公平性能成绩；最终轮应在空闲计算资源下重启运行。V8增加响应预览/认证直达策略并兼容V3–V7读取，仅同构建承诺决策重放一致。新增策略是工程扩展，不能宣称论文公式逐项不变。
- 回滚/备份：`备份/EGO1P1_before_navigation_benchmark_20260929_192938.tar.gz`；三个项目初始源码SHA清单和优化前可执行文件已存benchmark目录；保留此前未提交改动，未推送。
- 后续事项：继续优化提前避障及正则域边界停车，完成真实重复对照后如实报告是否胜出。


### 2026-09-29 — [EGO1P1] 三代算法固定起终点公平对照与迭代优化（实施中）

- 状态：实施中，未得出性能优劣结论。
- 目的：按用户要求固定同一地图、起终点，多次测量user_ego、EGO1P0和EGO1P1的到达时间与稳定性；若新算法较差继续分析优化。
- 项目路径：`/home/starry/isaac-data/EGO1P1`；旧基线读取`/home/starry/isaac-data/EGO1P0`与`/home/starry/isaac-data/user_ego/Fov-gvf`。
- 涉及文件：EGO1P1运行器/主入口、独立benchmark launch与协议/统计工具、后续控制优化、WORK_LOG及统一HISTORY。
- 修改内容：拟统一4倍Cloud地图、起点(-64,-11.2,2.8)、终点(64,11.2,2.8)、2m/s上限、0.48m实际机体、tau=.22/加减速1.2执行模型、统一目标输入/偏航和到达判据；原算法源代码快照留存，基线不参与调优。
- 验证：已读项目约束、控制及运行脚本；尚未运行新对照。user_ego无Git元数据，不能以Git状态代替源码SHA清单；EGO1P0和EGO1P1已有修改均保留。
- 影响与边界：只用地图几何验证起终点/离线安全和原有固定起始自由球，不给在线导航提供全图路线；失败、碰撞保护、超时计入结果，不把超时当成功平均值。不预先承诺一定胜出。
- 回滚/备份：见`EGO1P1/performance/navigation_benchmark_20260929/backup.txt`及三份初始源码SHA清单。
- 后续事项：建立协议、构建三个算法、初测、诊断和迭代、重复验收与平均时间报告。


### 2026-09-29 19:24 — [EGO1P1] 完成方向一致性与速度整形；限定验收通过，停顿未完全解决

- 状态：本轮代码和限定验收完成；全程平滑、任意状态持续可控、大角度低延迟及严格50Hz仍未通过。本条补充18:52实施中记录。
- 目的：修复停车后背离摇杆超过90°持续行驶与速度反复突变，保留完整安全认证。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`src/pc_gvf/src/paper_guidance.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/src/paper_replay.cpp`、`src/pc_gvf/tools/paper_replay.cpp`、`src/pc_gvf/test/cpp/{motion_consistency_check,paper_guidance_check,intent_response_check}.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`tools/analyze_motion_consistency.py`、README、版本说明、PAPER_IMPLEMENTATION、MOTION_CONSISTENCY_ACCEPTANCE、WORK_LOG、`performance/motion_consistency_20260929/`及HISTORY三份状态/日志文档。
- 修改内容：取消90°轮转至后方的无进展搜索；最终虚拟图140°×140°，移动仍以实测速向居中，未采用实施中提到的25°偏置。主入口开启正向目标/最终命令约束，反向实测速先制动重设；摇杆目标及整条角段认证可行时INTENT_DIRECT收回旧绕障偏向，否则调和场/反馈。正则流盒失效先重设；锚点用解析J避免差分越界误拒绝。正常非零幅值增减限1.2m/s²，安全速度细化后完整认证；连续选择角步，失败/超时零输出同步回核心。V6保存新策略，离线audit支持120方向认证前缀检查。
- 验证：主入口最终三包构建成功；CTest16/16，平台15/15，Python AST和git diff --check通过；两轮84秒同输入Cloud持续逆向（排除改向制动窗口）均0，采样逆向命令0；意图有效位移旧33.68→41.36/45.11m；诊断大跳变71→30/20次，P95 0.378→0.120/0.100m/s；全机体扫掠/外部保护均0，最小间距0.1717/0.1339m；21帧V6重放一致。停车旧7.70→9.73/5.87秒，最长1.50/1.03秒，不能声称稳定减少。发布43.55/44.43Hz；空旷90°响应仍10.98秒，180°反向0.96秒。未做真实手柄/实物飞行验收。
- 失败记录：25°偏置导致空旷转向失败，删除；首次CTest两项NumPy/SciPy ABI收集失败，使用既有PYTHONNOUSERSITE=1解决；cloud4/6/7有输入停车38.75/16.22/51.02秒，均保留；停车帧审计揭示可认证前下方60°方向被旧虚拟垂直域遗漏，扩大候选域后继续验证；新增0.1mm几何余量用例受保守分辨率拒绝，改为3mm且仍禁止直前方后通过，未降低认证要求。详见验收报告与原始日志。
- 影响与边界：仍单一规划三维方向×标量速度；四相机、0.48m机体/0.58m总包络、未知拒绝、观测时效及80ms计算时限不变。半球约束和安全直达捕获是遥操作工程扩展，不是论文原式6/17的逐帧原样复现；可能拒绝必须先反向的路线。紧急零指令最大跳变仍约1m/s，不是严格jerk受限控制。EGO1P0未修改，未提交/推送。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_intent_motion_20260929_184710.tar.gz`；独立差异及最终SHA256清单在本轮performance目录。保留原有未提交改动，按本轮差异恢复后重新构建。
- 后续事项：改善正则域边界短停、计算时延与实测角步/执行响应耦合，并补真实手柄联调；当前不宣称任意环境无停顿。


### 2026-09-29 18:52 — [EGO1P1] 修复背离摇杆的局部搜索与速度突变（实施中）

- 状态：实施中，未完成验收。
- 目的：按用户要求定位停车后超过90度持续偏航及速度突变。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：paper_guidance头/实现、控制节点、重放V6、launch、CMake及motion_consistency_check测试；证据`performance/motion_consistency_20260929/`；WORK_LOG/HISTORY。
- 修改内容：去掉停车后逐次转90度的无进展搜索；虚拟图在保留实测速向的前提下朝摇杆偏转，主入口增加意图前半球目标/输出约束，反向实测速先制动再按新意图起步。对可行正则状态原地重建流盒以恢复反馈；正常速度增加受标量加速度约束，安全失败时细查可接受速度，最终矢量完整认证。实际发布零命令同步到下一周期，避免计算成功但被看门狗丢弃的指令成为旧方向/速度。新策略开关进入V6重放。
- 验证：已读用户18:35真实日志，约34.47Hz，最大callback325.75ms且频繁TRACKING/COMPUTE_DEADLINE/OUTSIDE_FLOW_BOX；上一轮Cloud轨迹相对输入超过90度运动约12.73秒、最长4.98秒、指令最大跳变1m/s。当前正在构建，未宣称测试或运行通过。
- 影响与边界：属于遥控意图约束的工程扩展，不是论文原式6的无限制所有方向选择；无显式回退指令时不自动反向逃逸。未知、传感器、0.58m包络和完整制动认证不放宽，仍单规划方向乘标量速度。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_intent_motion_20260929_184710.tar.gz`；用户原日志/快照已复制保存。
- 后续事项：完成针对性/全套回归及同输入Cloud对照，验证偏航、速度变化、停顿与安全指标。


### 2026-09-29 18:18 — [EGO1P1] 完成累计改向与水源初始化修复；新增方向响应验收

- 状态：本轮修复与限定验收完成；大角度低延迟、全场景摇杆方向跟随及严格50Hz仍未通过。
- 目的：解决深度刷新吞掉慢改向累计量，并修复本轮方向测试发现的源网格取整漂移/源边界反向初始化。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`；`src/pc_gvf/src/{paper_guidance,paper_replay,depth_angular_controller_node}.cpp`；`src/pc_gvf/tools/paper_replay.cpp`；CMake、launch；`src/pc_gvf/test/cpp/{intent_response_check,paper_guidance_check,paper_closed_loop_check}.cpp`；`src/pc_gvf/test/fixtures/paper/isaac_intent_response_trace.json`；采样脚本；`tools/{analyze_intent_response,verify_navigation_replays}.py`；README、版本、论文说明、当前/历史验收提示、WORK_LOG；`performance/intent_fix_20260929/`；HISTORY/README、FOV_GVF_FAMILY、CHANGELOG。
- 修改内容：独立参考意图，0.05rad累计变化立即重建/禁止旧参考；同一新鲜深度的小目标更新允许按100ms周期进行。世界源方向连续保存，源势边界先短步初始化后规则反馈。新增端到端方向字段，V5状态及事件重放；旧V3/V4仅迁移读取，不伪称恢复原文件未保存的参考基准。阈值范围校验，默认速度/转向/安全参数不变。
- 验证：最终构建成功，完整CTest15/15、平台15/15；水平/竖直1度/步至20度各6次重设（旧0次），微抖/幅值/松手/同深度更新、第一步不背离目标、水源不漂移、累计量重放通过。空旷动态90/180度分别12.22/28.40秒进入10度内，末尾趋近0；障碍/扰动/可行场变化闭环回正≤10度全部通过。原Cloud84.017秒，慢扫误差中位1.310度，路径56.314m，有输入停车7.700秒/最长1.067秒；0.48m完整扫掠重叠0、外部保护0、最小间隙0.1144m。9个V5真实快照及改向事件一致，V3/V4读取成功。Python语法/主脚本语法/git diff检查通过。
- 失败与限制：首次缺Eigen Geometry导致链接失败；抖动夹具Eigen临时表达式导致误零；早期90度反向/初始化停滞，继续修复。最初8秒大转向指标低于现有角步长+tau响应能力，保留失败并改为明确25/45秒有界检查，同时报告实际迟缓；52秒夹具飞出30m自由球后安全停车，扩大测试自由球至100m，仅测试生效。build7旧源区延续假设/过宽初始化导致2项回归失败，build8收紧源势边界并先进入正则场再检验延续，原30次反馈门槛保留。平台首轮缺安装环境收集失败，加载后通过。原Cloud+Y18秒阶段没有达到10度跟随，不能记为通过；目标受限且实际跟踪仍迟缓，回正阶段遇新障碍也再次偏转。约39.44Hz、仍有超时和短停；未验收实体手柄、实飞、全域证明。
- 影响与边界：仅EGO1P1/HISTORY；不改变0.48/0.58m机体/包络、四相机、场景、未知拒绝或完整制动；输出仍为单三维规划矢量，无新增逃逸/分轴控制。build10只加强测试，Cloud9对应同一运行算法。本次仿真退出0，无本任务运行进程残留，未提交推送。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_intent_20260929_175528.tar.gz`；源码清单、最终重放程序与各阶段日志位于`performance/intent_fix_20260929/`。恢复后通过原主构建脚本重建。
- 后续事项：实体手柄现场验证；如继续要求快速大角度转弯，需单独评估角步长与真实速度响应模型的匹配，不能只提高水流刷新频率；继续优化源/流盒切换和最坏计算周期。


### 2026-09-29 18:00 — [EGO1P1] 累计改向参考重设修复（实施中）

- 状态：实施中，未完成验收。
- 目的：按用户要求完善摇杆改变方向时的水流更新和参考线重设。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`src/pc_gvf/src/{paper_guidance,paper_replay,depth_angular_controller_node}.cpp`、launch、CMake、`src/pc_gvf/test/cpp/intent_response_check.cpp`、改向输入夹具、`scripts/isaac/run_fov_gvf_navigation.py`；证据`performance/intent_fix_20260929/`。
- 修改内容：参考线意图独立于上次构场意图，0.05rad阈值相对当前参考累计；有效改向立即重建且禁止旧参考延续，同一新鲜深度可响应周期性小改向。重放V5保存新增状态，旧V3/V4以原上次构场意图作初始基准。新增意图/目标/参考/命令与重设诊断字段。
- 验证：首次主入口build1失败（新增cross计算缺少Eigen/Geometry定义导致链接错误），已补头文件并运行build2；核心、闭环和Isaac验收尚未完成。
- 影响与边界：仅EGO1P1与HISTORY；安全包络、传感器、未知拒绝及单矢量输出保持原配置。未提交推送。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_intent_20260929_175528.tar.gz`，含本次之前未提交源码/脚本/说明。
- 后续事项：完成回归、慢改向/大改向/回正验收并记录所有失败及边界。


### 2026-09-29 17:44 — [EGO1P1] 核查摇杆改向：确认慢改向可漏掉参考线重设

- 状态：仅检查；诊断完成，控制源码与运行配置未修改。
- 目的：核对用户所问“摇杆改向后水流目标更新并从当前状态重设参考线”是否充分实现。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`performance/intent_audit_20260929/{README.md,probe.cpp,probe.csv,compile.log}`、`WORK_LOG.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：仅保存诊断与结论。确认明显改向会立即重建并禁止旧参考延续；但0.05rad阈值相对每次构场更新的previous_intent_比较，慢改向可能一直被判为不变。实时目标仍按最新意图计算，不能将漏重设等同于目标完全不更新。
- 验证：独立g++链接现有核心，编译与诊断退出0；初始化后1度/步至20度的20次更新均continued=1且未触发COMMAND_CHANGE，之后20→35度触发重设。空旷人工夹具前20步目标与输出正确跟随，故未证明实际偏航由此单独造成。已核对论文III.E式23–24和现有测试：明显改向事件有覆盖，慢改向与方向收敛时间/误差验收缺失。未运行Isaac、ROS、实体手柄、完整构建或全套回归。
- 失败与限制：默认sandbox命令失败，升级执行后可读；旧/tmp论文文本与主构建目录不存在，不能复用旧程序，改为重新提取与临时独立诊断。夹具固定位置、人工自由种子且非动力学闭环，仅支持事件逻辑结论，不代表真实飞行。飞行时局部角域跟随实测速向，侧向/反向意图可能先投影到域内目标，属于另一个响应限制。
- 影响与边界：仅新增检查证据和双重记录；未变更控制、安全、场景、入口、EGO1P0或其它项目，未提交推送。前次恢复用例通过不等于摇杆方向跟随已完整验收。
- 回滚/备份：无需源码回滚；本次新增记录可按目录定位，原有记录完整保留。
- 后续事项：分离上次构场意图与参考线对应意图，保留慢改向累计量；补充慢扫、90/180度改向、绕障后回正的方向误差与响应时间验收。


### 2026-09-28 22:28 — [EGO1P1] 完成连续通道、近场观测保留与停车重放；限定恢复用例通过

- 状态：本轮方案实施与验收完成；定义的改向/长待机恢复用例通过，严格50Hz及任意状态持续可控未通过。补充前述实施中条目，失败证据保留。
- 目的：按用户批准的论文接轨方案解决已知空间在记录/查询中被裁掉、短前视被固定距离门槛否决及停车不可重放的问题。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`；`src/pc_gvf/src/{paper_guidance,paper_replay,depth_angular_controller_node}.cpp`；`src/pc_gvf/tools/paper_replay.cpp`；`src/pc_gvf/test/cpp/continuous_recovery_check.cpp`；CMake、launch；`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/isaac/run_fov_gvf_navigation.py`；`tools/verify_navigation_replays.py`；`{README,EGO1P1_VERSION,PAPER_IMPLEMENTATION,ENVELOPE_ACCEPTANCE,CONTINUOUS_ACCEPTANCE,WORK_LOG}.md`；`performance/continuous_20260928/`；HISTORY/{README,FOV_GVF_FAMILY,PROJECT_INDEX,CHANGELOG}.md。
- 修改内容：最多256个完整认证实测胶囊段、128个实际包络进入后保留原中心/半径的运动证明球；凸体/收缩证据中心线区间并集及有界细分证明连续扫掠，曲段合并扣除偏离误差。原始障碍命中空间索引逐点撤销，时间回退清空。32个深度关键帧改为按近场覆盖贡献淘汰；在已启用的静态保留且历史不确定性增长0的配置下，来源深度与到达几何统一静态保留，不再单独60秒过期，非静态仍按配置过期，0.3秒新鲜深度门禁保留。前视0.10m改作期望下限，实际下限由实测速率制动距离及5mm数值保留量确定，式22也使用同一保留量；0.58m物理/测量/执行包络不变。逐像素只做最多五级阈值认证，最终输出方向再精查；构场失败且有认证方向时虚拟图加密，真实深度不补全。新增细分构场原因、CSV字段、每运行最近16帧快照及确定性单步重放。输出仍为同一三维规划矢量，没有附加分轴控制。
- 验证：最终构建成功；CTest14/14、平台pytest15/15、真实ROS拒绝契约通过；36个Python AST、两个主Shell语法、git diff --check通过。最终94秒相同输入两轮停车6.93/19.30秒、最长1.53/3.48秒、累计53.00/34.54m，各5段输入均有位移（前一轮同类默认25.18秒/最长12秒）；67秒松手后10秒输入净位移7.946m（前一轮0）；2m/s输入沿X前进10.619m达到原≥8m标准，最高1.99977m/s；关闭起点认证保持零位移/零速度。最终5组0.48m独立扫掠球重叠0、外部保护0，运动组最小间隙0.109m。最终33个真实状态快照逐值重放一致，记录与可执行程序/源码哈希保留。
- 失败与限制：build1声明不匹配构建失败后修正；build5过深细分导致超时，停车66.07秒；build6紧通道接缝回归失败，build7解析区间并集修正；build8一次停车11.53秒，但build9复测46.42秒/最长12秒，其停车位置到达球仅比认证包络多0.295mm，故继续处理来源观测丢失。build11额外3cm终点余量停车34.72秒/最长11.95秒，默认关闭该实验约束。build13夹具绕过step历史接入导致失败，build14修正并通过；算法与build12一致。perf采样被系统权限限制，未更改系统权限；重放最初路径写错一次，修正隔离安装路径后最终验证成功。最终活跃运行约35.76–48.69Hz，仍有超过80ms的周期与短暂停车；不声明严格50Hz、全场景无停车、全域ISS或实物实验通过。原16:15手柄运行没有原始快照，本轮不能宣称逐帧重放其截图位置。
- 影响与边界：仅EGO1P1及HISTORY/本任务备份；场景、四相机、真实机体半径0.48m、统一包络0.58m、完整制动检查、未知拒绝不变。静态历史保留是工程前提，不推广为任意动态环境保证；保留球形机体，未实现可选的姿态相关非球形包络。未修改EGO1P0/其他工程，未提交或推送Git；未进行实体手柄/真实飞行验收。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_continuous_20260928.tar.gz`；阶段日志、代表性源码/重放程序及最终清单在`performance/continuous_20260928/`。回滚需恢复对应源码后使用原构建主入口。
- 后续事项：真实手柄持续运行复核、扩大不同轨迹/场景样本，继续优化最坏计算周期；不能把有限用例改善当作任意位置必然可通行。


### 2026-09-28 21:34 — [EGO1P1] 连续通道认证、低速恢复与失败帧重放（实施中）

- 状态：实施中，当前仅构建和部分回归已完成，尚未完成仿真验收。
- 目的：按用户批准方案减少已知空间被离散记录/保守查询丢失造成的停车，保持论文单三维规划矢量。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`src/pc_gvf/src/{paper_guidance,paper_replay,depth_angular_controller_node}.cpp`、`src/pc_gvf/tools/paper_replay.cpp`、`src/pc_gvf/test/cpp/continuous_recovery_check.cpp`、CMake、launch、主运行脚本、`tools/verify_navigation_replays.py`；证据在`performance/continuous_20260928/`。
- 修改内容：保存完整认证的实测连续胶囊通道并保留障碍撤销；按空间/视角冗余淘汰深度帧；凸证据精确包含和有界细分；前视下限与速度残余距离统一使用显式5mm认证分辨率（包络误差仍计入0.58m）；几何可行但角域构建失败时2倍方向图细化；细分构建失败原因，默认保留最近16个停车状态快照，支持同版本离线确定性重放。
- 验证：build1接口声明不一致失败，修正后build2/3/4构建成功；build3有缩进警告待清理；新增continuous_recovery_check通过，完整CTest进行中。未完成仿真/碰撞/性能验收，不宣称停车问题已解决。
- 影响与边界：仅EGO1P1及HISTORY。机体、场景、传感器、完整运动制动检查保留；不加入未知自由空间，不新增分轴控制；当前连续证据依赖原静态场景前提。未提交推送Git。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_continuous_20260928.tar.gz`，含本任务开始前已有未提交工作。
- 后续事项：完成核心/ROS/原场景重放、长待机与未知拒绝验收，根据真实失败修正后更新最终结论。


### 2026-09-28 13:32 — [EGO1P1] 统一安全包络、校核真实机体并完成多轮余量对照；持续可控未通过

- 状态：本轮优化代码与验收完成；持续可控目标部分完成，长时改向/待机后恢复未通过。本条更新12:34实施中记录，保留中间失败。
- 目的：按用户批注统一运动/记录范围，按实际机体和误差核对附加余量，并对照停车与碰撞。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`src/pc_gvf/src/{paper_guidance,depth_angular_controller_node}.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf/test/cpp/observed_space_check.cpp`、`src/pc_gvf/test/fixtures/paper/{isaac_envelope_recovery_trace,isaac_evidence_expiry_trace}.json`、`scripts/isaac/run_fov_gvf_navigation.py`、`tools/{calibrate_vehicle_envelope,calibrate_motion_discretization,analyze_envelope_trace}.py`、`{README,EGO1P1_VERSION,PAPER_IMPLEMENTATION,PAPER_ACCEPTANCE,ENVELOPE_ACCEPTANCE,WORK_LOG}.md`、`performance/envelope_20260928/`；同步HISTORY/{README,FOV_GVF_FAMILY,PROJECT_INDEX,CHANGELOG}.md。运行入口生成的普通性能记录保留。
- 修改内容：USD含桨外接半径0.4763707m，主入口/外部保护body_radius从0.25校正0.48m；安全/执行附加余量0.20/0.10→0.02/0.02m，深度0.03m及固定最大帧龄预算0.03m，共0.58m。15工况60Hz与1ms参考积分最大差0.0156668m，这是理想仿真有限样本而非实物误差界。统一当前位置、方向前缀、完整运动与到达记录的最低半径；不再额外要求10cm才能记录，只保留可独立证明的部分余量。记录松手制动后的实测点，最新点与15cm空间样本分开；完整制动预测证明只能在真实包络到达后接续，不能直接把未到达路径当自由。增加相邻已认证球根平面/球缺解析包含检查，修复粗分块的薄重叠误拒绝；未知月牙、端部与障碍撤销仍拒绝。最低前视1→0.10m，仍受实测速率制动需求约束；最后一次有界尝试显式落到安全下限，修复仅五次减半未实际到达下限的问题。原始命中用AABB筛选再精确检查；增加证据计数和可选CSV，诊断订阅仅在验收启用。
- 验证：主脚本build10三包成功；最终CTest13/13，平台pytest15/15，真实ROS节点未知/过期/松手拒绝探针通过。最终相同代码/输入/0.48m真实机体的94秒对照：宽总半径0.84m与默认0.58m，有输入停车59.98/25.18s（本次减少约58%）、最长13.17/12.00s、路程9.50/40.03m，独立0.48m全机体扫掠球最小间隙0.407/0.153m、重叠0、外部保护0。长待机组第二段10秒输入位移0，恢复未通过，且待机前已有NO_REGULAR_FIELD，不能单独归因于过期。2m/s/24秒前进12.641m，达到原≥8m标准；最大实测速率1.99976m/s、扫掠球间隙0.156m、保护0。禁用起点认证6秒位置/速度不变并拒绝未知。最终5组重叠及保护均0；默认改向/宽余量/待机组无UNOBSERVED_BODY_ENVELOPE或COMPUTE_DEADLINE转入。默认命令49.19Hz、最大间隔113.097ms，严格50Hz未通过。构建/测试/原始CSV/摘要/复现命令及源码哈希保留。
- 失败与比较边界：用户11:31原始日志为持续UNOBSERVED_BODY_ENVELOPE，11:28为NO_REGULAR_FIELD。旧0.25m机体基线同输入停车仅3.33s，未复现用户原始锁死，不能声称最终版全面优于旧版。build1仅统一范围、build3只保留0.4s证明、build4完整制动但1m前视、build5–7较小余量/部分证明均有长时停车；build8候选：窄/宽停车21.10/63.40s，2m/s前进7.660m低于8m未通过；build9窄/宽48.02/44.08s，并非余量越小每次都更好。完整原始结果保留。几何工具第一次工作目录/路径错误后移回项目tools，无第三方源码残留。Isaac退出仍有已有rosout context invalid告警，最终进程正常退出。
- 影响与边界：只修改EGO1P1及HISTORY；场景、相机、单三维规划速度链不变，新增未知及完整制动检查保留。当前余量适用于有明确前提的理想静态仿真，不是实物标定。长时连续可控、任意停止点恢复、严格50Hz及完整论文理论/实物验收仍未通过；未进行实体手柄交互。未修改EGO1P0/其他工程，未提交或推送Git。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_envelope_optimization_20260928_123444`；`performance/envelope_20260928/source_stage9/`另保留解析两球查询前的源码。中间日志未删除，旧论文验收加日期说明。
- 后续事项：围绕失败帧的连续认证空间与狭窄可行角域做可重放诊断，继续处理NO_REGULAR_FIELD恢复；不能继续盲目缩小真实机体或取消未知拒绝。


### 2026-09-28 12:34 — [EGO1P1] 开始统一安全包络与到达证据，并按实际几何标定

- 状态：实施中。
- 目的：按用户批注要求统一运动/记录认证范围、核对真实机体、进行余量对照，解决持续拒绝运动。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：后续代码与验收另记；本条WORK_LOG.md及HISTORY/CHANGELOG.md。
- 修改内容：保存源码/配置/文档快照及用户两次卡住日志。只读USD发现螺旋桨几何最外缘约0.477m，旧0.25m碰撞半径不足；不能直接缩小机体。
- 验证：历史11:31运行持续UNOBSERVED_BODY_ENVELOPE，11:28运行NO_REGULAR_FIELD；当前未执行优化后测试。
- 影响与边界：仅EGO1P1与HISTORY；新增未知仍拒绝，沿用单三维矢量。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_envelope_optimization_20260928_123444`。
- 后续事项：几何/误差标定、认证衔接、原参数和新参数对照、回归及最终记录。

### 2026-09-24 18:04 — [EGO1P1] 固定起点认证、论文自由域/反馈完善及实际绕障验收

- 状态：本轮实现与有界静态场景验收完成；完整论文复现仍部分完成（实际43–47Hz、理论全域条件/实物实验未验收）。本条更新15:55“实施中”状态，保留15:05严格停车失败记录。
- 目的：用户批准启动时验证固定起始自由邻域，随后依靠真实深度和已认证运动维护自由空间；按论文式1–31继续补齐工程实现并恢复正常运动，新增未知空间仍不可进入。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：`src/pc_gvf/include/pc_gvf/{paper_guidance,verified_start}.hpp`、`src/pc_gvf/src/{paper_guidance,verified_start,depth_angular_controller_node}.cpp`、`src/pc_gvf/{CMakeLists.txt,package.xml}`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf/test/cpp/{paper_guidance_check,observed_space_check,paper_closed_loop_check}.cpp`、`src/pc_gvf/test/test_manual_control_math.py`、`src/pc_gvf/test/fixtures/paper/{isaac_verified_motion_trace,isaac_max_speed_trace}.json`、`scripts/isaac/{manual_control_math,run_fov_gvf_navigation}.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`及平台测试、`tools/{ros_vector_guidance_probe,validate_isaac_depth_contract,analyze_paper_isaac_run}.py`、`{README,EGO1P1_VERSION,PAPER_IMPLEMENTATION,PAPER_ACCEPTANCE,WORK_LOG}.md`、`performance/paper_completion_20260924/`；同步HISTORY/{README,FOV_GVF_FAMILY,PROJECT_INDEX,CHANGELOG}.md。
- 修改内容：控制器直接核验已知USD/occupancy的SHA-256并认证半径2.5m的单个固定世界坐标球；首个零位姿等待实际位姿，禁止运行中凭几何扩张。球/未知像素截断锥距离、胶囊覆盖、多视图八叉包含、连续实际方向认证和网格穿越检查取代过保守/不完整几何；正证书仅当帧复用。静态Cloud保留最长60s/最多32个深度关键帧及最多32个实测位置已认证到达球，真实原始像素障碍冲突撤销证据；不同相机乱序不误清历史，同路时间回退清空。通用模式非有限值仍未知，主入口明确采用已实测的Isaac正无穷无返回契约，只认证10m；修正半像素中心，NaN/0/-Inf未知。虚拟方向图不附带虚构深度；规则场采用24×18方向图/12×9粗场、最小1m可行前视、角速度16像素/s、最小100ms场更新、原分辨率掩码回退。参考按有序折线最近线段延续；增加有效跳变、2R界、J范数和冻结图内执行偏差诊断。所有输入模式统一tau=0.22s、加速度1.2m/s²执行模型，零指令恒减速到零，预测200ms延迟+200ms响应+完整制动。回调超过80ms拒绝非零结果，桥/执行端100ms超时发零。传感器独立回调组、最新帧队列；OpenMP4线程并行独立证据/方向检查、线程私有当帧缓存、等价平方距离球冲突检查提高性能；串行回退保留，导出OpenMP依赖。
- 验证：主脚本最终build33三包成功；`PYTHONNOUSERSITE=1 ctest --test-dir /tmp/fov_gvf_ego1p1_isaac_build/pc_gvf --output-on-failure` 13/13，平台pytest15/15；ROS真实节点探针通过无种子拒绝、过期深度/意图及松手停车。四路原始320×240捕获的19,163个+Inf像素，独立精确DDA在10m内真实几何命中0，工具和NPZ保留。九场景各500步独立闭环零碰撞：立柱终点x=9.27867、扰动x=9.24794、可行环境变化x=9.27453，分别75/297/72个TRACKING步；障碍侵入余量后停车是预期拒绝，不宣称动态到达。无反馈/无延续/单场消融执行，重置数167对7，原始chi不同坐标不能直接排序性能。最终实际Isaac四路90°相机：1m/s输入29s前进17.519m、2m/s输入24s前进19.608m，外部保护介入均0、COMPUTE_DEADLINE状态转入均0；实际指令分别46.97/43.37Hz（ROS时间），回调最大47.035/65.820ms，未满足所有20ms周期。三维输入/改向及t=3s横向速度扰动12s运行通过，t=10s松手后零命令，保护0；禁用起点认证6s保持(-64,-11.2,2.8)并UNOBSERVED_BODY_ENVELOPE。另保留同方法较慢实现两次16.419/16.676m重复和并行版16.551m结果。两主Shell语法、13个Python AST和git diff --check通过；闭环图已渲染并目视核对。
- 失败记录：初期零位姿认证拒绝、1.5–2.5m停滞、多球/原生高分辨率/抗锯齿试验不奏效，最终恢复单球、原相机和TAA；早期未加期限/匹配制动版本求解约3.109s、外部保护142次，明确失败。build18编译作用域错误，其后误跑旧二进制的方向图日志不作新代码证据，build19修复。CTest用户NumPy ABI与平台未source导入失败均保留，隔离/source后通过，未修改依赖版本。深度DDA早期错误像素/网格约定造成95/22个伪冲突，经实际约定修正为0，未筛除像素。自动审批拒绝可伪造ROS自由证书方案后改为本地文件SHA+几何验证，未创建证书消息。退出仍有rosout context invalid告警但节点正常退出。所有关键中间原始日志和说明保留于experiments/，旧验收/实现说明单独保存。
- 影响与边界：人工输入仍只是三维期望，执行一条规划矢量；没有XYZ分量合成、人工Z叠加或额外路径控制器。未增加相机或修改场景；历史/已到达证据仅针对静态Cloud前提，+Inf契约不能推广到未知真实设备。外部保护0只支持本轮轨迹未被阻断，不证明全局安全。表I米制参数与正文像素方向图转换缺失，当前参数明确不同；连续Laplace/flow-box/ISS全域条件、严格50Hz及真实平台/统计基线实验仍未证明。只改EGO1P1和HISTORY；EGO1P0/其他工程/远程分支未改，未提交或推送；既有未提交三维控制改动保留。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_geometry_refinement_20260924_155516`。最终源码及验收输入SHA-256、构建/测试/实际运行日志存于`performance/paper_completion_20260924/`；不要把中间失败二进制/日志与最终版混用。
- 后续事项：将高开销自由域刷新与快速测量反馈做严格时限设计并验证50Hz；厘清论文表I与方向图单位；验证更广共同域/扰动界和多种子基线；具备实际平台后开展实物验收。当前通过项与未满足项详见PAPER_ACCEPTANCE.md。

### 2026-09-24 15:55 — [EGO1P1] 继续论文实现：精确自由域、规则场及运动验收

- 状态：实施中，最终验收尚未执行。
- 目的：用户要求继续按顺序补齐软件，采用更贴近论文的方法恢复可用运动；保留此前失败记录。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：本阶段WORK_LOG.md、HISTORY/CHANGELOG.md；后续实际修改和验收另记。
- 修改内容：备份当前源码/脚本/说明；复核论文式3–5确实要求可认证起始段，论文未说明机载近场盲区初始化。先改进独立于初始化约定的几何和场计算；向用户询问是否允许场景几何验证固定起始自由邻域。
- 验证：本阶段仅核对论文和建立快照；未宣称新算法或运行通过。
- 影响与边界：只在EGO1P1继续开发；EGO1P0、其他项目和远程分支不变。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_geometry_refinement_20260924_155516`，包含src/scripts/tools和相关文档；场景及.git未修改，无需重复复制。
- 后续事项：自由体积查询、粗细场、参考延续、误差诊断及动态/Isaac验收。

### 2026-09-24 15:05 — [EGO1P1] 论文核心实现及严格观测验收；完整运行未通过

- 状态：部分完成；本轮实现与验收已执行，完整论文复现/导航验收未通过。
- 目的：按用户计划实现论文方法；用户明确选择未知空间不可通行、观测不足停车，允许完整运行验收暂不通过。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：新增 `src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`src/pc_gvf/src/paper_guidance.cpp`、`src/pc_gvf/test/cpp/paper_guidance_check.cpp`、`paper_closed_loop_check.cpp`、`src/pc_gvf/test/fixtures/paper/{isaac_intent_trace,isaac_strict_observation_trace}.json`、`PAPER_IMPLEMENTATION.md`、`PAPER_ACCEPTANCE.md`、`performance/paper_acceptance_20260924/`；修改 `src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scripts/isaac/manual_control_math.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf/test/test_manual_control_math.py`、`tools/ros_vector_guidance_probe.py`、`README.md`、`EGO1P1_VERSION.md`、`WORK_LOG.md`；同步 HISTORY/README.md、FOV_GVF_FAMILY.md、CHANGELOG.md。
- 修改内容：主launch启用paper_enabled；保守深度缩减、40ms近邻采集位姿匹配、相机0.22m/0.02m外参；严格机体包络及方向扫掠自由距离下界，未知直接拒绝；24×18粗场/最多32×24细场与边界耦合；C2势场重建及C1规则向量场；等势截面返回、有限差分J及横向反馈；50ms场区间冻结和世界参考方向转图延续；单一三维指令及含制动尾段的扫掠检查。仅trace验证模式加入tau=0.22s、加速度1.2m/s²的一阶运动模型和文件扰动；正常人工输入不增加独立轴向控制。旧算法保留作对照。
- 验证：主脚本构建三包成功；最终PYTHONNOUSERSITE=1下CTest 11/12（退出8，paper_closed_loop_check未通过），平台pytest14/14。数值直场chi 3→1.0981、曲场3.08273→1.12845；J u、跨片导数、场冻结/换图/耦合及未知/制动拒绝断言通过。闭环八场景零碰撞，但立柱停在x≈2.0402m，未达到既定x>4.5m；未降低或跳过验收标准。障碍场景没有有效TRACKING样本，不能报告动态消融优势。真实ROS探针--paper退出0，全部机载输入命令为零并正确报告状态。最终Isaac严格trace运行6秒、363帧、301命令、200有输入调用，位置保持(-64,-11.2,2.8)，ESDF阻断0，主入口退出0；这是停车契约通过，不是运动通过。Shell语法/Python AST/git diff --check通过。
- 失败记录：初期Eigen链接、低速换图、源区/目标区处理和前视距离问题作出修正；严格规则下闭环仍失败。初轮CTest另有2项NumPy用户依赖冲突，隔离用户依赖后消除，未修改依赖版本。长扫掠包围盒过保守，改连续短段；最终方向域与rollout余量统一。Isaac仍有内部Humble回退和退出rosout context invalid告警，进程正常结束。中间失败与严格规则前运动结果在验收目录分别标记保留。
- 影响与边界：四水平机载相机无法完整覆盖自身包络，默认运行会返回UNOBSERVED_BODY_ENVELOPE并停车；未增加相机、先验自由球或多视图/历史观测融合。闭环外部相机数学夹具在严格域收窄时返回NO_REGULAR_FIELD；不能据零碰撞声称到达。连续模型稳定性全域条件、真实平台实验和完整论文复现未验证。源码只改EGO1P1，保留此前未提交三维控制改动；EGO1P0及其他工程不变，未提交/推送GitHub。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_paper_20260923_205045/EGO1P1`；恢复时先保存本轮工作，不覆盖先前未提交修改。自动构建目录继续使用/tmp，验证原始日志已归档项目performance。
- 后续事项：取得机体及运动空间的真实观测覆盖；在严格域内建立可用规则场并通过障碍/扰动闭环，取得有效反馈/延续/粗细场消融样本，再做完整论文运行验收；详见PAPER_ACCEPTANCE.md。

### 2026-09-23 20:50 — [EGO1P1] 开始分阶段实现论文方法

- 状态：部分完成，实施与验收进行中。
- 目的：按用户授权实现论文保守观测、锚定flow-box反馈、跨帧延续、粗细场及运动验收。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：后续阶段记录逐项列出；本条涉及WORK_LOG.md及HISTORY/CHANGELOG.md。
- 修改内容：保存当前含未提交三维控制改动的完整快照，再逐阶段实现；保留旧算法作为对照，不覆盖既有改动。
- 验证：本阶段仅完成快照复制；方法验收尚未执行。
- 影响与边界：只修改EGO1P1及统一记录，不修改EGO1P0或远程分支。论文不完整的数值实现细节将明确说明，不将数值测试等同实物实验或理论证明。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_paper_20260923_205045/EGO1P1`。
- 后续事项：实施五个阶段并逐项记录实际结果。

### 2026-09-23 20:17 — [EGO1P1] 统一为深度规划三维建议矢量

- 状态：完成。
- 目的：用户确认人工输入只给三维期望方向，最终执行深度规划矢量，取消分别生成方向速度后再合成，不引入额外控制器。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：scripts/isaac/manual_control_math.py、beitong_joystick.py、run_fov_gvf_navigation.py；src/pc_gvf/src/depth_angular_controller_node.cpp、depth_angular_core.cpp、include/pc_gvf/depth_angular_core.hpp、launch/isaac_cloud_navigation.launch.py；src/pc_gvf/test/cpp/horizontal_360_check.cpp、test_manual_control_math.py、test_beitong_joystick.py；src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py、test/test_platform_math.py；新增 tools/ros_vector_guidance_probe.py；README.md、EGO1P1_VERSION.md、WORK_LOG.md、HISTORY/FOV_GVF_FAMILY.md、HISTORY/CHANGELOG.md。
- 修改内容：输入沿原始三维方向统一归一化和限幅；四相机仍按水平朝向选视图，但完整三维输入、当前速度及目标参与规划，horizontal_only=false；取消清零Z、独立升降回退及 composeHorizontalVerticalCommand。直接输出规划矢量，整体缩放满足总速度2m/s、Z分量1m/s；取消控制器 limited() 后置速度混合、加速度/jerk滤波及4个失效launch参数。保留规划器内部方向变化约束、制动和rollout检查。holonomic命令桥旋转坐标后整体缩放，不逐轴截断；仿真碰撞保护整体停止，不提取安全轴分量继续滑动。
- 传感器与行为边界：仍为四路水平90°相机，不新增传感器。纯上/下和所选视图外方向输出 DIRECTION_OUTSIDE_DEPTH_FOV；深度缺失/过期、输入过期、非法指令整体停止。视野内允许斜向升降，规划也可给出含Z的绕障方向。没有球面感知或完整三维全局规划。转向/yaw输入、场景与地图可视化保留。
- 验证：主构建脚本三个ROS包成功（最终构建14.3s）；CTest 10/10、平台pytest 14/14通过；Python AST和git diff --check通过。新增合成ROS探针在独立域83启动真实C++控制器，检查前上/左上/后下/右下/三维斜向、前方障碍、纯上下视野外、深度过期、输入过期和松杆；186组同帧发布速度与command_ray方向余弦>1-1e-10，所有限速/状态断言通过。合成障碍输出约(0.8931,0.4344,0.1172)，未叠加人工Z。日志 /tmp/ego1p1-vector-probe.log、/tmp/ego1p1-vector-controller-g3ti4ydb.log。
- 启动验证：经两个主入口完成构建和ISAAC_MANUAL_INPUT_MODE=keyboard、ISAAC_MANUAL_TIMEOUT=3、默认Cloud+RViz启动；183仿真帧、151命令、guidance=0、碰撞阻断0、MANUAL_TIMEOUT后退出码0。日志 /tmp/ego1p1-vector-smoke.log，性能记录 /tmp/ego1p1-vector-performance.md。系统rclpy不可用后内部Humble成功加载；退出仍出现一次rosout context invalid告警，各节点正常退出。本次未执行Isaac运动绕障或实体手柄操作；合成探针不是实际飞行验收。
- 影响与边界：当前EGO1P1控制语义变化，旧XY/Z与平滑性能结论不自动继承。EGO1P0及其他工程源码未修改；未提交或推送本次修改到GitHub。
- 回滚/备份：修改前精简基线为50f7c5500af86e31d0d9af03de8ec9a29ffb10e1（已发布远程ego1p0）；可在保存后续工作后按需恢复对应文件。
- 后续事项：动态Isaac绕障和实体手柄三维方向体验尚待验证；若需纯上下规划，须另行扩展深度覆盖范围。

### 2026-09-23 20:00 — [EGO1P1] 发布当前版本到 GitHub ego1p0 支线

- 状态：本地版本整理完成，提交并推送待执行；远程结果以 HISTORY 后续核验记录为准。
- 目的：按用户要求以当前 EGO1P1 替换仓库 ego1p0 支线内容，不修改其他支线。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：此前复制和精简形成的全部项目改动，以及 WORK_LOG.md、HISTORY/CHANGELOG.md。
- 修改内容：本地 ego1p1 提交当前完整项目状态，显式推送 HEAD:refs/heads/ego1p0 到 Chenwill1899/Fov-gvf；保留提交历史，不强制推送、不创建其他远程分支。
- 验证：推送前远程 ego1p0 为 6d4e5778ccc3d97b585004c1d4f86e1f4a325ffc，dev 为 804f26e2e631d0571f5e1cc5088100481df1b16d；git diff --check 通过。本次不修改算法、不重跑构建或仿真；已有验证见上一条整理记录。
- 影响与边界：仅发布 EGO1P1 项目内容到指定远程支线；本地 EGO1P0 与其他远程分支不改动。
- 回滚/备份：旧版本提交 6d4e5778ccc3d97b585004c1d4f86e1f4a325ffc 仍保留在历史；整理前完整快照见上一条记录。
- 后续事项：推送后核对目标分支 SHA 和其他远程引用。

### 2026-09-23 19:51 — [EGO1P1] 精简为两个主入口及运行/回归依赖

- 状态：完成。
- 目的：按用户要求整理 EGO1P1，主要保留 scripts/build_isaac_ros_workspace.sh 与 scripts/run_isaac_fov_gvf_navigation.sh 及相关依赖。
- 项目路径：`/home/starry/isaac-data/EGO1P1`。
- 涉及文件：完整整理前快照及清单在 `/home/starry/isaac-data/备份/EGO1P1_before_cleanup_20260923_194915`。55 条移出路径见 `REMOVED_PATHS.json`，包括旧 build/install/log、KEYBOARD/MISSANDKEYBOARD 独立入口、备用场景、场景生成/查看脚本、build_ros2.sh、未接入的 src/pc_gvf_core、隐藏研究框架和旧说明。迁移手柄模块到 scripts/isaac/beitong_joystick.py，测试到 src/pc_gvf/test/test_beitong_joystick.py；修改运行器导入、pc_gvf/CMakeLists.txt 测试注册、README、EGO1P1_VERSION、AGENTS、WORK_LOG、performance 自动记录及 HISTORY 四份状态/记录文档。
- 修改内容：scripts 顶层只保留构建/运行两个 Shell 入口，isaac 子目录保留运行器/手柄/控制数学三个 Python 文件，lib 保留设备识别。三个在用 ROS 包、Cloud/平地导航资产、RViz、算法测试/Python数值基准、ROS探针、性能记录和 Git 历史继续保留。算法、launch 参数和场景关键文件 SHA-256 与整理前一致；手柄模块仅迁移位置，内容一致。README 重写为当前最小运行指南，原说明完整保存在快照中，旧 WORK_LOG 不删除。
- 验证：整理前完整快照 1476 个文件/链接与 SHA-256/链接目标清单逐项一致。主构建脚本三包成功；新独立构建树 CTest 10/10（新增注册已有手柄4项测试），平台 pytest 13/13 通过；29 个 Python AST 与3个 Shell bash -n 检查通过；git diff --check 通过。OpenUSD ComputeAllDependencies 核对两个保留 USD 均为单层、无外部资产、无未解析引用。执行主入口，显式 keyboard、ISAAC_MANUAL_TIMEOUT=3、默认 Cloud+RViz，运行ID ego1p1_cleanup_smoke_20260923；成功加载4路320×240相机、0.40m ESDF、第三人称视图、在线地图及唯一控制发布者。183仿真帧、151命令帧、50Hz仿真时间命令频率，位置保持(-64,-11.2,2.8)，碰撞阻断0，MANUAL_TIMEOUT后退出码0。
- 失败/告警与边界：仿真加载内部 Humble rclpy 前尝试系统 rclpy 不可用，内部加载成功；退出时桥接 rosout 记录一次 context invalid 告警，节点仍正常退出。日志保存在快照 cleanup_smoke.log。未注入运动，guidance=0；本次仅证明整理后的完整启动/零输入/退出链可用，不声称动态绕障、实体手柄推杆或性能达标。手柄迁移由现有单元测试验证。EGO1P0 未修改。
- 回滚/备份：`/home/starry/isaac-data/备份/EGO1P1_before_cleanup_20260923_194915/EGO1P1` 为整理前完整文件树（含Git和未提交修改）；MANIFEST.json 校验来源，REMOVED_PATHS.json列出移出项目的路径。需要恢复时先保存后续工作，再按需取回，避免覆盖新的优化。
- 后续事项：算法优化继续在精简后的EGO1P1中进行，日常使用README中的两条命令。

### 2026-09-23 19:44 — [EGO1P1] 完整复制 EGO1P0 并建立后续算法开发主线

- 状态：完成。
- 目的：按用户要求将 EGO1P0 当前文件复制到 EGO1P1，之后以 EGO1P1 作为新开发主线。
- 项目路径：`/home/starry/isaac-data/EGO1P1`；来源 `/home/starry/isaac-data/EGO1P0`。
- 涉及文件：完整复制来源目录 1473 个文件/符号链接，包含 Git 历史、未提交修改、场景、性能记录、研究元数据及原 build/install/log/缓存。路径适配文件：`scripts/run_cloud_pillar_scene.sh`, `scripts/run_blender_isolated_cloud_scene.sh`, `scripts/prepare_ego_swarm_cloud_navigation.sh`, `scripts/prepare_ego_swarm_navigation.sh`, `scripts/prepare_flat_ground_navigation.sh`, `scripts/export_blender_cloud_to_isaac.sh`, `scripts/run_isolated_cloud_scene.sh`, `scripts/run_crack_maze_scene.sh`, `scripts/build_isaac_ros_workspace.sh`, `scripts/run_isaac_fov_gvf_navigation.sh`, `scripts/blender/generate_isolated_cloud_scene.py`, `scripts/isaac/generate_crack_maze_scene.py`, `scripts/isaac/generate_cloud_pillar_scene.py`, `scripts/isaac/generate_isolated_cloud_scene.py`, `KEYBOARD/run_keyboard_only.sh`, `KEYBOARD/run_joystick_only.sh`, `MISSANDKEYBOARD/run_joystick_avoidance.sh`。新增 `EGO1P1_VERSION.md`；更新 `AGENTS.md`、`README.md`、`DONG.md`、`USER_MANUAL_CONTROL_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`ISAAC_FOV_GVF_NAVIGATION.md`、两个输入入口 README、`WORK_LOG.md`；统一更新 HISTORY 的 README、PROJECT_INDEX、FOV_GVF_FAMILY 和 CHANGELOG。
- 修改内容：完整复制后先校验逐文件 SHA-256/符号链接目标一致，再将新目录可执行脚本中的绝对根路径改为 EGO1P1，构建前缀改为 `/tmp/fov_gvf_ego1p1_isaac_{build,install,log}`，默认 ROS 日志为 `/tmp/fov_gvf_ego1p1_isaac_ros_log`。复制的独立 Git 仓建立无上游本地 `ego1p1` 分支，继承 `6d4e577`，保留所有来源未提交修改；未创建提交、未推送或修改远程。EGO1P1 设为后续优化主线，EGO1P0 保留基线。
- 验证：复制前后清单 1473 项完全一致；适配后复核来源清单未变，算法 src 与场景 scenes 的非缓存文件字节一致。14 个 Shell 文件 `bash -n` 和 38 个 Python 文件 AST 语法检查通过，可执行源码无 EGO1P0 绝对路径残留；`git diff --check` 通过。`bash EGO1P1/scripts/build_isaac_ros_workspace.sh` 在新前缀成功构建 pc_gvf_msgs、pc_gvf_platforms、pc_gvf，三包 colcon_build.rc 均为 0；source Humble/新 install 并设 PYTHONNOUSERSITE=1 后，独立 pc_gvf 构建树 CTest 9/9 通过。未启动 Isaac、未执行手柄动态避障或性能实验。
- 影响与边界：算法、参数、输入语义、ROS 话题与场景不变。共用 ROS_DOMAIN_ID=42 和原单实例锁，继续禁止两个副本同时控制。来源版本说明、WORK_LOG 旧条目、performance 与 .research 历史保留原内容，不宣称旧实验已在新版本复验。原目录内复制的 build/install/log 可能含旧绝对路径，仅保留来源记录，正式入口使用本轮新建的独立 /tmp 产物。
- 回滚/备份：EGO1P0 原目录完整未改；新目录与构建产物独立，可在明确要求回滚时移除，并追加历史更正，不删除旧记录。
- 后续事项：后续算法修改默认在 EGO1P1 进行，并同步项目 WORK_LOG 和 HISTORY；动态仿真另行执行。

---

以下为完整继承的来源历史记录。

# Fov-GVF 分支工作日志

> 分支：`feature/local-modifications`  
> 基线分支：`dev`  
> 基线提交：`804f26e2e631d0571f5e1cc5088100481df1b16d`  
> 建立日期：2026-09-06

本文档用于记录 `feature/local-modifications` 分支的功能基线和后续变更。凡是对本分支源码、配置、脚本或文档的修改，都应在文末“修改记录”中追加一条记录。

---

## 一、当前项目已经完成的功能

### 1. ROS 1 到 ROS 2 的独立迁移

- 项目已从旧 ROS 1/catkin 工作区迁移为 ROS 2/ament 工作区。
- 当前仓库可以独立构建，不依赖旧 ROS 1 仓库中的文件。
- 上游文档以 ROS 2 Jazzy 为目标；本机已额外验证 ROS 2 Humble 可以完成构建、测试和演示运行。

### 2. 四个 ROS 2 功能包

| 功能包 | 当前职责 |
|---|---|
| `pc_gvf_msgs` | 定义统一的 `PositionCommand` 位置、速度、加速度和轨迹状态消息 |
| `pc_gvf_core` | 提供二维/三维流体场求解、深度栅格投影和笛卡尔空间导引算法 |
| `pc_gvf` | 提供深度视场角域 GVF 算法、ROS 2 控制器和合成深度演示 |
| `pc_gvf_platforms` | 提供平台适配、运动学仿真、手柄意图、轨迹回放和 RViz 可视化 |

### 3. 深度视场感知

- 接收 `sensor_msgs/msg/Image` 深度图和 `sensor_msgs/msg/CameraInfo` 相机内参。
- 支持 `32FC1` 米制深度图。
- 支持 `16UC1` 毫米制深度图。
- 将输入深度图缩放为角域规划网格，默认大小为 `48 × 36`。
- 对无效值、负深度和超出量程的数据进行清理和截断。
- 将深度像素反投影为空间障碍物点，用于安全距离和碰撞判断。

### 4. FOV/角域 GVF 局部避障

- 根据机器人尺寸和安全余量计算碰撞锥自由距离。
- 在相机视场中生成不可通行角域掩码。
- 根据目标方向、历史指令方向和障碍物分布选择安全目标。
- 在安全角域内求解调和势场/导引向量场。
- 对指令方向进行角速度限制，降低方向突变。
- 根据可用净空、制动距离和预测碰撞结果限制速度。
- 在视场不足、深度过期或导引无效时输出安全停止命令。

### 5. ROS 2 闭环控制器

控制器接收：

- 里程计；
- 深度图；
- 相机参数；
- 人工或自动导航意图。

控制器输出：

- `pc_gvf_msgs/msg/PositionCommand`；
- 当前导航状态；
- 视场、障碍点、参考方向、目标方向、命令方向和角域 GVF 可视化标记。

已实现的运行状态包括：

- `WAITING_ODOMETRY`：等待里程计；
- `WAITING_DEPTH`：等待深度图；
- `STALE_DEPTH`：深度数据超时；
- `STALE_INTENT`：导航意图超时；
- `NAVIGATING`：正在导航；
- `DEGRADED`：导引场退化；
- `GOAL_REACHED`：到达目标。

### 6. 自包含合成深度闭环演示

- 内置运动学无人机模型，无需 Gazebo、PX4 或 Isaac Sim 即可运行算法闭环。
- 合成发布里程计、深度图、相机参数和前进意图。
- 接收控制器速度命令并更新机器人位置。
- 对实际运动线段执行碰撞检查。
- 默认从约 `(0.0, 0.0, 1.2)` 飞向 `(7.2, 0.0, 1.2)`。

当前内置场景：

| 场景 | 用途 |
|---|---|
| `empty` | 无障碍直线导航基准 |
| `single_pillar` | 中央立柱绕障 |
| `offset_box` | 偏置箱体避障 |
| `center_sphere` | 中央球形障碍避障 |
| `overhead_bar` | 上方横杆和高度方向安全性 |
| `diagonal_gap` | 斜向间隙通过能力 |
| `narrow_gate` | 狭窄通道通过和安全停止能力 |

### 7. RViz 可视化

默认演示可以显示：

- 测试障碍物几何体；
- 无人机模型和当前姿态；
- 速度指令箭头；
- 已执行路径；
- 里程计；
- 前向深度图；
- 相机视场边界；
- 深度障碍点；
- 不安全角域；
- 参考射线、目标射线和命令射线；
- 角域 GVF 向量。

### 8. 地面平台适配

- 提供 `PositionCommand` 到 `geometry_msgs/msg/Twist` 的转换桥。
- 支持全向和差速运动学仿真模式。
- 提供地面机器人 RViz 配置。
- 对输出线速度和角速度进行限制。

### 9. 人工意图和确定性回放

- 支持将 ROS 2 `Joy` 手柄输入转换为 `TwistStamped` 导航意图。
- 支持死区、方向映射、最大速度和手柄超时保护。
- 支持从 YAML 文件回放确定性意图轨迹。
- 自带 `forward_stop.yaml` 前进—停止测试轨迹。

### 10. 测试与性能统计

- 包含二维/三维粗细网格边界回归测试。
- 包含深度投影和笛卡尔导引集成测试。
- 包含角域核心算法、深度解码、平台数学和可视化测试。
- 离线仿真会输出成功状态、碰撞状态、最终距离、最小净空、路径长度、速度、加速度和抖动指标。
- 算法内部记录 `solve_ms`，并输出计算耗时 P50、P95 和 P99。

### 11. 当前本机验证结果

验证环境：

- Ubuntu 22.04；
- ROS 2 Humble；
- Python 3.10；
- 使用 `/usr/bin/python3`；
- 使用 `PYTHONNOUSERSITE=1` 隔离用户目录中的 NumPy 2.x。

验证结果：

- 四个 ROS 2 包构建成功；
- C++ 与 Python 合计 13 项测试通过；
- `single_pillar` 图形和无界面演示均成功进入 `NAVIGATING`；
- 演示最终进入 `GOAL_REACHED`，未报告碰撞；
- 深度输入实测约 `8.9 Hz`；
- `PositionCommand` 输出稳定在约 `50 Hz`；
- `single_pillar` 离线算法耗时约为 P50 `1.09 ms`、P95 `8.29 ms`、P99 `8.92 ms`。

### 12. 已知限制和待处理事项

- 上游 `build_ros2.sh` 固定使用 `/opt/ros/jazzy`，不能直接用于本机 Humble 环境。
- 用户 Python 目录中的 NumPy 2.2.6 与 Ubuntu SciPy 1.8.0 不兼容；运行本项目时必须使用 `PYTHONNOUSERSITE=1`，或者建立独立兼容环境。
- `rosdep` 无法解析两个 Python 包中的 `ament_pytest` 测试依赖名称，但当前本机已有测试工具，实际测试可以运行。
- 运行时偶尔出现 `sequence size exceeds remaining buffer` DDS 反序列化警告；当前数据流仍能继续，但需要在后续修改中定位和消除。
- 当前默认飞行演示使用合成深度相机和运动学模型，并非真实相机或完整飞行动力学。
- 当前仓库尚未提供与 Isaac Sim、PX4 或真实无人机直接连接的专用适配器。
- 仓库没有给出明确的论文题名、DOI、BibTeX 或作者对应关系；相关论文仍需进一步确认。

---

## 二、后续修改记录规范

每次修改都在本节最上方追加记录，建议使用以下格式：

```text
### YYYY-MM-DD — 修改标题

- 目的：为什么修改。
- 涉及文件：修改了哪些文件。
- 修改内容：具体做了什么。
- 验证：执行了什么测试，结果如何。
- 遗留问题：尚未完成或需要注意的内容；没有则写“无”。
```

---

## 三、修改记录

### 2026-09-17 — 四视角水平360°避障与XY/Z独立控制

- 目的：将原前向约90°局部避障扩展到W/S/A/D和四个对角方向，同时保证升降输入不
  进入水平规划、水平避障不改写Z。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、
  `src/pc_gvf/src/depth_angular_core.cpp`、`depth_angular_controller_node.cpp`、launch、
  CMake及新增两项测试；`scripts/isaac/prepare_uav_navigation_scene.py`、
  `run_fov_gvf_navigation.py`、新增 `manual_control_math.py`；平地/Cloud导航USD及相关
  用户、版本、场景说明；同步更新 `HISTORY/CHANGELOG.md`。
- 修改内容：保留前向相机路径和 `/sim/depth/*`，新增左/后/右三台120°相机与话题，
  四视角覆盖水平360°；控制器用期望XY角选择最近相机。核心新增水平中线规划模式，
  仍使用原碰撞锥、调和导引、制动和rollout。键盘XY单独归一化，Z独立限幅；控制器
  拆分 `desired_xy/vz`，只规划XY后重新组合。纯Z直接发布且XY为零。ESDF/AABB末端
  保护改为分量检查，可在水平候选碰撞时保留安全Z，但不冒充三维规划。
- 验证：Python语法通过；Humble四包构建成功；`pc_gvf` 9/9测试通过。新增Python测试
  表覆盖用户要求的15种输入及Z保护，新C++测试覆盖八个水平方向、目标障碍偏转和零Z
  泄漏；四相机USD朝向检查为 `+X/+Y/-X/-Y`。首次沙箱内Isaac联动受ROS套接字权限
  限制失败；主机环境复跑成功创建四路160×120深度视角，123帧零输入保持
  `(0,0,1.5)`且碰撞阻断0。未注入物理键，真实动态15项仍待现场验收。
- 遗留问题：水平360°不提供上下感知或三维绕障；RViz在线点云继续只显示前向兼容
  话题。需要用户在Cloud障碍场实际操作15组输入完成最终动态验收。
- 载入修正：用户首次运行反馈场景未正常出现。检查13:01和13:02两次Kit及性能日志，
  确认USD均成功打开，但主入口默认值仍为历史键盘测试用 `flat`，实际载入的是零障碍
  `flat_ground_navigation.usd`，不是Cloud载入失败。已将完整主入口默认场景改为
  `cloud`，平地仅在显式设置 `FOV_GVF_SCENE_MODE=flat` 时使用，并同步修正文档。

### 2026-09-10 — 新增 USER-EGO 用户工程总览 DONG.md

- 目的：面向用户统一整理当前 USER-EGO 完成进展、运行链路、全部主要目录、文件命名原因、职责边界和参数修改位置。
- 涉及文件：新增 `DONG.md`，更新 `WORK_LOG.md`。
- 具体内容：按当前源码和 launch 实际状态区分主运行链、辅助工具、备用场景、历史迁移文件和未接入的 `pc_gvf_core` 三维势流研究库；记录当前自动导航、2 m/s、在线地图、ESDF、Isaac/RViz 第三人称和渲染配置。在线地图参数采用当前 launch 实际值 `map_resolution=0.08`、`publication_resolution=0.15`，并明确旧文档数值可能滞后。
- 验证：完成目录文件盘点、ROS 包构建项和入口注册核对；后续执行 Markdown 路径与关键名称检查。

### 2026-09-10 — RViz 无人机第三人称跟随视图

- 目的：让 RViz 像 Isaac Sim 的 ThirdPersonCamera 一样，从无人机后上方持续跟随观察。
- 修改：将 `isaac_cloud_navigation.rviz` 默认 View Controller 改为 `rviz_default_plugins/ThirdPersonFollower`，目标帧使用现有 `world -> radar` 动态 TF；默认 `Distance=8 m`、`Pitch=0.42 rad`、`Yaw=3.14159 rad`。原来的全局 Orbit 视图保留为 Saved View `Navigation overview`。
- 影响范围：只改变 RViz 相机显示，不改变 C++ 导航控制、深度输入、在线地图、Isaac Sim 运动或 ESDF 保护。

### 2026-09-10 — 提亮 Isaac Sim 主视口暗部

- 目的：解决 USER-EGO Cloud 场景在 Isaac Sim 主视口中暗处偏暗、第三人称观察细节不够清楚的问题，不改变 ROS 深度输入、导航控制和 ESDF 碰撞保护。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：复查本机 Isaac Sim 6.0 的 `omni.rtx.settings.core` 后确认其实际枚举为 `1 = Constant Diffuse`、`2 = Texture Diffuse`、`3 = Diffuse/Glossy/Emission`，与 `SimulationApp` 中遗留的参数说明不一致；此前使用模式 `3` 才是截图中黑色材质大面积不可见的直接原因。现将默认 `ISAAC_MINIMAL_SHADING_MODE` 修正为 `1`，设置中性浅灰蓝恒定颜色 `ISAAC_MINIMAL_CONSTANT_COLOR=0.60,0.65,0.70`。经过可见主视口逐组抓帧标定，最终采用 `ISAAC_SUN_INTENSITY=500`、`ISAAC_DOME_INTENSITY=0`、`ISAAC_AMBIENT_LIGHT_INTENSITY=0.15`：方向光提供几何轮廓，弱全局环境光抬起阴影，避免原 `3500/2400/2.0` 组合造成过曝；当载入的 USD 场景没有灯时，仍创建运行时灯光节点。
- 验证：Python 语法和 Bash 语法检查通过；ROS 2、C++ 控制器、Isaac Sim、深度相机、ESDF 和第三人称视口均完成短时联动启动。最终主视口抓帧 `/tmp/fov_gvf_constant_contoured.png` 的 `mean_rgb=161.21`，暗部可见且保留障碍轮廓；测试按设定超时正常退出，未改变导航控制参数。
- 遗留问题：默认 Constant Diffuse 更适合工程观察，但会弱化真实材质与光影层次；如需恢复材质颜色，可运行前设置 `ISAAC_MINIMAL_SHADING_MODE=2`。

- 后续对比：用户复验认为 Constant Diffuse 画面灰蒙、阴影不够黑。可见主视口对比测试后，将默认切回无随机采样噪点的 `2 / Texture Diffuse`，并将光照重新平衡为 `ISAAC_SUN_INTENSITY=800`、`ISAAC_DOME_INTENSITY=0`、`ISAAC_AMBIENT_LIGHT_INTENSITY=0.05`。最终抓帧 `/tmp/fov_gvf_texture_contrast.png` 的 `mean_rgb=72.30`；蓝色障碍、灰色地面和绿色无人机均可区分，阴影恢复为深色但没有大面积死黑，灰雾感消失。

### 2026-09-09 — 清除 Isaac Cloud 主视口黑白噪点

- 目的：修复 USER-EGO 运行时 Isaac Sim 第一/第三人称画面出现密集黑白及青色盐粒、难以观察无人机过程的问题，且不改变深度导航输入和控制算法。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：确认噪点只存在于 RGB 主视口，ROS 深度和控制链正常；先后复验 TAA、单 NVIDIA GPU、关闭 TV/film grain、色调映射抖动、随机采样光照、反射、AO 和间接漫反射，RaytracedLighting 截图均没有实质改善，因此没有将这些无效尝试单独当作修复。最终将正常观察默认改为 Isaac 官方 `MinimalRendering` 的 `Textured Diffuse` 模式，保留障碍物/地面/无人机材质色和几何遮挡，避开 156 万三角形体素 Cloud 在完整 RTX 光照下的持续像素噪声；固定 NVIDIA GPU 0 并关闭多 GPU，避免不受 RTX 支持的 AMD 核显参与；保留 `ISAAC_VIEW_RENDERER=RaytracedLighting` 作为显式画质诊断开关。抗锯齿只允许 `OFF/TAA/FXAA`，避免 DLSS/DLAA 破坏低分辨率度量深度标注。
- 验证：三次 GUI 短测逐项排除无效设置，最终短测同时显示 `[RENDER] mode=MinimalRendering`、深度相机 `320×240`、ESDF 正常加载、第三人称相机成功绑定，C++ 控制器从 `WAITING_DEPTH` 进入 `NAVIGATING` 并在 2 秒窗口内开始移动。相同第三人称截图的 3×3 局部中值残差均值由 `33.55` 降至 `0.16`，残差大于 40 的强噪声像素比例由 `26.67%` 降至 `0.09%`，唯一 RGB 颜色数由 `106783` 降为 `52`；人工查看确认盐粒噪声消失，障碍轮廓、地面和无人机可见。导航/ESDF/调和控制代码未修改。
- 遗留问题：默认工程模式不显示完整 RTX 阴影与反射；如用户主动切到 RaytracedLighting，当前超密体素 Mesh 的噪点可能重现，这是画质模式取舍，不影响默认导航。

### 2026-09-09 — USER-EGO 连续调和导引与运动平滑阶段 3–4

- 目的：继续阶段 1–2 的稳定化工作，消除离散像素选路、场刷新重锚、固定控制周期和一阶速度限幅造成的橙色命令箭头异常抖动，同时保持深度直达导航、固定终点、碰撞锥/rollout 安全限制和 ESDF 仿真保护结构。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/test/cpp/angular_field_check.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：阶段 3 在源像素邻域以加权最小二乘拟合调和势局部平面，用连续负梯度和 `2.5` 像素前视替代运行时离散路径第 3 点，遇到障碍时沿同一方向缩短前视；安全目标滞回权重增至 `0.40`，增加 `0.35 s` 最短保持且旧目标失效时立即重锚；节点保留世界系命令方向作为场刷新间的活动参考，调和 CG 使用上一有效势函数 warm start。阶段 4 将角速度限制改用实际 ROS 控制周期，加入显式命令加速度状态与 `8 m/s³` jerk 限制；速度恢复限制为 `1.0 m/s²`，预测制动允许 `2.5 m/s²`，两者非对称；只有导引射线进入约 `0.05 m` 接触距离才旁路平滑急停。首次实现误将所有 rollout 限速视为急停，实测出现 4 次在途单帧归零，随后拆分“预测性制动”和“真正急停”，并消除加速度上限切换及越过目标速度时的内部状态跳变。
- 验证：隔离 Isaac 安装重新构建成功；五项 C++ 核心测试全部通过，其中新增连续梯度无像素量化和 harmonic warm-start 一致性测试；Isaac Python 兼容环境中的 3 项既有 Python 基准测试通过。系统 Python 的两项 Python 测试仍因本机 NumPy 2.2.6 与 Ubuntu SciPy 二进制 ABI 不兼容而仅在收集期失败，与源码结果无关。最终完整 Cloud 场景从 `(-64,-11.2,2.8)` 出发，在 `t=70.35 s` 到达 `(63.274,11.862,4.926)`，无碰撞、无在途零速度脉冲；`/position_cmd` 记录 3517 帧、`50.02 Hz`，速度变化 P50/P95/最大值为 `0.00323/0.02144/0.08294 m/s`，加速度 P50/P95/最大值为 `0.1715/1.20/2.4883 m/s²`，方向角速度 P50/P95/最大值为 `0.0774/0.5980/1.2065 rad/s`，jerk P50/P95/最大值为 `0.928/8.000/8.000 m/s³`。相比阶段 1–2 的速度变化 P95 `0.040 m/s` 和 jerk P95 约 `60–70 m/s³`，异常控制抖动已消除；运行中累计约 `2.38 s` 的 `DEGRADED` 为调和场暂时无效时的连续限角回退，期间仍满足平滑约束，不属于箭头抖动。
- 遗留问题：当前实跑已满足抖动验收，无需继续用显示滤波掩盖控制问题。后续若要提高算法鲁棒性，可独立处理偶发 `DEGRADED`：只在源点所在自由连通域组装 Laplace 方程，并发布 solver 迭代数/残差诊断；这不是本阶段安全运行或平滑性的阻塞项。

### 2026-09-09 — USER-EGO 运行隔离与深度时域稳定阶段 1–2

- 目的：消除 RViz 原始控制箭头因重复控制器、低分辨率深度轮廓和单帧障碍释放造成的异常跳变，同时不改变既有调和场导航目标和安全限速结构。
- 涉及文件：`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/pc_gvf_platforms/navigation_visualizer.py`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、相关测试、`USER_EGO_GUIDE.md` 和重新生成的 Cloud 导航 USD。
- 修改内容：USER-EGO 默认隔离到 `ROS_DOMAIN_ID=42`，增加单实例文件锁、启动前零发布者检查、启动后单发布者验收和独立进程组清理；RViz 同时发布橙色原始命令箭头与紫色 0.20 秒显示专用低通箭头；深度相机垂直孔径由 `16` 改为 `18`，在保持 90 度水平视场时使 `fx=fy`（原 `160×120` 配置对应 80，升级到 `320×240` 后对应 160），输出分辨率由 `160×120` 提升为 `320×240`，Replicator 实时传感器渲染切换为 FXAA；C++ 角域规划掩码增加“新障碍立即占用、连续三张新深度帧为空才释放”的非对称滞回，并避免控制循环重复使用同一帧时错误累计；Python ROS 节点补充优雅处理外部关闭异常。
- 验证：Shell/Python 静态检查通过；三项 ROS 2 包构建成功；`pc_gvf_platforms` 6 项测试、5 项 C++ 核心测试及 Isaac Python 环境中的 3 项原有 Python 数值/行为基线全部通过。完整 Isaac Sim + RViz 实跑启动时确认 `/position_cmd` 发布者恰为 1，重复启动被锁拒绝；无人机在 `t=69.85 s` 从 `(-64,-11.2,2.8)` 到达 `(63.468,11.992,5.064)`，没有碰撞；控制遥测最大相邻速度变化 `0.04 m/s`、最大加速度 `1.20 m/s²`。相机内参强制修正告警消失；显式调用 Replicator FXAA 后 DLSS 低输入分辨率告警也在短测中消失。
- 遗留问题：当前阶段只稳定运行环境、感知输入和显示，不包含连续调和梯度、状态锚定参考或 jerk 限制；这些属于后续第三、四阶段。

### 2026-09-09 — 建立 user_ego 自动导航双视角副本

- 目的：保留 user 键盘版本不变，另建只负责自动起点到终点导航、并可切换 Robot 第一/第三人称相机的独立副本。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、副本中的路径与构建脚本、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：从提交 `d267feb` 重新复制到 `/home/starry/isaac-data/user_ego/Fov-gvf`；保持固定前进、固定终点和原 C++ 避障链路；增加绑定 Robot 的第三人称跟随相机及 `1/3/V` 视角切换；构建目录隔离为 `/tmp/fov_gvf_user_ego_isaac_*`。
- 验证：相关 Python 文件通过 `py_compile`，全部 Shell 脚本通过 `bash -n`；确认 launch 中 `fixed_forward_intent=True`、`use_fixed_goal=True`、`stop_at_goal=True`，并确认整个副本不存在 `/keyboard_intent`、运动键状态或 dead-man 代码。成功重新生成带两台 Robot 子相机的 4 倍 Cloud USD，三项 ROS 2 包在独立目录构建成功。Isaac Sim 与 RViz 实跑中默认启用 `/World/Robot/ThirdPersonCamera`，主视口平均 RGB `96.07`；无人机自动从 `(-64,-11.2,2.8)` 出发，经三维避障后在 `t=83.18 s` 到达 `(63.433,11.999,4.910)` 并进入 `GOAL_REACHED`，未触发碰撞保护。
- 遗留问题：无。

### 2026-09-09 — 接入 4 倍 EGO-Swarm Cloud 场景与 ESDF 三维碰撞保护

- 目的：在独立分支使用 `/home/starry/isaac-data/ego_swarm_cloud` 的 Perlin 三维云场景继续无人机避障任务，并按用户要求不用合并 Mesh 的 AABB、加入 RViz、将地图整体放大 4 倍。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/prepare_ego_swarm_cloud_navigation.sh`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、`src/pc_gvf_platforms/test/test_platform_math.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`scenes/ego_swarm_cloud/`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：环境 Mesh 从 `40×30×5 m` 统一放大为约 `160×120×20 m`，无人机不缩放；场景准备器记录 4 倍比例并把相机量程同步扩至 `40 m`。复制 `400×300×50` 的 `occupancy.bin`，在 Isaac Python 中通过欧氏距离变换建立 ESDF，分辨率随场景变为 `0.4 m`，候选位置使用三线性插值并按 `0.25 m` 机体半径拦截碰撞，不对单体 Cloud Mesh 使用 AABB。速度桥新增 `z` 速度与 `max_vz`，仿真运动学积分改为三维，控制器最大垂直速度设为 `1.0 m/s`。起终点为 `(-64,-11.2,2.8)`、`(64,11.2,4.8)`；RViz 默认启动，网格和观察距离扩大到大场景尺度。
- 验证：导航 USD 重新生成成功，OpenUSD 复核确认场景边界约为 `160×120×20 m`、包含 `/World/PhysicsScene`、Robot、DepthCamera 和 2 个静态碰撞 Mesh；运行时成功从 `occupancy.bin` 建立分辨率 `0.4 m` 的 ESDF。ROS 2 三包构建成功；`pc_gvf_platforms` 6 项测试全部通过（含新增垂直速度限幅），`pc_gvf` 5 项 C++ 测试通过。Isaac Sim 与 RViz 同时启动，RViz 使用 OpenGL 4.6，Isaac 主视口平均 RGB 为 `204.27`；无人机从 `(-64,-11.2,2.8)` 出发，经 `(-28.77,-2.97,3.05)`、`(-0.33,-4.02,3.13)`、`(28.10,2.58,1.87)`、`(57.14,9.21,3.57)` 完成三维避障，在 `t=68.52 s` 到达 `(63.331,11.927,4.734)` 并进入 `GOAL_REACHED`，未触发碰撞保护。Python/Shell 静态检查和 `git diff --check` 通过。
- 遗留问题：`pc_gvf` 的 2 项旧 Python 测试在系统 Python 环境收集阶段因 NumPy 2.2.6 与旧 SciPy 二进制不兼容而报错；这不是本次 ESDF 运行环境的问题，Isaac Python 内的 NumPy 2.5.2 与 SciPy 1.18.1 已实际完成 ESDF 构建和闭环运行。

### 2026-09-09 — 完成 EGO-Swarm Blender 场景运行接入

- 目的：补完上一项未完成的 `ego_swarm_blender` Isaac Sim 导航场景接入，并进行真实运行验证。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/prepare_ego_swarm_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm/ego_swarm_navigation.usd`、`WORK_LOG.md`。
- 修改内容：导航场景补充 `/World/PhysicsScene`；使用 501 个静态碰撞 Mesh、前向深度相机和 C++ 深度角域控制器执行约 60 m 横穿任务。首次终点为 `(30,-1,1.2)`；在该目标附近触发碰撞保护后，另准备了净空更大的 `(25,4,1.2)` 终点配置，但按用户决定不继续以到达终点作为此项接入验收条件。
- 验证：Python/Shell 语法与 `git diff --check` 通过；三项 ROS 包成功重建。Isaac Sim 实跑成功加载场景，识别 500 个空中障碍，视口平均 RGB `223.93`；控制器进入 `NAVIGATING` 并以接近 `2 m/s` 从 `(-30,0,1.2)` 飞至 `(25.772,-0.248,1.2)`，期间曾在局部无有效场时进入 `DEGRADED`，最终由 `COLLISION_GUARD` 安全拦停，没有穿越障碍。用户确认能正常运行即可，允许进入下一项。
- 遗留问题：在此超长、密集的柱体/圆环随机场景中，当前局部深度角域算法未保证到达远端目标；这是算法可达性问题，不是 Isaac/ROS 场景接入故障。

### 2026-09-09 — 保存 EGO-Swarm Blender 场景适配中间检查点

- 目的：在转向 `ego_swarm_cloud` 场景前，保存当前 `feature/ego-swarm-blender-navigation` 分支已完成的场景泛化工作，避免丢失并为新分支提供基础。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/prepare_ego_swarm_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm/ego_swarm_navigation.usd`、`WORK_LOG.md`。
- 修改内容：场景准备器支持通过命令行创建起终点；运行器的灯光和碰撞保护由旧场景固定路径/14 障碍泛化为按 USD 类型与飞行高度识别；默认场景切换为由 `/home/starry/isaac-data/ego_swarm_blender/ego_swarm.usd` 派生的导航 USD；设置 `(-30,0,1.2)` 至 `(30,-1,1.2)` 横穿任务和大场景观察相机。
- 验证：相关 Python、Shell 语法检查和 `git diff --check` 已通过；导航 USD 成功生成，OpenUSD 检查确认 501 个 Mesh/碰撞体、起点、终点、无人机和深度相机存在。
- 后续完成：场景准备器已补充 `/World/PhysicsScene`；导航 USD 重新生成、ROS 包重新构建和 Isaac 闭环实跑结果见同日后续完成记录。

### 2026-09-08 — 保存当前 Isaac C++ 深度导航基线

- 目的：在适配 `ego_swarm_blender` 新场景前保存当前可运行成果，确保后续场景迁移可以独立开发和安全回退。
- 涉及文件：`.gitignore`、当前工作区内尚未提交的 C++ 控制器、ROS launch、Isaac/Blender 脚本、场景、测试、方案文档及 `WORK_LOG.md`。
- 修改内容：把当前 `feature/cpp-3d-voxel-flow` 的项目成果整理为 Git 基线；将 `build_current_humble/`、`build_humble/`、`install_current_humble/`、`install_humble/`、`log_current_humble/`、`log_humble/` 明确列为可再生成构建产物并排除在提交外。
- 验证：提交前执行 `git diff --check`；具体提交号和新分支状态在完成 Git 操作后补充。
- 遗留问题：此基线仍使用 `scenes/blender_isolated_clouds` 的 seed 42 云团场景；`ego_swarm_blender` 场景适配将在新分支完成。

### 2026-09-08 — 将 Isaac 场景导航速度提高到 2 m/s

- 目的：把当前 C++ 深度角域控制器在 Isaac 云团场景中的目标移动速度由 `0.45 m/s` 提高到 `2.0 m/s`。
- 涉及文件：`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`ISAAC_FOV_GVF_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：将控制器的 `speed` 与 `max_speed` 同步设为 `2.0`，并将下游 `position_cmd_to_twist` 的 `max_vx`、`max_vy` 同步放宽至 `2.0`，避免适配器二次限速；保留控制器原有加速度限制、安全制动、深度 rollout 和避障降速逻辑。
- 验证：launch Python 语法检查和 `git diff --check` 通过；使用 `scripts/build_isaac_ros_workspace.sh` 将 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三包重新构建到 `/tmp/fov_gvf_isaac_install`，构建成功，并确认安装后的 launch 四个速度参数均为 `2.0`。随后以关闭 RViz 的方式完成 Isaac 图形闭环实跑：控制器进入 `NAVIGATING`，无人机从 `(-8,0,1.2)` 经场景上侧绕障，在 `t=29.72 s` 到达 `(7.711,0.074,1.200)`，未触发 `COLLISION_GUARD`。同一任务原 `0.45 m/s` 配置耗时约 `39.5 s`。一次未设置 `ROS_LOG_DIR` 的 `ros2 launch --show-args` 辅助检查因沙箱禁止写入 `~/.ros/log` 而失败，与 launch 内容无关；正式脚本使用 `/tmp/fov_gvf_isaac_ros_log` 并成功运行。
- 遗留问题：`2.0 m/s` 是目标速度与硬上限，不是强制恒速；本次障碍密集场景中安全制动和 depth rollout 将日志采样点的实际平移命令主动降至约 `0.45 m/s`，因此已经允许空旷区加速到 `2.0 m/s`，但不会为追求恒定 `2.0 m/s` 而绕过避障安全约束。

### 2026-09-08 — 修复 Isaac 黑视口并接入 RViz 同步观察

- 目的：解决闭环运行期间 Isaac Sim 主视口全黑、且一键脚本不打开 RViz 的可视化问题。
- 涉及文件：`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`ISAAC_FOV_GVF_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：最初在非 headless 模式下移动 `/OmniverseKit_Persp`，但用户复验确认 Isaac 主视口仍为黑屏；现改为创建独立 `/World/NavigationOverviewCamera`，等待主视口 ready，并在 ROS 深度 RenderProduct 创建完成后把活动视口显式绑定到该相机，防止传感器 RenderProduct 影响 UI 视图；增加实际主视口 RGB 截图和平均亮度诊断；观察点和目标点可分别通过 `ISAAC_VIEW_EYE`、`ISAAC_VIEW_TARGET` 调整。诊断阶段主视口与独立离屏截图均为 `mean_rgb=0.00`，进一步确认并非相机或视口绑定，而是 Blender USD 的 Sun/Dome 强度仅为 `0.55/1.0`、不适合 Isaac RTX；运行时现默认提升为 `3000/900`，并允许通过 `ISAAC_SUN_INTENSITY`、`ISAAC_DOME_INTENSITY` 覆盖。验证根因后移除临时的第二路离屏 RenderProduct，避免正式运行重复渲染。导航 launch 默认同时启动 `navigation_visualizer` 和使用仿真时间的 `rviz2`；Isaac 专用 RViz 配置显示无人机、命令、轨迹、里程计、实时深度图、FOV、深度点和角域 GVF；一键脚本支持用 `FOV_GVF_RVIZ=false` 显式关闭 RViz。
- 验证：第一版 Shell、Python 和 launch 静态检查及 `git diff --check` 均通过；`pc_gvf_platforms` 增量构建成功，新 RViz 配置已安装到 `/tmp/fov_gvf_isaac_install`，平台包 5 项测试全部通过。实际图形运行确认 `rviz2` 进程成功启动并使用 OpenGL 4.6，控制器进入 `NAVIGATING`；无人机经 `(0.14,-1.00)`、`(4.45,-1.96)`、`(6.15,-1.38)` 绕障后，在 `t=39.47 s` 到达 `(7.766,-0.185,1.200)` 并进入 `GOAL_REACHED`，未触发碰撞保护。用户随后确认 RViz 正常但 Isaac 仍黑；第二轮实跑确认活动视口已绑定 `/World/NavigationOverviewCamera`、分辨率为 `1280×720`，但主视口与独立离屏相机截图平均 RGB 均为 `0.00`，从而将根因收敛到可见光照。将 Sun/Dome 提升至 `3000/900` 后，主视口截图平均 RGB 恢复为 `97.88`、独立诊断相机为 `98.43`，截图可清楚看到地面、14 个障碍物和无人机；同次闭环在 `t=39.53 s` 到达 `(7.768,-0.183,1.200)`，未触发碰撞保护。移除临时离屏诊断后，Shell、Python 静态检查和 `git diff --check` 再次通过。
- 遗留问题：RViz 当前显示实时感知到的深度障碍点，不显示从 USD 读取的完整障碍 Mesh，以保证控制可视化与控制器实际观测一致。

### 2026-09-08 — 恢复 Blender 云团场景的首次 C++ 控制器闭环运行

- 目的：使用当前已部署的 C++ 深度角域控制器，让运动学四旋翼在 Blender 导入的 14 障碍场景中完成首次真实避障运行。
- 涉及文件：`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`WORK_LOG.md`。
- 修改内容：修正导航 launch 遗留的终点坐标方向错误，将 `goal_x` 从起点 `-8.0` 改为导航 USD 中 `/World/UAV_Goal` 的实际 X 坐标 `8.0`；首次实机运行确认深度和 CameraInfo 发布者存在且约为 136–137 Hz，但控制器因仿真时间消息与墙钟不一致持续报告 `STALE_DEPTH`，因此在 OmniGraph 中增加 `/clock` 发布并让控制器/速度适配器启用 `use_sim_time`；只在 Isaac 子进程中清除系统 ROS 2 Python 3.10 路径，并加载 Isaac Sim 内置 Humble bridge 库，避免污染其 Python 3.12 环境；加入机体 yaw、body/world Twist 正确互换和零速原地扫描；把控制器的历史命令与历史安全目标改为世界系保存并按当前相机姿态逐帧重投影；最终利用全向平台的独立航向自由度让 90 度前视相机持续朝向终点，而平移仍执行 GVF 指令，并将安全余量设为 `0.20 m`、速度设为 `0.45 m/s`。
- 验证：终点、Shell 和 Python 静态检查通过；ROS 2 三包干净构建成功，修改后的 `pc_gvf` 增量构建成功，隔离安装中的 C++ 控制器、速度适配器和修正后的 launch 均存在；`pc_gvf` 的 7 项 C++/Python 测试全部通过。前十一次运行依次定位并排除了时间基准、Isaac Python 环境、固定相机、历史像素坐标系、过宽视场退化及擦边安全余量问题；其中 120 度和 105 度视场均安全停在起点，未被误判为成功。最终第十二次运行从 `(-8.00,0.00,1.20)` 出发，经 `(-6.08,0.78)`、`(-3.93,0.87)`、`(-1.99,-0.19)`、`(0.07,-0.96)`、`(2.18,-1.56)`、`(4.38,-1.96)`、`(6.09,-1.40)` 绕过障碍，在 `t=39.60 s` 到达 `(7.765,-0.184,1.200)` 并进入 `GOAL_REACHED`；全过程未触发 `COLLISION_GUARD`，闭环验收通过。
- 遗留问题：当前成功结果针对 seed 42 的 14 障碍场景和运动学全向四旋翼；尚未验证其他随机种子，也尚未接入真实旋翼动力学或飞控。当前接管路线仍是 C++ 深度角域控制器，不代表 C++ 三维体素势流已经接管。

### 2026-09-08 — 将无人机和 FOV-GVF 导航适配到 Isaac Sim 场景

- 目的：把当前 Blender 云团环境扩展为可放置无人机、可发布深度/里程计并接收 FOV-GVF 速度命令的 Isaac Sim 导航场景，起点到终点采用定高绕障任务。
- 涉及文件：`scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/build_isaac_ros_workspace.sh`、`scripts/export_blender_cloud_to_isaac.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`ISAAC_FOV_GVF_NAVIGATION.md`、`scenes/blender_isolated_clouds/isolated_clouds_seed42_navigation.usd`、`WORK_LOG.md`。
- 修改内容：在 USD 中加入 `/World/Robot` 四旋翼可视模型、前向 `/World/Robot/CameraMount/DepthCamera`、机体半径和导航元数据；新增 Isaac Sim ROS 2 OmniGraph 适配，发布 `/sim/odom`、`/sim/depth/image_raw`、`/sim/depth/camera_info`，订阅 `/sim/cmd_vel` 并按速度命令更新无人机；新增固定终点、定高、低速和安全余量参数的 ROS 2 launch；一键导出脚本现在自动生成环境 USD 和导航 USD；新增干净构建脚本。
- 验证：导航 USD 的 OpenUSD 检查确认起点 `(-8,0,1.2)`、终点 `(8,0,1.2)`、Robot、DepthCamera 和 14 个障碍均存在；ROS 2 三包工作区使用 `/tmp/fov_gvf_isaac_install` 干净构建成功。受限沙箱闭环尝试未完成：控制器和速度适配器启动，但 DDS 被 `getifaddrs/UDP socket Operation not permitted` 阻断，Isaac Sim 另报告 `NVML_ERROR_DRIVER_NOT_LOADED`，无人机安全停在起点；这属于运行环境限制，不能记作导航成功。非沙箱 GPU/ROS 运行仍需在具备 NVIDIA 驱动和 DDS 网络权限的环境中复验。
- 遗留问题：当前 Isaac 适配使用运动学四旋翼（速度接口），尚未接入真实旋翼动力学/飞控；未完成具备 GPU 和 DDS 权限环境中的最终 `GOAL_REACHED` 闭环验收。

### 2026-09-08 — 将 Blender 云团场景导入 Isaac Sim

- 目的：保留当前 Blender 母版，并将其材质、14 个障碍及起终点导出为可直接载入 Isaac Sim 的物理 USD。
- 涉及文件：`scripts/blender/export_scene_to_usd.py`、`scripts/isaac/prepare_blender_cloud_usd.py`、`scripts/isaac/view_blender_cloud_scene.py`、`scripts/export_blender_cloud_to_isaac.sh`、`BLENDER_ISOLATED_CLOUD_SCENE.md`、`scenes/blender_isolated_clouds/`、`WORK_LOG.md`。
- 修改内容：新增 Blender Z-up/米制 USD 导出脚本；新增 OpenUSD 后处理，为场景加入 PhysicsScene，为地面和每个云团 Mesh 加入精确静态 CollisionAPI/MeshCollisionAPI，并为 `UAV_Start`、`UAV_Goal` 写入显式角色属性；增加独立 Isaac Sim 查看器和可重复的一键导出命令。
- 验证：Blender USD 导出和 Isaac 后处理脚本通过语法检查；Blender 5.2.1 成功输出原始 USD，OpenUSD 复核确认 Z-up、1 米单位、14 个云团 Mesh、起终点均存在；后处理后 `PhysicsScene`、地面碰撞和全部 14 个精确静态 MeshCollisionAPI 均存在，起终点角色属性分别为 `uav_start`/`uav_goal`；Isaac Sim 6.0.1 在 RTX 5070 上成功 `open_stage returned True` 并打印场景就绪信息。后续导航验证发现 Blender 的可选 USD 前向轴转换会镜像已在 Z-up 世界中 authored 的 X 方向，已关闭 `convert_orientation` 并将 Isaac 导航目标与实际 `/World/UAV_Goal` 读取保持一致。
- 遗留问题：尚未把实际无人机、深度相机和 ROS 2 控制闭环加入该 USD。

### 2026-09-08 — 在 Blender 中实现独立云团柱体地图

- 目的：将独立不规则云团地图实现为 Blender 原生可编辑三维场景，并保留明确的无人机起点和终点。
- 涉及文件：`scripts/blender/generate_isolated_cloud_scene.py`、`scripts/run_blender_isolated_cloud_scene.sh`、`BLENDER_ISOLATED_CLOUD_SCENE.md`、`scenes/blender_isolated_clouds/`、`WORK_LOG.md`。
- 修改内容：通过 Blender Python API 生成白色地面、独立封闭云团柱体 Mesh、起终点彩色圆盘及位于 `z=1.2 m` 的 `UAV_Start`/`UAV_Goal` 空对象；使用障碍外接圆执行保守最小间距约束；保存 `.blend`、JSON manifest 和顶视 PNG，并增加一键生成后打开 Blender 的脚本。
- 验证：系统 Python 语法检查和 Shell `bash -n` 通过；使用已安装 Blender 5.2.1 LTS 后台成功生成并渲染 seed 42，输出 `.blend`、JSON 和 PNG，保守保证的最小障碍间隙为 `0.936 m`（请求 `0.90 m`）；重新打开 `.blend` 检查确认 `Cloud_Obstacles` 恰有 14 个对象、Ground 和 Overview_Camera 存在，`UAV_Start=(-8,0,1.2)`、`UAV_Goal=(8,0,1.2)`；`git diff --check` 通过。首次实测发现该 Blender 构建的 Eevee 枚举仍为 `BLENDER_EEVEE` 且空场景没有 World，修正后生成成功；后台运行仍打印 PulseAudio mainloop 权限提示和无法写用户缩略图缓存提示，但不影响 `.blend` 与预览输出。
- 遗留问题：尚未导入具体无人机模型，也未导出 USD 或在 Isaac Sim 中复核材质、碰撞与坐标层级。

### 2026-09-08 — 新增独立不规则云团场景生成器

- 目的：新增障碍物彼此不直接连接、每团形状不规则且团间留出无人机通行空隙的场景生成方式，保留原云状随机场生成器不变。
- 涉及文件：`scripts/isaac/generate_isolated_cloud_scene.py`、`scripts/isaac/view_isolated_cloud_scene.py`、`scripts/run_isolated_cloud_scene.sh`、`ISAAC_ISOLATED_CLOUD_SCENE.md`、`WORK_LOG.md`。
- 修改内容：使用带低频角向扰动的椭圆云团逐个放置；通过栅格膨胀约束保证云团间最小间距；只保护起终点圆盘而不人工挖出显眼主通道；验证黑色连通分量数量与云团数量相同、带安全净空的自由空间连通，并将黑色区域复用现有精确静态 Mesh 挤出为三维柱体。
- 验证：初始默认 16 团在保留人工主通道的尺寸和间隙约束下安全拒绝（只能放置 8 团）；中间参数成功生成 10 个分量，但呈上下两排，因此改为全场随机分布 14 团并仅保护起终点；最终使用欧氏距离场执行团间间隙约束和实际最小间隙复核。新增脚本通过 `py_compile` 和 `bash -n`，seed 12、27、42、68、103、205 均成功生成，每张恰有 14 个黑色连通分量、带净空自由区为 1 个连通分量，实测最小团间距为 `0.914–0.937 m`，均不小于请求的 `0.90 m`；默认 seed 42 的 OpenUSD 复核确认精确静态 CollisionAPI/MeshCollisionAPI 存在且 approximation 为 `none`；`git diff --check` 通过。
- 遗留问题：尚未与无人机、深度相机和 Fov-gvf 闭环合入同一启动脚本。

### 2026-09-08 — 新增全连通裂隙迷宫场景生成器

- 目的：保留原有白底黑色云状障碍生成方式，同时新增“从全黑障碍中挖出全连通白色裂隙道路”的 Isaac Sim 场景，包含分岔、死胡同和急转弯，且不产生独立自由空间。
- 涉及文件：`scripts/isaac/generate_crack_maze_scene.py`、`scripts/isaac/view_crack_maze_scene.py`、`scripts/run_crack_maze_scene.sh`、`ISAAC_CRACK_MAZE_SCENE.md`、`scenes/crack_maze/crack_maze_seed42.{usd,json}`、`scenes/crack_maze/crack_maze_seed42_map.png`、`WORK_LOG.md`。
- 修改内容：使用偏向长路径并随机回访旧节点的 Growing Tree 迷宫算法，在粗网格上建立覆盖全部路口且具有更多分岔的生成树，再将带路口扰动和逐边宽度变化的树边栅格化为白色道路；选择树直径端点作为起终点；强制验证全部白色区域只有一个连通分量，以及起终点在指定净空阈值下连通；将其余黑色区域拉伸为带精确静态 Mesh 碰撞的三维障碍，并增加独立生成、查看和一键启动入口。原云状场景脚本未修改。
- 验证：两个新增 Python 脚本通过 `py_compile`，新增 Shell 启动器通过 `bash -n`，`git diff --check` 通过；使用 Isaac Sim 自带 Python 成功生成 seed 42，并对 seed 1、7、103、999 做额外生成抽查；seed 42 的全部白色区域和按 `0.35 m` 净空收缩后的中心区均各为 1 个连通分量，具有 7 个分岔点、9 个末端，实际黑色障碍占比约 51.51%；OpenUSD 复核确认 default prim、PhysicsScene、MazeField、CollisionAPI、MeshCollisionAPI 均存在，碰撞近似为 `none`；Isaac Sim 6.0.1 在 RTX 5070 上成功打开新 Stage，打印 `open_stage returned True` 及三行 `[SCENE READY]`，查看器保持运行。
- 遗留问题：场景尚未与无人机、深度相机和 Fov-gvf 闭环合入同一启动脚本；尚未用动态刚体执行接触碰撞实测。

### 2026-09-08 — 新增 Isaac Sim 云状柱障碍场景生成器

- 目的：按用户提供的俯视参考图生成随机黑色障碍/白色通道，并将二维障碍区域拉伸为 Isaac Sim 中可碰撞的三维云状柱体。
- 涉及文件：`scripts/isaac/generate_cloud_pillar_scene.py`、`scripts/isaac/view_cloud_pillar_scene.py`、`scripts/run_cloud_pillar_scene.sh`、`ISAAC_CLOUD_SCENE.md`、`scenes/cloud_pillars/cloud_pillars_seed42.{usd,json}`、`scenes/cloud_pillars/cloud_pillars_seed42_map.png`、`WORK_LOG.md`。
- 修改内容：使用固定种子的多尺度高斯随机场生成平滑连通斑块；清出弯曲飞行走廊并用距离变换检查起终点宽通道连通性；将占据栅格合并为一个带精确静态三角网格碰撞体的黑色拉伸柱体；输出 USD、manifest 和二维占据图；增加显式载入 Stage、检查 CloudField 并设置固定斜俯视视角的 Isaac Sim 查看器和一条启动命令。
- 验证：生成器和查看器通过 `py_compile`，启动脚本通过 `bash -n`；使用 Isaac Sim 自带 Python/OpenUSD 成功生成并重新打开 seed 42 场景。默认实际障碍占比约 53.85%，验证通道净空阈值为 0.5225 m，网格包含 84884 个顶点和 21221 个四边形面；独立 OpenUSD 复核确认 default prim、PhysicsScene、CloudField、CollisionAPI 和精确 MeshCollisionAPI 均存在；Isaac Sim 6.0.1 在 RTX 5070 上打印 `open_stage returned True` 以及三行 `[SCENE READY]`，自动斜俯视查看器保持运行。
- 遗留问题：当前场景只包含障碍、地面、标记和灯光，尚未把 ARL Robot、深度相机和 Fov-gvf 控制闭环合入同一个启动脚本；尚未用动态刚体执行接触碰撞实测。

### 2026-09-07 — 记录深度图直驱三维势流导航的换账号交接方案

- 目的：把用户最新确定的“深度图直接导航、不建立持久三维地图/ESDF”的技术路线整理为可脱离原聊天记录独立阅读的交接文档，便于更换账号后继续实施。
- 涉及文件：`DIRECT_DEPTH_3D_NAVIGATION_HANDOFF.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增换账号交接文档，明确删除点云构造、跨帧融合、占据地图和 ESDF 环节，但保留短生命周期局部三维求解网格；
  - 记录目标数据流、选择理由、当前已经完成和尚未完成的真实状态、约 `8.2 ms` 的三维导引性能基线及第一轮 CPU 优化顺序；
  - 固化 `Hit / NoReturn / Invalid`、Unknown、`TrustedFree` 和 fail-closed 安全边界；
  - 记录与 FlowForge 点云/ESDF 路线的可复用部分和不能直接复制的边界；
  - 指定新账号从阶段 `3D-1` 接手，并列出阅读顺序、单步任务、退出条件、关键文件和脏工作树保护要求。
- 验证：仅新增/更新 Markdown 文档，未修改算法和运行配置；已检查文档内容与当前分支 `feature/cpp-3d-voxel-flow`、现有源码状态及已记录性能数据一致，未把未执行的测试写为通过。
- 遗留问题：阶段 `3D-1` 尚未开始；下一步先实现显式深度像素语义及其纯 C++ 测试，正式 `/position_cmd` 继续由当前 C++ 角域控制器独占。

### 2026-09-07 — 建立 C++ 三维体素势流路线分支与实施方案

- 目的：在已经完成 C++ 深度角域迁移的基础上，建立独立开发分支，核实现有三维体素/势流能力与缺口，并制定不影响当前稳定控制路径的分阶段切换方案。
- 涉及文件：`CPP_3D_VOXEL_FLOW_PLAN.md`、`WORK_LOG.md`；新分支为 `feature/cpp-3d-voxel-flow`。
- 修改内容：
  - 从当前 `feature/local-modifications` 工作状态建立 `feature/cpp-3d-voxel-flow`，完整保留尚未提交的迁移成果和用户已有构建目录，不删除、不重置任何现有修改；
  - 阅读并核对 `pc_gvf_core` 的 `DepthFrame3D`、深度栅格投影、三维有限体积 Poisson 系统、粗细场边界先验、矩阵无关 CG、三线性场采样、停滞横流锁存、积分流线和横向误差反馈实现；确认当前 `pc_gvf` 没有调用 `FluidGuidance::calcGuidance3D()`；
  - 对照论文截图和 `FOV_GVF_原理与CPU方案分析.md`，区分已有工程近似与论文条件：当前参考是积分折线最近点反馈，不是完整流盒坐标/Jacobian 伪逆；当前三维核心也没有最终连续运动和制动尾段认证；
  - 新增三维路线计划，明确采用局部滚动三维状态栅格而非持久全局地图，复用 `pc_gvf_core`，按纯 C++ 契约加固、流线/安全核心、非执行 ROS 影子、仿真可视化、闭环 CPU 验收、可回退切换六个实现阶段推进；
  - 明确零深度/无回波/无效值、近相机 `TrustedFree`、数据 freshness、垂向控制和未知空间是正式接管前的强制修正项；当前已验收的 C++ 角域控制器继续独占正式输出。
- 验证：
  - `git branch --show-current` 确认为 `feature/cpp-3d-voxel-flow`；
  - 使用独立 `/tmp/fov_gvf_3d_analysis_{build,install,log}` 前缀以 CMake Release 构建 `pc_gvf_core` 成功；
  - 现有 `depth_guidance_integration_check --benchmark` 各测 40 次：单线程粗细深度投影 P50/P95 约 `7.472/7.489 ms`，完整三维导引约 `8.256/8.268 ms`；四线程分别约 `7.464/7.476 ms` 与 `8.138/8.208 ms`；
  - 四线程收益很小，证实现阶段首要 CPU 瓶颈是逐体素深度包围框扫描；当前结果可支持 10 Hz 场更新加 50 Hz 缓存场跟踪的首版设计，但不能证明完整链路小于 1 ms。
- 遗留问题：本记录只完成分支、源码核对、性能基线和实施设计，尚未修改默认控制算法；下一步从阶段 3D-1 开始，先加固深度三态语义、近域证据和结构化失败结果，未经影子及闭环验收不得让三维路线发布 `/position_cmd`。

### 2026-09-07 — 阶段 5.2：清理影子迁移设施并完成实现收口

- 目的：在 Python 控制器已经删除且正式 C++ 路径通过闭环验收后，移除只为双实现并行验证而存在的影子构建、启动入口、诊断数据和在线对照工具，将运行结构收敛为单一正式 C++ 控制器。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`（重命名为 `src/pc_gvf/src/depth_angular_controller_node.cpp`）、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 将同时生成 active/shadow 两个目标的 CMake 循环改为只构建和安装 `depth_angular_controller`，删除 `PC_GVF_SHADOW_MODE` 编译定义及 `depth_angular_shadow` 安装目标；
  - 将共享源码正式命名为 `depth_angular_controller_node.cpp`，固定节点名、`cmd_topic` 参数和 `/pc_gvf/angular_field` 输出，删除影子命令/可视化话题分支及影子模式日志；
  - 删除迁移期 `~/diagnostics` JSON 发布器、逐帧汇总与求解计时；该接口只被双实现对照探针消费，不属于冻结公共控制契约，移除后也减少了每个控制周期的字符串格式化开销；
  - demo launch 删除 `shadow` 参数和条件影子进程，保留正式控制器、合成模拟器、平台可视化和 RViz 结构；删除已经失去第二实现输入源的 `tools/ros_shadow_parity_probe.py`；
  - 明确保留 Python 数值核心、场景/合成模拟器、黄金数据生成器、C++ 闭环与状态探针作为持续回归基础；保留 `pc_gvf_core` 独立二维/三维 C++ API，不把仍有独立功能的模块当作迁移垃圾删除。
- 验证：
  - 使用全新的 `/tmp/fov_gvf_stage52_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功，`depth_angular_controller_node.cpp` 在 `-Wall -Wextra -Wpedantic` 下编译、链接成功；全量测试汇总 18 项，0 错误、0 失败、0 跳过；
  - 干净安装的 `ros2 pkg executables pc_gvf` 只列出 `depth_angular_controller`、`depth_angular_core` 和 `depth_angular_demo_sim`，不存在 `depth_angular_shadow`；launch 参数只剩 `rviz` 与 `scenario`；
  - 安全状态探针再次完整观测 `WAITING_ODOMETRY → STALE_INTENT → ZERO_INTENT → WAITING_DEPTH → NAVIGATING → STALE_DEPTH → NAVIGATING → STALE_INTENT → INVALID_ODOMETRY → INVALID_GUIDANCE → GOAL_REACHED`，共收到 290 个命令样本，保护状态持续零命令、正常导航最大速度约 `0.9998 m/s`，消息契约检查无失败；
  - 从干净安装运行 `empty` 正式闭环：约 6.61 s 无碰撞到达目标，最终目标距离约 `0.183 m`，收到 330 个命令样本，稳定频率约 `50.000 Hz` 和 58 组角域可视化；命令与可视化话题均只有 `depth_angular_controller` 发布；
  - 运行代码和工具中不再存在 `PC_GVF_SHADOW_MODE`、`depth_angular_shadow`、影子 launch、迁移诊断或双实现探针引用；沙箱中的 Fast DDS UDP 权限警告仍由共享内存回退覆盖，不影响测试结果。
- 遗留问题：阶段 5 已完成。下一阶段应在正式 C++ 路径上逐项实现已记录的安全与 CPU 改进；其中完整深度保守处理、未知空间语义、最终连续运动/制动认证等属于行为变更，必须各自增加测试和验收，不能与迁移清理混合实施。

### 2026-09-07 — 阶段 5.1：删除已被 C++ 替代的 Python 控制器路径

- 目的：在阶段 4 完成公共接口和闭环验收后，移除已不再承担运行职责的 Python 控制器、回退入口及其专用测试，同时保留仍用于行为验收的 Python 数值 oracle 和合成场景基础设施。
- 涉及文件：`src/pc_gvf/pc_gvf/depth_angular_controller.py`、`src/pc_gvf/scripts/depth_angular_controller`、`src/pc_gvf/scripts/depth_angular_controller_py`、`src/pc_gvf/test/test_depth_visualization.py`、`src/pc_gvf/test/test_depth_angular_core.py`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`src/pc_gvf/setup.py`、`README.md`、`FOV_GVF_PRINCIPLES_RECORD.md`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 删除 Python ROS 控制器模块、旧公共包装器和阶段 4 使用的显式 Python 回退包装器；CMake 与 setuptools 均不再安装或声明这些入口；
  - demo launch 删除 `controller` 参数、`PythonExpression` 条件和 Python 回退节点，正式 `depth_angular_controller` C++ 节点改为无条件启动；
  - 删除只针对旧 Python 控制器 Marker 发布实现的测试，并从核心测试中移除两个直接构造旧控制器的用例；对应四元数和非法深度编码行为已经由 C++ 单元测试及阶段 4.3 ROS 状态探针覆盖；
  - README 删除临时回退说明，原理记录明确 Python 控制器仅是迁移前行为基准；保留 `depth_angular_core.py`、`depth_angular_demo_sim.py`、黄金数据生成器和场景/验收探针，它们不再构成可选生产控制路线；
  - 暂不删除 `pc_gvf_core`：其独立二维/三维 API 尚未由新的角域 C++ 控制器复现；暂不删除 C++ 影子入口和在线对照工具，统一留到阶段 5.2 核定和清理。
- 验证：
  - 使用全新的 `/tmp/fov_gvf_stage51_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；全量测试汇总 18 项，0 错误、0 失败、0 跳过；
  - 干净安装的 `ros2 pkg executables pc_gvf` 只列出 `depth_angular_controller`、`depth_angular_core`、`depth_angular_demo_sim` 和 `depth_angular_shadow`；安装树中不存在 `depth_angular_controller_py` 或 `depth_angular_controller.py`，排除了旧安装产物掩盖依赖的可能；
  - `ros2 launch ... --show-args` 确认旧 `controller` 选择参数已经消失；仅保留 `rviz`、`shadow` 和 `scenario`；
  - 从干净安装运行 `empty` 正式闭环：6.57 s 内无碰撞到达目标，最终目标距离约 `0.183 m`，收到 328 个命令样本，稳定频率约 `50.000 Hz`，并收到 57 组角域可视化；`/position_cmd` 与 `/pc_gvf/angular_field` 均只有 C++ `depth_angular_controller` 发布；
  - 排除文档、旧构建目录和缓存后全仓搜索，运行代码中不再引用 Python 控制器模块、包装器或 `controller:=python`；沙箱中的 Fast DDS UDP 权限警告仍存在，但共享内存通信完成闭环验收，不影响结果。
- 遗留问题：阶段 5.2 需要删除迁移期 C++ 影子可执行文件和只服务于双实现对照的工具，并把目前同时编译 active/shadow 的源文件整理为正式控制器命名；Python 数值 oracle、合成模拟器以及 `pc_gvf_core` 的去留需按其剩余独立功能分别决定，不能随影子设施机械删除。

### 2026-09-07 — 阶段 4.3：完成 C++ 七场景、频率与安全状态验收

- 目的：在删除 Python 控制器前，对已经取得正式接口的 C++ 路径执行最终运行验收，确认闭环无碰撞结果、控制频率、保护状态、消息契约和两种深度编码均满足冻结基准。
- 涉及文件：`src/pc_gvf/CMakeLists.txt`、`tools/ros_cpp_closed_loop_probe.py`、`tools/ros_cpp_state_probe.py`、`README.md`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 当调用方没有指定构建类型时，将 `pc_gvf` 默认配置为 CMake `Release`；未启用 `fast-math`，16 组 Python 数值黄金数据仍作为语义约束；该设置使完整 C++ 节点在含调和求解和 Marker 生成时满足 20 ms 控制周期；
  - 新增闭环验收探针，订阅正式里程计、命令、状态和角域 Marker，逐段使用场景几何检查带 `0.25 m` 机体半径的碰撞，记录最小机体表面净空、目标距离、稳定停车、发布频率和实际发布者；
  - 新增安全状态探针，以定时输入序列覆盖无里程计、无/零/过期意图、无/过期深度、正常导航、NaN 里程计、零范数四元数和到达目标，并分别使用 `32FC1` 米制及 `16UC1` 毫米制深度；
  - 安全状态探针同时检查保护状态持续零速度、正常导航存在非零速度，以及 `PositionCommand` 的坐标系、READY 标志和 Python 原先保留为零的加速度、jerk、yaw rate、增益和轨迹 ID 字段；
  - README 明确默认运行入口已经是 C++，并记录 `controller:=python` 临时回退方式。
- 验证：
  - 使用全新的 `/tmp/fov_gvf_stage43_clean_{build,install,log}` 前缀以默认 Release 配置从零构建四个 ROS 2 包成功；全量测试汇总 22 项，0 错误、0 失败、0 跳过，五个 C++ 黄金数据检查在 Release 下继续通过；
  - 七场景正式 ROS 闭环全部通过且 `/position_cmd`、`/pc_gvf/angular_field` 均只有 C++ 控制器发布：`empty`、`single_pillar`、`offset_box`、`center_sphere`、`diagonal_gap` 无碰撞到达目标，最终目标距离分别约为 `0.183/0.183/0.167/0.169/0.179 m`；
  - `overhead_bar` 与 `narrow_gate` 未发生机体碰撞并稳定降为零速度，分别在约 `(2.533, 0.052, 1.2)` 与 `(2.574, -2.319, 1.2)` 停车；七场景测得命令频率范围为 `50.001–50.007 Hz`；
  - 七场景最小机体表面净空均非负；障碍场景约为 `0.226/0.195/0.246/0.217/0.030/0.193 m`（依次对应立柱、偏置箱、球体、横杆、斜向间隙、窄门），验收探针未检测到膨胀几何相交；
  - 状态注入依次观测到 `WAITING_ODOMETRY → STALE_INTENT → ZERO_INTENT → WAITING_DEPTH → NAVIGATING → STALE_DEPTH → NAVIGATING → STALE_INTENT → INVALID_ODOMETRY → INVALID_GUIDANCE → GOAL_REACHED`；共收到 290 个命令样本，各保护状态均至少连续收到 3 个零速度命令，正常导航最大命令速度约 `0.9998 m/s`，消息字段检查全部通过；
  - 沙箱内 Fast DDS 仍报告 UDP socket 权限警告，但共享内存完成全部闭环和状态测试，不影响结果。
- 遗留问题：阶段 4 已通过，下一步可删除被替代的 Python 算法/控制器及临时回退入口；Python 合成场景、黄金数据生成器和测试仍是当前验收基础，暂不应一并删除；独立 `pc_gvf_core` 仍提供新 C++ 角域控制器尚未复现的二维/三维 API，不能仅因默认链路已切换就删除；`diagonal_gap` 的最小机体表面净空仅约 `0.030 m`，虽未碰撞且与迁移行为一致，但保守安全余量改进仍属于阶段 6。

### 2026-09-07 — 阶段 4.2：将公共控制器入口切换为 C++ 并保留 Python 回退

- 目的：在不删除 Python 行为基准的前提下，让已完成在线等价验收的 C++ 节点取得正式控制器入口和 ROS 公共输出，为下一阶段七场景闭环验收建立真实运行路径。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/scripts/depth_angular_controller_py`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 将同一 C++ ROS 节点源码通过编译定义生成 `depth_angular_controller` 和 `depth_angular_shadow` 两个 ELF 可执行文件；正式构建保持节点名 `depth_angular_controller`，声明既有 `cmd_topic` 参数并发布 `/position_cmd`、私有 `~/status`/`~/diagnostics` 和 `/pc_gvf/angular_field`，影子构建继续固定使用私有隔离输出；
  - 停止把原 Python 包装器安装为公共入口，新增 `depth_angular_controller_py` 显式回退入口；Python 模块和算法本体暂时保留，尚未删除；
  - demo launch 新增取值限定为 `cpp`/`python` 的 `controller` 参数，默认 `cpp`；`controller:=python` 会以原节点名启动回退入口，因此既有私有状态话题和外部 launch 调用仍兼容；
  - 扩展在线探针的 active-C++ 模式：将 Python 回退节点重命名并隔离其命令/可视化，把 C++ 接到正式话题，同时继续逐帧调用 Python oracle 校验诊断并检查命令、状态、Marker 和发布者归属；
  - 将控制定时器由墙钟改为节点 ROS 时钟，保持 Python `create_timer()` 在 `use_sim_time` 下的语义；将无效里程计位置的 `NaN/+Inf/-Inf` 替换值对齐 NumPy `nan_to_num` 的 `0/最大有限值/最小有限值`。
- 验证：
  - 增量安装后 `ros2 pkg executables pc_gvf` 同时列出 C++ 公共入口、Python 回退入口和 C++ 影子入口；`file` 确认 `depth_angular_controller` 为 x86-64 ELF，launch 参数检查确认 `controller` 默认 `cpp` 且只接受 `cpp`/`python`；
  - 使用全新的 `/tmp/fov_gvf_stage42_final_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；全量测试汇总 22 项，0 错误、0 失败、0 跳过；
  - 基于全新安装前缀运行正式接口在线对照，收到 134 组连续 C++ 诊断且全部匹配 Python oracle；双方状态均为 `NAVIGATING`，稳定命令误差和 7 类 Marker 最大几何误差均为 `0.0`；
  - ROS 图确认正式 `/position_cmd` 和 `/pc_gvf/angular_field` 均只有 C++ `depth_angular_controller` 发布，Python oracle 只发布到隔离话题；
  - 分别短时运行默认 `controller:=cpp` 和回退 `controller:=python` 的 `empty` 场景 launch，两条路径均进入 `NAVIGATING`；C++ 日志明确显示命令发布在 `/position_cmd`，Python 回退进程按预期由 `depth_angular_controller_py` 启动。
- 遗留问题：尚未完成 C++ 正式路径的七场景闭环结果、约 50 Hz 命令频率、深度/意图超时、无效里程计、零意图、到达目标和安全停止验收；Python 回退和原 Python 源码在这些检查全部通过前继续保留；短时 `timeout` 终止 Python 演示/回退进程时仍会出现既有 `KeyboardInterrupt` 退出栈，不属于运行期导航失败。

### 2026-09-07 — 阶段 4.1：移植并在线验收 C++ 角域可视化

- 目的：在切换正式控制权之前补齐 C++ 控制器的最后一个外部功能面——角域 `MarkerArray` 可视化，并继续以 Python 输出作为在线行为基准。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/package.xml`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - C++ 影子节点新增 `field_radius` 参数和瞬态本地 QoS 的私有 `~/angular_field` 发布者，保持正式 `/pc_gvf/angular_field` 仍由 Python 控制器拥有；
  - 移植 Python 可视化中的势场梯度填充、三像素采样、制动速度着色、时间脉冲透明度、箭杆/箭头、相机视锥、参考/目标/命令射线、禁行像素和深度障碍点投影，输出同样的 7 类 Marker；
  - 扩展 ROS 对照探针，将 Python `/pc_gvf/angular_field` 重映射到隔离测试话题，并订阅 C++ 私有可视化；检查命名空间集合、ID、类型、动作、坐标系、点/颜色数量、全部三维点和 RGB，同时检查两个可视化话题的发布者归属；
  - 首次在线对比发现 C++ `project` lambda 以 `auto` 返回 Eigen 延迟表达式，引用了已销毁的临时射线，表现为可视化 Y/Z 坐标接近零；改为显式返回 `Eigen::Vector3d` 后强制求值，几何恢复并与 Python 完全一致。
- 验证：
  - 增量编译、链接和安装成功；第一次对照准确捕获 Eigen 生命周期错误，修复后的增量在线对照收到 128 组诊断，命令误差和 Marker 最大几何误差均为 `0.0`；
  - 使用全新的 `/tmp/fov_gvf_stage41_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；全量测试汇总 22 项，0 错误、0 失败、0 跳过；
  - 基于全新安装前缀再次在线对照，收到 132 组连续诊断，Python 与 C++ 状态均为 `NAVIGATING`，稳定命令误差 `0.0`，7 类 Marker 最大几何误差 `0.0`；命令和可视化的四个隔离话题均只有预期节点发布，正式 `/position_cmd` 无 C++ 发布者；
  - 沙箱内 Fast DDS 仍报告 UDP socket 权限警告，但共享内存通信完成全部在线检查，不影响验收结果。
- 遗留问题：本阶段只补齐并验收可视化，正式 `depth_angular_controller` 入口、`/position_cmd`、`~/status` 和 `/pc_gvf/angular_field` 尚未交给 C++；下一步应建立可回退的入口切换，随后完成状态保护、频率和七场景闭环验收，Python 控制器此时仍不得删除。

### 2026-09-07 — 阶段 3：接入非执行 C++ ROS 影子控制器

- 目的：在 Python 继续独占正式控制输出的前提下，把已验收的 C++ 导引核心接入真实 ROS 2 消息和控制状态机，并用同一组不可变传感器输入完成在线 Python—C++ 对照。
- 涉及文件：`src/pc_gvf/src/depth_angular_shadow_node.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/package.xml`、`src/pc_gvf/launch/depth_angular_demo.launch.py`、`tools/ros_shadow_parity_probe.py`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ `depth_angular_shadow` ROS 2 节点，复现 Python 控制器的参数约束、里程计/相机/深度/意图订阅、`32FC1` 与 `16UC1` 深度解码、最近邻角域缩放、相机内参缩放、数据时效检查、参考重锚、四元数变换、历史像素更新、垂向限幅、加速度限幅、状态字符串和 `PositionCommand` 字段；
  - 影子节点不声明正式 `cmd_topic` 参数，命令、状态和 JSON 诊断固定发布到私有 `/depth_angular_shadow/command`、`/status`、`/diagnostics`；诊断包含序号、深度版本、场有效性、四个关键像素、原始/限幅速度、自由距离摘要、禁行像素数、有限势值数量/总和和求解耗时；
  - demo launch 新增默认关闭的 `shadow:=false` 参数；设为 `true` 时同时启动影子节点并共享控制器输入参数，Python 节点仍是 `/position_cmd` 的唯一控制方；
  - 新增 ROS 对照探针，连续发布固定里程计、相机内参、中央近障深度和前进意图，逐帧以 Python `compute_guidance()` 重算 C++ 诊断，检查诊断序号、场状态、像素、原始命令和数值摘要，并比较两节点的最终状态、稳定速度及实际发布者归属。
- 验证：
  - C++ 影子节点在阶段 2.5 临时构建目录增量编译、链接和安装成功；launch Python 语法检查通过，`ros2 launch ... --show-args` 确认 `shadow` 默认值为 `false`；
  - 使用全新的 `/tmp/fov_gvf_stage3_clean_{build,install,log}` 前缀从零构建四个 ROS 2 包成功；`colcon test-result` 汇总 22 项测试，0 错误、0 失败、0 跳过，其中 `pc_gvf` 的五个 C++ 数值检查和三个 Python 注册测试均通过；
  - 基于全新安装前缀运行在线对照，收到 141 组连续 C++ 诊断，全部通过 Python oracle 检查；Python 与 C++ 最终状态均为 `NAVIGATING`，稳定速度向量误差为 `0.0`；
  - ROS 图检查确认 `/shadow_test/python_command` 仅由 `depth_angular_controller` 发布，`/depth_angular_shadow/command` 仅由 `depth_angular_shadow` 发布，正式 `/position_cmd` 在隔离测试中无发布者，因此影子节点没有取得执行权；沙箱禁止 UDP socket 时 Fast DDS 打印传输警告，但共享内存通信完成全部对照，未影响结果。
- 遗留问题：C++ 尚未接管原 `depth_angular_controller` 可执行入口、`/position_cmd`、角域 Marker 可视化和正式闭环；下一阶段需先补齐可视化与公共接口，再执行七场景、发布频率、数据超时和安全停止验收，验收前不得删除 Python 控制器或旧 C++ 独立功能。

### 2026-09-07 — 阶段 2.5：组合并验收完整 C++ 导引核心

- 目的：将阶段 2.1 至 2.4 已独立验收的 C++ 子模块组合为单一导引调用，从原始深度和机器人状态生成完整诊断结果及最终世界系速度，完成 ROS 无关核心的 Python—C++ 等价迁移闭环。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/guidance_composition_check.cpp`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ `AngularSolution` 和 `GuidanceResult`，保存深度、方向自由距离、规划掩码、调和势、参考/源/目标/命令像素、场有效性、返回目标和世界系速度命令；墙钟耗时继续不进入确定性结果；
  - 新增 C++ `computeGuidance()`，接收不可变深度图、相机、配置、位置、速度、目标、历史像素、相机姿态及可选参考线，串联深度反投影、碰撞锥、参考方向投影、安全目标、调和场、方向选择、参考修正、制动和滚动限速；
  - 保持 Python 的相机后方目标边界回退、FOV 一像素禁行边界、无安全目标回退、源目标角距小于 `0.4°` 的短路分支、无效调和场回退、参考修正重投影和最终速度生成顺序；
  - 新增 `guidance_composition_check`，直接从 16 组黄金输入调用完整 C++ 核心，对深度及 NaN 布局、逐像素自由距离/掩码/势场、四个诊断像素、场有效性、返回目标和最终三维速度执行端到端比较。
- 验证：
  - 阶段 2.4 构建目录增量编译成功，`guidance_composition_check` 通过全部 16 组完整 Python 基准，全套 CTest 8/8 通过；
  - 使用全新的 `/tmp/fov_gvf_stage25_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 8/8 通过：五个 C++ 检查和三个 Python 测试注册项全部通过；
  - 完整比较遵循迁移计划容差：掩码和有效性精确一致，自由距离为 `atol=1e-9`、`rtol=1e-9`，势场及命令为 `atol=1e-6`、`rtol=1e-6`，NaN 布局精确一致；
  - 本阶段未改变黄金数据，组合 SHA-256 保持 `ffe6a2d034b0311b00045e7aff2d7d244769da307b30968a806333259bf3ed95`，Python 算法源码 SHA-256 保持 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`。
- 遗留问题：正式 ROS 控制器仍为 Python，C++ 核心尚未订阅真实 ROS 消息或发布影子诊断；下一阶段需要建立非执行的 C++ ROS 影子节点，在相同传感器快照上比较两条链路，并确保仍只有 Python 控制器拥有命令输出权；既有安全语义缺口和 CPU 优化仍留到切换后阶段处理。

### 2026-09-07 — 阶段 2.4：移植路径、参考修正、制动与滚动限速

- 目的：完成 `compute_guidance()` 组合前的最后一组 ROS 无关子模块移植，将调和势转换为受限方向，并复现参考线收敛、制动和前向滚动速度认证行为。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/motion_safety_check.cpp`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ 双线性图像采样，保持越界或四点中存在非有限值时返回调用方默认值的语义；
  - 新增离散调和路径，保持源点裁剪/偶数舍入、8 邻域扫描顺序、已访问惩罚、目标距离微小破同值项和最多 500 步规则；
  - 新增相机射线角距离、像素空间角速率限制和最终命令方向选择，保持“先回到安全源点，否则沿调和路径前看两步”的分支；
  - 新增状态锚定参考线方向修正，保持横向误差、净空平滑门控、最大修正角和三维单位化语义；
  - 新增延迟加制动距离速度上限、加速度/一阶速度响应滚动、点云扫掠球碰撞、相机 FOV 检查和固定 10 次二分速度限制；
  - 黄金数据新增路径、历史点到安全源点角距、限角目标、直接选择命令、参考修正前命令、观测净空、参考修正方向、命令净空、制动速度、全速滚动判定和最终限速结果；
  - 新增 `motion_safety_check`，逐场景核对这些中间量，并补充双线性默认值、零/大净空制动、微小速度、参考门控、空点云和初始扫掠碰撞分支测试。
- 验证：
  - 生成器与 Python 基准回归通过 `py_compile`，Python 黄金数据回归 1 项通过；
  - 16 组黄金数据连续两次强制生成的组合 SHA-256 均为 `ffe6a2d034b0311b00045e7aff2d7d244769da307b30968a806333259bf3ed95`，算法源码 SHA-256 仍为 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`；
  - 在阶段 2.3 构建目录增量编译成功，新增 `motion_safety_check` 通过 16 组 Python 基准，全套 CTest 7/7 通过；
  - 使用全新的 `/tmp/fov_gvf_stage24_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 7/7 通过：四个 C++ 检查和三个 Python 测试注册项全部通过；
  - 二分限速的三个代表性分支均与 Python 一致：`center_sphere_near` 为 `0.49108990452468421 m/s`，`reference_line_offset` 为 `1.170703125 m/s`，`single_kept_near_pixel` 为 `0.36780645666149114 m/s`。
- 遗留问题：阶段 2.5 尚未把全部 C++ 子模块组合成单一 `computeGuidance()` 并直接比较完整诊断结果；正式运行入口仍为 Python；当前为等价迁移而保留“空障碍点立即判定滚动安全”和固定 10 次二分等既有语义，动态容器、逐点滚动和路径集合也尚未做部署 CPU 优化。

### 2026-09-07 — 阶段 2.3：移植安全目标、连通域与角域调和场

- 目的：继续以 Python 为行为基准移植 ROS 无关核心，实现安全源点/目标选择、连通域标记和角域调和势求解，同时保持正式 Python 控制器和外部 ROS 接口不变。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/angular_field_check.cpp`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 在 C++ 中建立与 Python `SimConfig` 对齐的配置结构，为本阶段和后续核心组合提供一致默认值；
  - 新增最近自由像素选择和 8 邻域连通域标记，保持 NumPy 行优先首次最小值、历史目标代价和确定性图像左偏置语义；
  - 新增安全目标选择，保持参考像素裁剪/偶数舍入、同连通域限制、欧氏净空奖励和目标滞回语义；净空距离当前使用精确直接搜索以优先锁定 SciPy 行为，尚未做 CPU 专项优化；
  - 新增源/目标圆盘掩码和 C++ 共轭梯度调和求解，保持四邻域 Laplace 离散、障碍物及 FOV 边界零法向通量、源值 1、目标值 0、重叠/空未知域分支、迭代上限及有效性语义；
  - 黄金数据新增原始可空安全源点/目标、连通域数量/逐像素标签，以及独立调用调和求解器得到的有效性和势场；这避免最终 `compute_guidance()` 的回退分支掩盖子模块错误；
  - 新增 `angular_field_check`，除 16 组黄金数据外，单独覆盖对角 8 邻域连通、左右同距时的固定左偏、源目标重叠、全 Dirichlet、源目标被遮挡和半径一圆盘等分支。
- 验证：
  - 生成器与 Python 黄金数据回归通过 `py_compile`，单独 Python 基准回归 1 项通过；
  - 16 组黄金数据连续两次强制生成的组合 SHA-256 均为 `dfff964b2efcda4706fdde9fde6831674c61f77f28bb9c473e76e6dab03f5338`，算法源码 SHA-256 仍为 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`；组合哈希变化来自新增选择、标签和独立势场字段；
  - 在阶段 2.2 构建目录增量编译成功，CTest 6/6 通过；新增测试对连通域标签和选定像素执行精确比较，对势场使用迁移计划规定的 `atol=1e-6`、`rtol=1e-6` 和精确 NaN 布局比较；
  - 使用全新的 `/tmp/fov_gvf_stage23_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 6/6 通过：三个 C++ 检查和三个 Python 测试注册项全部通过。
- 遗留问题：阶段 2.4 的离散路径、角速度限制、参考线收敛、制动和前向滚动尚未移植；正式运行入口仍为 Python；本阶段的精确直接欧氏距离计算适合当前 `48 × 36` 行为对齐，但需要在完成迁移后结合部署 CPU 指标决定是否替换为线性时间距离变换并重新验证。

### 2026-09-07 — 阶段 2.2：移植深度反投影与碰撞锥自由距离

- 目的：继续按 Python 行为基准分片移植 ROS 无关核心，在不切换控制器入口、不改变正常配置下算法行为的前提下，实现并独立验收 C++ 深度几何层。
- 涉及文件：`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/test/cpp/depth_geometry_check.cpp`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增 C++ `backprojectObstaclePoints()`，保持 Python 的有限值、正深度、最大量程减 `1e-6`、二维采样步长和行优先点序语义；
  - 新增 C++ `collisionConeFreeDistance()`，保持相机射线缓存、传感器最大可知距离、球形膨胀接触距离、严格碰撞锥判定和分块计算语义，并对不匹配的图像尺寸及无效调用参数给出异常；
  - 将 Python 反投影障碍点数量和坐标加入 16 组语言无关黄金数据，使 C++ 测试可分别定位反投影误差与自由距离误差；Python 回归同步验证新增中间量；
  - 新增 `depth_geometry_check`，逐场景检查障碍点数量/坐标和每个像素的自由距离，覆盖全零、全 NaN、步长保留/漏采单像素、不同分块大小及错误输入；
  - 更新迁移计划，将下一检查点推进到阶段 2.3；Python 控制器、三个公开可执行入口和 ROS 接口均未切换。
- 验证：
  - 生成器与 Python 回归测试通过 `py_compile`，单独黄金数据回归 1 项通过；
  - 16 组黄金数据连续两次强制生成的组合 SHA-256 均为 `5e85e861d15d63dcf251d7d8497d03e35e6db0d7b44f5a88cc0ae2e5cc3fed6a`，算法源码 SHA-256 仍为 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`；组合哈希变化仅来自新增确定性障碍点字段；
  - 在阶段 2.1 临时目录完成增量编译，CTest 5 项全部通过；
  - 使用全新的 `/tmp/fov_gvf_stage22_clean_{build,install,log}` 前缀构建 `pc_gvf_msgs`、`pc_gvf_platforms`、`pc_gvf` 三个包成功，随后 CTest 5/5 通过：两个 C++ 检查和三个 Python 测试注册项全部通过。
- 遗留问题：阶段 2.3 及之后的目标选择、连通域、调和场、路径/参考/制动和整体组合尚未移植；正式运行入口仍为 Python；本阶段只验证算法数值和全新构建，未重复 ROS 闭环演示，因为运行实现没有切换。

### 2026-09-07 — 阶段 2.1：建立 C++ 数学、相机与混合构建基础

- 目的：在不切换运行入口、不改变避障算法行为的前提下，在 `pc_gvf` 同一 ROS 包内建立首个可独立验收的 C++ 基础层，并让 C++ 测试能够读取阶段 1 的语言无关黄金数据。
- 涉及文件：`src/pc_gvf/CMakeLists.txt`、`src/pc_gvf/package.xml`、`src/pc_gvf/include/pc_gvf/depth_angular_core.hpp`、`src/pc_gvf/src/depth_angular_core.cpp`、`src/pc_gvf/test/cpp/fixture_reader.hpp`、`src/pc_gvf/test/cpp/camera_geometry_check.cpp`、`src/pc_gvf/scripts/depth_angular_controller`、`src/pc_gvf/scripts/depth_angular_demo_sim`、`src/pc_gvf/scripts/depth_angular_core`、`CPP_MIGRATION_PLAN.md`、`WORK_LOG.md`。
- 修改内容：
  - 将 `pc_gvf` 从纯 `ament_python` 构建改为 `ament_cmake` + `ament_cmake_python` 混合构建，在迁移期间继续安装原 Python 模块、launch 和三个原名可执行入口；
  - 新增独立 C++ 静态库 `pc_gvf_depth_angular_core`，实现与 Python 对齐的向量归一化/限幅、固定相机安装旋转、四元数旋转矩阵、针孔相机内参、像素—射线变换和射线缓存；
  - 新增只用于测试的标准库 `key=value` 黄金数据读取器，能够读取 16 组基准及 `nan/inf` 数组；
  - 新增 `camera_geometry_check`，检查默认和替代相机参数的 Python 数值基准、坐标轴约定、四元数有效性、全部黄金数据形状、每个像素的缓存射线和像素—射线往返；
  - 新增显式 Python 入口包装器。最初直接安装模块文件时，控制器和模拟器因没有模块级 `__main__` 调用而启动后立即正常退出；包装器恢复 setuptools console-script 原有的 `main()` 调用语义。
- 验证：
  - 使用独立的 `/tmp/fov_gvf_stage21_{build,install,log}` 前缀完成 `pc_gvf_msgs`、`pc_gvf_platforms` 和混合 `pc_gvf` 构建；`pc_gvf` 的 `colcon_build.rc` 为 0，C++ 库、头文件、Python 模块、launch 和三个可执行入口均安装成功；
  - CTest 共 4 项全部通过：`camera_geometry_check`、原 Python 核心测试、可视化测试和 16 组黄金行为回归；其中 Python 测试函数合计 6 项通过；
  - `ros2 pkg executables pc_gvf` 仍列出 `depth_angular_controller`、`depth_angular_core` 和 `depth_angular_demo_sim`，`depth_angular_core --help` 正常；
  - 沙箱内首次短时启动因 DDS 无权创建 UDP socket 而无法验证，获批在沙箱外以 `ROS_LOCALHOST_ONLY=1` 重跑；修复入口包装器后，无界面 `single_pillar` 演示依次进入 `WAITING_ODOMETRY`、`WAITING_DEPTH`、`NAVIGATING` 和 `GOAL_REACHED`；8 秒测试结束时 `timeout` 的重复 SIGINT 在三个 Python 进程销毁阶段产生 `KeyboardInterrupt` traceback，导航目标已在中断前到达；
  - 验证过程中另有三次命令环境错误并已纠正：`--log-base` 最初放在 `build` 子命令后、全新前缀只选择目标包而未先构建工作区依赖、一次直接 `ctest` 在加载 ROS 环境前执行；这些失败均未归因于源码测试。
- 遗留问题：阶段 2.2 及之后的深度几何、调和场和运动预测尚未移植；当前正式控制器仍是 Python；混合包暂时保留不再由 ament 构建使用的 `setup.py/setup.cfg`，待 C++ 切换和清理阶段处理；短时启动测试的超时退出不作为优雅关停验收。

### 2026-09-07 — 阶段 1：冻结 Python 行为基准

- 目的：按照“先以 Python 为行为基准、再并行实现 C++、验收后切换和删除旧实现”的迁移路线，建立第一阶段可由 Python 和后续 C++ 共同读取的确定性行为基准。
- 涉及文件：`CPP_MIGRATION_PLAN.md`、`tools/generate_python_baseline.py`、`src/pc_gvf/test/test_python_behavior_baseline.py`、`src/pc_gvf/test/fixtures/python_behavior_v1/README.md`、`src/pc_gvf/test/fixtures/python_behavior_v1/MANIFEST.txt`、`src/pc_gvf/test/fixtures/python_behavior_v1/*.fixture`、`WORK_LOG.md`。
- 修改内容：
  - 新建分阶段迁移计划，定义迁移不变量、运行范围、六个阶段检查点、ROS 公共接口和数值一致性容差；
  - 新增 Python 行为基准生成器，以无需 JSON/NumPy 读取器的 UTF-8 `key=value` 格式保存输入、中间量和最终输出，便于后续 C++ 标准库直接解析；
  - 生成 16 组黄金数据，覆盖七个内置场景，以及全零、全 NaN、保留/漏采单像素、历史方向与侧向速度、相机偏航、替代内参、参考直线偏移和相机后方目标；
  - 冻结深度、方向自由距离、禁行掩码、调和势、源/目标/命令像素、场有效性和最终核心速度；墙钟耗时因不可重复而未写入；
  - 新增自动回归测试，确认当前 Python 核心仍与全部黄金数据一致；清单记录算法源码 SHA-256 `e4aa4f7d37633c2a13ddfe946fc4ae0d6c08e75884b19adad9f669bcc50487e5`。
- 验证：
  - `/usr/bin/python3 -m py_compile` 检查生成器和回归测试通过；
  - 黄金数据生成成功，共 16 组、约 876 KiB；连续两次强制生成的组合 SHA-256 均为 `b9275a9ebdbc7f416a2864b1232d3796c31ceb619c2af580fa210002841e2791`；
  - 新增黄金数据回归测试：1 项通过；
  - `src/pc_gvf/test` 全部测试：6 项通过；首次全包测试因测试命令覆盖 ROS 注入的 `PYTHONPATH` 而在收集阶段缺少 `rclpy`，改为在原 ROS 路径前追加源码目录后通过，属于验证环境命令问题而非代码失败。
- 遗留问题：当前基准有意保留 Python 已知安全语义缺口，只用于迁移等价性；尚未实现 C++ 解析器和任何 C++ 算法模块；旧 C++ 独立功能是否保留要在删除阶段前明确决定。

### 2026-09-06 — 记录 FOV-GVF 原理、论文方法与 CPU 路线核对结果

- 目的：完整阅读用户提供的原理分析 Markdown 和四张论文方法截图，并把附件主张、项目实际实现、二者对应关系及后续判断边界形成可追溯记录。
- 涉及文件：`FOV_GVF_PRINCIPLES_RECORD.md`、`WORK_LOG.md`。
- 修改内容：
  - 新建原理核对记录，整理截图公式（1）至（22）中的粗—细调和场、状态锚定局部参考、横向误差反馈、场刷新和扰动最终界；
  - 区分默认 ROS 2 Python 角度域运行链路、C++ 二维 ESDF 流函数链路和 C++ 三维深度栅格势流链路；
  - 对照源码记录附件 CPU 方案中完整深度、未知语义、邻接一致性、最终输出认证、编译内核和整链路计时的已实现与未实现状态；
  - 记录附件文件哈希和核对范围，并明确附件外部实验材料不在本仓库，相关性能结论尚未在本机复现。
- 验证：完整读取 240 行附件 Markdown 和四张原始分辨率截图；静态核对项目 README、迁移说明、启动配置、Python 核心/控制器/测试、C++ 深度投影/求解/指导策略及回归测试；执行 Markdown 差异与空白检查。未运行算法性能或安全测试。
- 遗留问题：论文截图缺少题名、作者和完整上下文，暂不能确认论文身份；附件引用的外部实验脚本和结果不在当前仓库；原理记录列出的算法语义缺口均尚未修复。

### 2026-09-06 — 建立独立修改分支和工作日志

- 目的：保留上游 `dev` 基线，为后续定制修改建立可追溯记录。
- 涉及文件：`AGENTS.md`、`WORK_LOG.md`。
- 修改内容：
  - 从提交 `804f26e` 创建并切换到 `feature/local-modifications`；
  - 新建本工作日志并整理项目已有功能、实测结果、已知限制和记录规范；
  - 在 `AGENTS.md` 中增加“每次修改必须同步更新工作日志”的项目规则。
- 验证：确认当前分支为 `feature/local-modifications`；检查 Markdown 章节、表格和代码块结构。
- 遗留问题：`build_humble/`、`install_humble/` 和 `log_humble/` 为未跟踪构建产物，尚未调整忽略规则。

### 2026-09-10 — RViz 加入雷达点云与 EGO 膨胀占据地图

- 目的：在 USER-EGO 自动导航运行时同时观察传感器当前看到的三维障碍点，以及 EGO Cloud 场景真值占据栅格经机体安全半径膨胀后的局部地图，并保证新增可视化不改变深度直达控制路线。
- 涉及文件：`src/pc_gvf_platforms/pc_gvf_platforms/ego_map_visualizer.py`、`src/pc_gvf_platforms/setup.py`、`src/pc_gvf_platforms/package.xml`、`src/pc_gvf_platforms/test/test_ego_map_visualizer.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`scripts/run_isaac_fov_gvf_navigation.sh`、`USER_EGO_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：
  - 新增只读 `ego_map_visualizer` 节点，将 `32FC1/16UC1` 深度图以相机内参反投影为 X 前、Y 左、Z 上的 `radar` 坐标点云，并依据里程计持续发布 `world -> radar` 变换；点云发布到 `/pc_gvf/radar_points`，使用 Best Effort/Volatile QoS。
  - 读取与 Isaac ESDF 保护共用的 `400×300×50` EGO 场景 `occupancy.bin`，按导航空间 `0.4 m` 分辨率和机体半径加安全余量 `0.45 m` 做球形体素膨胀；发布以无人机为中心、水平半径 `25 m`、垂直半径 `10 m`、最多 60000 点的 `/pc_gvf/ego_inflated_occupancy`，使用 Reliable/Transient Local QoS。
  - RViz 默认新增青色实时雷达点与半透明红色膨胀体素两层；启动脚本统一导出占据文件路径，launch 仅在 RViz 启用时启动该节点。两个输出不订阅也不发布控制话题，不进入 C++ 控制器或 Isaac 碰撞判断。
- 验证：Python、launch 和 Shell 静态检查通过；新增反投影轴向、球形膨胀/裁剪、四元数与 PointCloud2 布局测试通过，`pc_gvf_platforms` 全包 9 项测试全部通过；ROS 2 三包重新构建成功。第一次短时联调因沙箱禁止 DDS/本地套接字而无法探测话题，随后在允许 ROS 通信的同等环境复验：控制器进入 `NAVIGATING`，两路 PointCloud2 均只有 1 个预期发布者且各有 RViz 订阅者，QoS 完全匹配；采样得到实时雷达云 4685 点、首帧局部膨胀地图 33649 点，RViz、控制器与可视化节点均正常清理退出。
- 遗留问题：当前红色层明确是 EGO Cloud 场景真值栅格的可视化，不是控制器在线重建地图；若以后把真实 EGO-Swarm `sdf_map` 接入本项目，应改订阅其 `/sdf_map/occupancy_inflate` 并关闭本节点的真值地图发布，避免把真值与估计地图混淆。

### 2026-09-10 — 以深度在线历史地图替换 RViz 场景真值地图

- 目的：彻底移除 `occupancy.bin -> /pc_gvf/ego_inflated_occupancy` 的 ROS/RViz 真值可视化链路，改为把无人机实际看见的逐帧深度点转换到 `world` 并持久累计，同时保持青色实时点云、C++ 深度直达导航和 Isaac 内部 ESDF 安全保护不变。
- 涉及文件：删除 `src/pc_gvf_platforms/pc_gvf_platforms/ego_map_visualizer.py`、`src/pc_gvf_platforms/test/test_ego_map_visualizer.py`；新增 `src/pc_gvf_platforms/pc_gvf_platforms/observed_map_visualizer.py`、`src/pc_gvf_platforms/test/test_observed_map_visualizer.py`、`tools/ros_observed_map_probe.py`；修改 `src/pc_gvf_platforms/setup.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`USER_EGO_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`。
- 修改内容：
  - `observed_map_visualizer` 保留 `/pc_gvf/radar_points`；按深度时间戳从 120 个最近里程计样本中选取最近位姿，以 `/sim/odom` 给出的 `T_world_base` 和 USD 已验证的相机外参平移 `(0.22,0,0.02)` 组合 `T_world_radar`，执行 `p_world=R_world_base*p_radar+t_world_base+R_world_base*t_base_radar`。同一组合结果继续广播直接 `world -> radar` TF，最大位姿时间差默认 `0.20 s`。
  - 深度有效范围改为 `0.30–15.0 m`，按 4 像素步长反投影。世界点以 `0.15 m` 体素哈希去重，每张深度帧内每个体素最多计一次，跨帧命中至少 2 次才进入永久 `occupied_voxels`；确认体素不随视野移动或后续未观测而删除。
  - `/pc_gvf/observed_map` 使用 `world` frame、Reliable/Transient Local QoS 和 2 Hz ROS 仿真时间定时发布。内部全局地图不裁剪；发布端以 `0.30 m` 做不影响内部数据的全局二次体素化，必要时继续自适应增大显示分辨率以守住 120000 点上限，避免按动态步长抽样造成地图闪烁。
  - RViz 删除红色 `EGO inflated occupancy`，新增紫色 `Observed obstacle map`：RGB `140;64;217`、Alpha `0.45`、`Points`、2 像素；青色实时雷达云、无人机、轨迹、原始/平滑速度箭头均保留。
  - Launch 不再向可视化节点传递占据文件、真值原点、真值缩放、局部裁剪或 `0.45 m` 膨胀参数；旧可执行入口和增量安装前缀中的三个可再生成残留文件已删除。`scripts/run_isaac_fov_gvf_navigation.sh` 中的 `FOV_GVF_ESDF_OCCUPANCY` 仍保留，且只传给 Isaac 脚本用于独立碰撞保护。
- 验证：
  - 最终在线地图纯函数测试 7 项通过，覆盖轴向反投影、外参/位姿组合、单帧去重、两帧确认、静止重复 100 帧不增长、移动后同一世界体素不重复、历史保留、新区域增长、发布上限和二次体素化；Flake8、Python/launch 编译和 Shell 语法检查通过；ROS 2 三包构建成功。
  - 有效运行探针采集 25 张地图消息：无人机移动 `20.71 m`，首张非空地图 `18330` 点、末张 `69453` 点，宽度单调不减，首张所有体素在末张的缺失数为 `0`；`map_frames=[world]`、`radar_frames=[radar]`，ROS 消息时间计算频率 `2.034 Hz`，雷达云均值 `3809.2` 点/帧。另一次坐标对齐采样给出精确 `2.0 Hz`，剔除独立地面 Mesh 后 36413 个观测点在 EGO Cloud 真值 1 体素邻域内的匹配率为 `100%`，未发现整体偏移。第一次集合探针在 Isaac 完成启动前即超时得到 0 样本，修正为等待 `NAVIGATING` 后重跑才得到上述有效结果。
  - ROS 图实测 `/pc_gvf/observed_map` 与 `/pc_gvf/radar_points` 各有且仅有一个 `observed_map_visualizer` 发布者并与 RViz QoS 配对；旧 `/pc_gvf/ego_inflated_occupancy` 返回 `Unknown topic`；`/position_cmd` 仍只有 `depth_angular_controller` 一个发布者。
  - 最终全程 Isaac+ROS+RViz 闭环在 `70.87 s` 到达 `(64.233,12.129,5.087)` 并返回 0，无碰撞退出；内部确认地图在最后一次周期日志为 247755 个 `0.15 m` 体素，最终 RViz 发布 94528 个 `0.30 m` 点，141 个发布样本点数全程单调增长且没有下降，未触发 120000 点硬上限。
  - `pc_gvf_platforms` 最终全包 13 项测试全部通过（含在线地图新增 7 项）；`pc_gvf` 五项 C++ 核心测试全部通过。`pc_gvf` 两项旧 Python 测试仍在收集阶段因系统 NumPy 2.2.6 与旧 SciPy 二进制 ABI 不兼容失败，和此前记录一致，不是本次变更导致。
- 遗留问题：当前候选体素只在确认后移出计数表，单次噪声候选尚未设置时间淘汰；本次最长运行候选约 4.25 万，未影响闭环。若未来进行数十分钟以上连续探索，应增加只针对未确认候选的 TTL/容量策略，不能删除已确认历史障碍。

### 2026-09-10 — 增加逐次运行的控制链性能指标与持久记录

- 目的：在每次 USER-EGO 运行期间和结束时显示避障计算、控制帧率、相邻控制延迟及控制发布到无人机命令接口的延迟，并把结果按运行 ID 持久记录，作为后续避障算法优化的量化基线。
- 涉及文件：`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf_platforms/pc_gvf_platforms/command_bridge.py`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、`performance/PERFORMANCE_METRICS.md`、`DONG.md`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：C++ 控制器使用 steady clock 统计 `computeGuidance()`、完整回调到发布、DDS publish、控制间隔，并使用 ROS 仿真时间统计深度帧年龄和 50 Hz 命令周期；命令桥统计 `/position_cmd` 时间戳到接收、Twist 转换与 `/sim/cmd_vel` 发布；Isaac 统计实际命令采样帧数、仿真/墙钟 FPS 和命令变化间隔。控制器与桥每 5 秒输出终端摘要，三段在退出时输出最终摘要并追加 Markdown。启动脚本自动创建运行 ID，也允许使用 `FOV_GVF_RUN_ID` 和 `FOV_GVF_PERFORMANCE_LOG` 覆盖。
- 验证：ROS 2 三包构建成功；五项 C++ 核心测试全部通过；Python/launch `py_compile` 与 Bash `bash -n` 通过；加载 Humble 和本项目 install 后，`test_platform_math.py` 5 项通过。第一次 C++ 回归因恢复源码保留旧时间戳而错误复用了此前已回滚实验的 `/tmp` 对象，强制重新编译后 5/5 通过；第一次沙箱内联调因 DDS 套接字权限失败，记录为 0 帧失败样本并保留在性能文档。获准在具备 ROS 通信权限的环境运行两次短时联调，最终修正口径的 `metrics_smoke_corrected_20260910` 在 5.017 s 仿真时间内得到控制命令 251 帧、ROS FPS 50.000 Hz、避障计算 mean/P95/max `1.025/1.406/3.181 ms`、深度到命令 mean/P95/max `16.801/16.667/33.333 ms`、控制器时间戳到桥接接收 mean/P95/max `0.066/0/16.667 ms`；运行按预设 5 s 超时结束，不是完整到达验收。
- 影响与边界：未修改 `depth_angular_core.cpp` 的避障数学、路径选择、安全距离、速度参数或控制输出值；性能统计会增加少量采样和退出写盘开销。`ROS2SubscribeTwist` 不带消息时间戳，因此没有伪造桥接发布到 OmniGraph 取用的逐消息精确延迟，改为分别记录桥接发布耗时和 Isaac 60 Hz 实际采样周期。
- 回滚/备份：修改前完整副本为 `/home/starry/isaac-data/备份/user_ego_before_avoidance_20260910_165519`；恢复时可从该副本取回上述文件并删除新增 `performance/`。
- 遗留问题：尚未运行约 70 秒的完整 Cloud 到达测试；后续正常运行会自动形成完整任务性能记录。

### 2026-09-10 — 提高 RViz 连贯性并降低纯显示链路负载

- 目的：解除 RViz 默认约 30 FPS 的显示限制，并减少高频完整路径、角域 Marker 和大规模历史点云重发造成的渲染阻塞，使第三人称跟随更连贯。
- 涉及文件：`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`src/pc_gvf_platforms/pc_gvf_platforms/navigation_visualizer.py`、`src/pc_gvf_platforms/test/test_robot_visualization.py`、`src/pc_gvf/src/depth_angular_controller_node.cpp`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`DONG.md`、`USER_EGO_GUIDE.md`、`WORK_LOG.md`。
- 修改内容：RViz 目标帧率由隐式默认值提高为 60 FPS；无人机/命令 Marker 保持最高 60 Hz，完整路径降为 10 Hz且位移至少 0.10 m 才新增轨迹点，缓存上限从 2000 降为 1000；角域 GVF Marker 降为 15 Hz；雷达/在线地图深度采样步长由 2 增至 4；历史地图显示由 `2 Hz/0.15 m/120000 点` 调整为 `1 Hz/0.25 m/60000 点`；两层 PointCloud2 队列设为 1 并关闭逐点选择，默认关闭与无人机 Mesh 重复的 Odometry 坐标轴。
- 验证：ROS 2 三包构建成功；五项 C++ 避障核心测试全部通过；可视化与在线地图 Python 测试 8 项全部通过；Python/launch 语法通过。使用安装后的 RViz 配置短时启动 8 秒，OpenGL 4.6 和 ThirdPersonFollower/显示配置正常加载，无插件错误；测试按 `timeout` 预期返回 124。安装目录已复核为 `Frame Rate: 60` 及上述降载参数。完整 `ctest` 另有 2 项 Python 基线测试在收集阶段失败，原因为系统 NumPy 2.2.6 与 Ubuntu SciPy（要求 NumPy <1.25）二进制不兼容，并非断言失败；随后再次定向执行本次相关的 5 项 C++ 和 8 项 Python 测试，全部通过。未执行完整 Isaac+RViz 到达测试，因此不宣称实际显示始终达到 60 FPS。
- 影响与边界：只降低 RViz 和辅助可视化的数据量/发布频率；`depth_angular_core.cpp`、控制器 `48×36` 深度规划输入、50 Hz `/position_cmd`、安全距离、速度和 Isaac ESDF 均未改变。历史地图内部仍以 0.08 m 累计，0.25 m只影响发布显示。
- 回滚/备份：修改前完整快照为 `/home/starry/isaac-data/备份/user_ego_before_rviz_fps_20260910_204300`，源/备份文件清单哈希均为 `e76c88f0ce0600ead2d36e1059259317742af3bbe9bc5d4d7557106c60b221e7`。
- 遗留问题：实际帧率仍受 GPU、窗口大小和同时显示项目影响；若用户机器上仍不够流畅，可在 RViz 中临时关闭 `Forward depth` 或 `FOV, depth hits, and angular GVF` 进一步降载。

### 2026-09-10 — 固化用户调整后的 RViz 默认配置

- 目的：保存用户在 RViz 界面中修改后的显示、视角与窗口布局，并使项目脚本和直接启动 RViz2 时都默认使用该配置。
- 涉及文件：`src/pc_gvf_platforms/config/isaac_cloud_navigation.rviz`、`/home/starry/.rviz2/default.rviz`、`WORK_LOG.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：确认 RViz 已在退出时把最新界面状态写入临时安装目录，以该版本覆盖项目源码配置和用户级 `default.rviz`；保存白色背景、关闭地面网格、第三人称距离约 14.65、历史点云 20 px 和 1920×2032 窗口布局，同时保留 60 FPS、点云队列 1、禁用点选择等性能优化。项目 launch 继续通过 `-d` 默认加载该项目配置。
- 验证：源码、临时安装目录和用户级默认文件 SHA-256 均为 `a01b082c747da498046e578bcb1085334f3ecc469b00f4dbe8fd4e1fafc037a2`；重新构建 3 个 ROS 2 包成功。未运行 Isaac 仿真。
- 影响与边界：只改变并固化 RViz 显示配置，未修改避障算法、控制链、传感器参数或性能统计。
- 回滚/备份：旧项目 RViz 配置保存于 `/home/starry/isaac-data/备份/isaac_cloud_navigation_before_user_default_20260910_2105.rviz`；完整项目前置备份为 `/home/starry/isaac-data/备份/user_ego_before_rviz_fps_20260910_204300`。
- 遗留问题：无。

### 2026-09-11 — Cloud 场景由 4 倍缩小为 2 倍并调整起终点

- 目的：把 EGO-Swarm Cloud 物理场景从原始地图的 4 倍缩小为 2 倍，并让无人机任务坐标保持相同的相对位置。
- 涉及文件：`scripts/prepare_ego_swarm_cloud_navigation.sh`、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`、`DONG.md`、`USER_EGO_GUIDE.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md`、`WORK_LOG.md`、`HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/ENVIRONMENT_AND_ASSETS.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：用户已先将 `--environment-scale` 从 4.0 改为 2.0，本次保留该修改；起点从 `(-64,-11.2,2.8)` 等比例改为 `(-32,-5.6,1.4)`，终点及 ROS 控制器固定目标从 `(64,11.2,4.8)` 改为 `(32,5.6,2.4)`。重新生成导航 USD，障碍空间约为 `80×60×10 m`；元数据驱动的 ESDF 分辨率由 0.4 m 变为 0.2 m，相机最大深度由 40 m 变为 20 m。无人机不缩放。
- 验证：Isaac Python 复核 USD 元数据为 `environmentScale=2.0`、起点 `[-32,-5.6,1.4]`、终点 `[32,5.6,2.4]`，障碍 Mesh 边界约 `(-40.1,-30.1,-0.1)` 至 `(39.9,29.9,9.9)`；ROS 2 三包构建成功，安装后的 launch 固定目标正确；Shell/launch 语法通过，5 项 C++ 核心测试和 8 项可视化/地图 Python 测试通过。未运行完整 Isaac 闭环飞行。
- 影响与边界：物理环境、任务起终点、ESDF 物理尺度和相机最大量程改变；避障算法、无人机尺寸、速度、安全参数、RViz 配置和在线观测地图参数不变。
- 回滚/备份：修改前完整快照为 `/home/starry/isaac-data/备份/user_ego_before_2x_map_20260911_184713`。
- 遗留问题：需要完整运行 2 倍场景，确认新起点净空和终点可达性。

### 2026-09-11 — 以当前 USER-EGO 重建 4 倍场景键盘自由控制版

- 目的：完整继承当前 USER-EGO 的连续调和避障、在线观测地图、RViz 降载和性能统计，同时把 USER 恢复为无固定起终点的键盘自由驾驶副本，并修复旧键盘方案依赖事件集合、可能漏松键的问题。
- 涉及文件：整体替换 `/home/starry/isaac-data/user`；重点修改 `scripts/isaac/prepare_uav_navigation_scene.py`、`scripts/isaac/run_fov_gvf_navigation.py`、`scripts/prepare_ego_swarm_cloud_navigation.sh`、`scripts/build_isaac_ros_workspace.sh`、`scripts/run_isaac_fov_gvf_navigation.sh`、其余绝对路径脚本、`src/pc_gvf/launch/isaac_cloud_navigation.launch.py`、`scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd`、`USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md`、`ISAAC_EGO_SWARM_CLOUD_NAVIGATION.md` 及 HISTORY 记录。
- 修改内容：将旧 USER 移入备份后复制当前 USER-EGO，构建目录和锁文件换为 USER 专用名称。环境比例改回 4.0；手动场景不创建 `/World/UAV_Start`、`/World/UAV_Goal`，只存 `spawn=(-64,-11.2,2.8)`。关闭固定前进、固定目标和到点停止；每仿真帧直接轮询物理按键状态并将世界坐标 `W/S/A/D/R/F` 意图以 `TwistStamped` 发布到 `/human_intent`。无运动键时输入为零，最终执行器同帧再次强制零速；ESDF 只阻止危险位移，不退出会话。
- 验证：USD 元数据和 Prim 检查确认 4 倍、无起终点，障碍范围约 `160×120×20 m`；ROS 2 三包构建成功；C++ 核心 5/5、平台/可视化/地图 Python 13/13 通过，静态检查通过。3.017 s 无按键 Isaac+ROS 短测中控制器进入 `ZERO_INTENT`，位置从始至终为 `(-64.000,-11.200,2.800)`。`get_keyboard_value()` 可调用并返回未按下值；自动按键探针因 Isaac 6.0 运行时没有文档所列 `buffer_keyboard_key_event` 方法而失败，故没有把真实按键写成已验证。
- 影响与边界：只改 USER；USER-EGO 仍是 2 倍固定起终点自动版。C++ 避障数学、在线地图原则、RViz 和性能采样保持继承状态。
- 回滚/备份：旧 USER 完整保存于 `/home/starry/isaac-data/备份/user_before_user_ego_replace_20260911_190000`。
- 遗留问题：用户需在 Isaac 主视口人工验收单键、组合键、松键、Space、窗口失焦和碰撞保护后改向继续驾驶。

### 2026-09-12 — 临时切换为无障碍平地控制测试场景

- 目的：排除 Cloud 障碍与避障路径干扰，单独测试 USER 手动版的键盘控制、组合键、停止和升降。
- 涉及文件：新增 `scripts/isaac/create_flat_ground_scene.py`、`scripts/prepare_flat_ground_navigation.sh` 和 `scenes/flat_ground/` 两个 USD；修改 `scripts/run_isaac_fov_gvf_navigation.sh`、`scripts/isaac/run_fov_gvf_navigation.py`、`USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md` 及 HISTORY 状态记录；性能文档自动记录两次探针。
- 修改内容：当前默认场景模式改为 `flat`，场地为 `200×200 m`、`z=0`，出生点 `(0,0,1.5)`。启动脚本支持 `FOV_GVF_SCENE_MODE=flat|cloud`，只自动加载所选场景同目录的占据文件，避免平地误用 Cloud ESDF。运行器允许零空中障碍，同时读取 `static_ground` 顶面并按0.25 m机体半径阻止穿地。原4倍 Cloud场景和 `occupancy.bin` 完整保留。
- 验证：Shell/Python语法检查通过；OpenUSD确认平地环境只有 `/World/Ground`，手动元数据、Robot和相机有效，无起终点。第一次3秒短测错误使用headless，手动键盘接口按设计拒绝启动并得到0帧；随后可见窗口短测成功进入 `ZERO_INTENT`，3.017 s内发布151个命令帧，无按键时位置保持 `(0,0,1.5)`，无碰撞阻断，按预设超时结束。
- 影响与边界：未修改C++避障算法、键位、速度、ROS/RViz或USER-EGO。真实按键事件无法由当前自动探针注入，仍需用户点击Isaac主视口后人工验证。
- 恢复方法：`FOV_GVF_SCENE_MODE=cloud bash scripts/run_isaac_fov_gvf_navigation.sh` 直接运行原4倍Cloud。
- 遗留问题：现场测试 `W/S/A/D/R/F`、组合键、松键、`Space`，以及下降到地面时的保护。

### 2026-09-12 — 摘出不含避障和ROS的 KEYBOARD 独立控制测试

- 目的：把物理键盘读取与无人机直接运动从完整USER控制链中单独摘出，避免ROS、深度避障、ESDF和RViz影响键盘实现测试。
- 涉及文件：新增 `KEYBOARD/run_keyboard_only.sh`、`KEYBOARD/keyboard_only.py`、`KEYBOARD/README.md`；更新 `USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md` 和统一HISTORY记录。
- 修改内容：独立程序每帧使用 `get_keyboard_value()` 读取 `W/S/A/D/R/F`、方向键、Page Up/Down和Space，组合键归一化为默认2 m/s世界坐标速度并直接积分Robot位置；水平移动时机头平滑朝向运动方向，`1/3/V`切换视角。没有ROS导入、节点、话题、FOV-GVF、深度控制、ESDF、RViz或在线地图；只保留 `z>=0.25 m` 防穿地钳制。终端在物理按键集合变化时输出 `[KEY STATE]`。
- 验证：Shell/Python语法通过，Isaac 6.0中所用键枚举全部存在；3秒可见窗口短测成功打开第三人称视图并明确打印全部控制链为off，运行181帧，无按键时位置保持 `(0,0,1.5)`，按预设结束。未发现ROS或避障进程。
- 影响与边界：原USER完整入口及USER-EGO均未改动；物理键盘事件不能由当前自动探针可靠注入，真实单键、组合键、松键和Space仍待用户现场验证。
- 运行：`cd /home/starry/isaac-data/user/Fov-gvf/KEYBOARD && bash run_keyboard_only.sh`。
- 遗留问题：用户现场观察 `[KEY STATE]` 与无人机运动是否一致。

### 2026-09-12 — 修正 KEYBOARD 过曝并增加地面方向参照

- 目的：解决独立键盘测试中无人机和地面大面积发白、难以判断飞行姿态与位移的问题。
- 涉及文件：`KEYBOARD/keyboard_only.py`、`KEYBOARD/README.md`、`WORK_LOG.md`、
  `HISTORY/CHANGELOG.md`。
- 修改内容：视口从会忽略物体颜色层次的 `MinimalRendering` 常量着色模式改为
  `RayTracedLighting`，关闭自动曝光，将太阳光强度从 `800` 降至 `2.5`、环境光设为
  `0.01`。运行时在出生点地面创建无碰撞中心台、红绿坐标十字及四个方向块，颜色约定
  为 `+X` 橙、`-X` 蓝、`+Y` 绿、`-Y` 品红；增加可选 `KEYBOARD_SCREENSHOT` 自动截图
  参数。这些 Prim 固定在世界坐标系，不随无人机移动。
- 验证：Python、Shell 语法和尾随空白检查通过；共进行四轮 4 s 可见窗口截图检查。
  第一轮为 `MinimalRendering + 250`，第二、三轮为 `RayTracedLighting + 250/25`，均仍
  存在明显高光裁切，未作为完成结果；最终太阳光
  `2.5` 的运行完成 241 帧并正常保存 `/tmp/keyboard_observation_final.png`。人工查看确认
  蓝色机身、深色电机、灰色桨叶、橙/绿坐标线和彩色方向块均可区分；全图任一通道
  `>=250` 的像素比例约 `0.066%`，截图 SHA-256 为
  `3ee319f732436feb0b9c883f0c42ec583d350cd9e11b296656b6677a0219adef`。
- 影响与边界：只改变 KEYBOARD 的显示渲染和无碰撞视觉参照；没有修改键位、速度、
  位置积分、最低高度限制、ROS、避障、完整 USER 入口或 USER-EGO。
- 回滚/备份：旧参数在本条记录中保留；恢复时将渲染器改回 `MinimalRendering`、太阳光
  改回 `800`，并删除 `KeyboardReferences` 与截图逻辑。
- 遗留问题：自动测试仍未注入真实物理按键，按键控制手感需用户现场验收。

### 2026-09-12 — KEYBOARD 独立入口改为 LiteRadio Mode 2 手柄控制

- 目的：以 Ubuntu 已识别的 BETAFPV LiteRadio 3 USB joystick 替换独立入口中的键盘
  飞行控制，并采用多旋翼常见 Mode 2 操作语义。
- 涉及文件：`KEYBOARD/keyboard_only.py`、`KEYBOARD/run_joystick_only.sh`、
  `KEYBOARD/run_keyboard_only.sh`、`KEYBOARD/README.md`、`USER_MANUAL_CONTROL_GUIDE.md`、
  `DONG.md`、`WORK_LOG.md`、`HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、
  `HISTORY/CHANGELOG.md`。
- 修改内容：直接使用 Linux `/dev/input/js` 非阻塞接口读取手柄，不依赖 Isaac/GLFW 的
  游戏手柄映射数据库。默认 AETR/HID 通道为轴0 Roll、轴1 Pitch、轴2 Throttle、轴3
  Yaw；Pitch/Roll 产生相对当前机头的前后/左右速度，Throttle 产生垂直速度，Yaw 独立
  积分机头角度，删除原来“移动方向自动带动偏航”的键盘行为。默认水平/垂直速度为
  `2.0/1.5 m/s`，最大偏航角速度 `75 deg/s`，死区 `0.08`。四杆回中保持 `0.5 s` 后
  才解锁，USB断连立即输出零运动；设备、轴、正负号、速度和死区均可由环境变量覆盖。
  新增 `run_joystick_only.sh` 主入口，旧 `run_keyboard_only.sh` 保留为兼容包装。
- 验证：主机只读检测确认 USB `0483:572b` 对应 `/dev/input/js0`，设备名
  `STMicroelectronics BETAFPV Joystick`，7轴、16按钮，内核绝对轴能力为 `0x7f`；Python
  和两个Shell脚本语法检查通过。5 s 可见 Isaac 联调成功直接打开设备并运行301帧；
  当时实测语义通道为 `roll=0.00, pitch=0.00, throttle=+0.64, yaw=0.00`，因油门未回中，
  安全门保持 `enabled=no`，无人机从始至终保持 `(0,0,1.5)`，证明非中位启动不会误动。
  GLFW 同时报“unknown remapping”，但本实现绕过GLFW直接读取js0，因此不影响采样。
- 影响与边界：只替换 `KEYBOARD/` 无ROS独立测试入口的输入与运动语义；完整 USER
  `scripts/run_isaac_fov_gvf_navigation.sh` 仍为键盘+避障，USER-EGO自动版不变。保留
  已验收光照、地面参照和最低高度保护。
- 回滚/备份：未建完整副本；旧键盘实现及参数可由前一条历史记录恢复，兼容Shell入口
  未删除。
- 遗留问题：需要用户把油门置中完成解锁，依次推动四个摇杆现场确认实际轴方向；若
  固件通道顺序不同，按 `KEYBOARD/README.md` 调整 `JOYSTICK_AXIS_*`/`SIGN_*`。

### 2026-09-12 — 独立手柄入口切换为 BEITONG A2P3A 映射

- 目的：用户用北通 A2P3A BFM 接收器替换 LiteRadio 后，使 Mode 2 控制使用新设备的
  实际 Linux joystick 轴布局。
- 涉及文件：`KEYBOARD/keyboard_only.py`、`KEYBOARD/run_joystick_only.sh`、
  `KEYBOARD/README.md`、`USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md`、`WORK_LOG.md`、
  `HISTORY/README.md`、`HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/CHANGELOG.md`。
- 修改内容：默认设备由 `/dev/input/js0` 改为稳定的北通 by-id 路径。Linux轴码表为
  `0=ABS_X, 1=ABS_Y, 2=ABS_Z, 3=ABS_RZ, 4=ABS_GAS, 5=ABS_BRAKE,
  6/7=ABS_HAT0X/Y`，据此将默认 Mode 2 映射改为 `Yaw=0, Throttle=1, Roll=2,
  Pitch=3`；Gas/Brake和方向键不参与飞行控制。正负号、速度、死区与安全门不变。
- 验证：USB只读检测确认 `20bc:511c`、设备名
  `BEITONG BEITONG A2P3A BFM DONGLE`、8轴16按钮。第一次按通用Xbox假设配置
  `Roll=3, Pitch=4` 的5 s联调中，静止Pitch被读成 `+1.00`，回中门保持关闭且无人机
  未移动；随后读取 `JSIOCGAXMAP` 发现轴4实际为Gas，纠正为轴2/3右摇杆。最终5 s
  Isaac联调中四通道均为 `0.00`，0.5 s后正常 `enabled=yes`，运行301帧，位置始终
  `(0,0,1.5)`。Python、Shell语法和KEYBOARD改动文件尾随空白检查通过。
- 影响与边界：只改变KEYBOARD独立入口的默认设备和轴编号；完整USER键盘避障、
  USER-EGO、运动速度、光照、地面参照与断连停车逻辑不变。
- 回滚/备份：未建完整备份；前一条LiteRadio记录保留旧设备路径和轴映射，可按记录
  恢复，所有通道仍支持环境变量覆盖。
- 遗留问题：空闲、中位解锁和无漂移已验证；四个摇杆的实际运动方向仍需用户推动
  手柄现场确认，若单轴方向相反只需调整相应 `JOYSTICK_SIGN_*`。

### 2026-09-12 — 新增 MISSANDKEYBOARD 北通手柄 + FOV-GVF 避障入口

- 目的：把 `KEYBOARD/` 保留为已验证的无ROS手柄参考，在独立
  `MISSANDKEYBOARD/` 入口中将同一套北通 A2P3A Mode 2 输入接入 USER 的完整深度
  避障链。
- 涉及文件：新增 `MISSANDKEYBOARD/beitong_joystick.py`、
  `MISSANDKEYBOARD/run_joystick_avoidance.sh`、`MISSANDKEYBOARD/README.md`；修改
  `scripts/isaac/run_fov_gvf_navigation.py`、`scripts/run_isaac_fov_gvf_navigation.sh`、
  `USER_MANUAL_CONTROL_GUIDE.md`、`DONG.md`、`WORK_LOG.md`、`HISTORY/README.md`、
  `HISTORY/FOV_GVF_FAMILY.md`、`HISTORY/CHANGELOG.md`；联调自动追加
  `performance/PERFORMANCE_METRICS.md`。
- 修改内容：完整运行器新增默认关闭的 `ISAAC_MANUAL_INPUT_MODE=joystick` 模式，原
  键盘默认行为不变。手柄采用实测北通映射 `Yaw=0, Throttle=1, Roll=2, Pitch=3`；
  Pitch/Roll 按当前 yaw 从机体系转换为 world 坐标，Throttle 作为 world Z，一并发布
  `/human_intent`，C++ FOV-GVF 修正后才经 `/position_cmd -> /sim/cmd_vel` 执行。
  Yaw 独立旋转机头和前向相机。保留0.5秒回中解锁、0.08死区、断连全停；末端 dead-man
  单独检查平移输入，确保只转Yaw时不会沿用旧平移命令。组合脚本默认选择4倍Cloud。
- 验证：Python与Shell语法通过。纯Python映射检查最初误把对角输入期望写成未归一化
  数值，断言失败；按程序既定的平面合速度归一化修正期望后通过，覆盖Mode 2符号、
  机体到world旋转、平面归一化、升降和偏航。受限环境无法读取`/dev/input`，启动脚本
  按预期拒绝运行；获得主机设备权限后执行5秒可见Isaac+ROS联调成功：识别北通8轴
  16按钮，`/position_cmd`发布者为1，加载4倍Cloud与0.40 m ESDF，四通道均为0，0.5秒
  后解锁，303个仿真控制帧内位置保持`(-64,-11.2,2.8)`、yaw为0、碰撞阻断为0，
  `MANUAL_TIMEOUT`后两个ROS节点干净退出。该次无人工推杆，控制器处于`ZERO_INTENT`、
  guidance帧为0，故未验证实际运动方向或动态绕障效果。
- 影响与边界：`KEYBOARD/` 未修改，仍是无ROS直控参考；原完整入口默认仍为键盘和平地。
  C++避障数学、launch参数、RViz与USER-EGO均未修改。手柄组合入口只做人工局部避障，
  不增加全局寻路能力。
- 回滚/备份：未建立完整副本；删除 `MISSANDKEYBOARD/`，并移除运行器的 joystick模式
  分支即可恢复。原键盘路径始终保留且默认关闭新模式。
- 遗留问题：用户需现场分别推动四杆确认方向，并在Cloud障碍前验证减速、绕行、回中
  停车和断连停车；未完成这些动作前不把动态避障标为通过。

### 2026-09-17 — 建立 EGO1P0 独立版本分支

- 目的：以 `user/Fov-gvf` 当前“北通手柄 + C++深度角域FOV-GVF避障”实现为主基线，
  建立后续可独立修改的 `EGO1P0` 目录版本，并完整说明现有功能、平台与参数。
- 涉及文件：从 `/home/starry/isaac-data/user/Fov-gvf` 复制当前源码、场景、配置、脚本、
  文档和性能基线到 `/home/starry/isaac-data/EGO1P0`；新增 `EGO1P0_VERSION.md`；只在
  构建、启动、场景准备/生成脚本中把绝对工程路径改为EGO1P0，并将colcon临时目录改为
  `/tmp/fov_gvf_ego1p0_isaac_{build,install,log}`。未复制可再生成的工程内
  `build/install/log`、`__pycache__`和`.pytest_cache`。
- 具体内容：算法源码、ROS launch参数、北通轴映射、USD/occupancy场景、RViz配置、
  ROS话题和默认行为均保持主基线建立时状态。`EGO1P0_VERSION.md`记录人工意图到避障
  再到Isaac的完整数据链，Ubuntu/ROS 2/Isaac/GPU/手柄平台，Isaac、手柄、控制器、
  命令桥、地图、RViz和性能参数，以及构建运行方法与验证边界。
- 验证：`rsync --checksum --dry-run`显示差异仅为新增版本说明和预期的路径适配
  文件；其余文件校验一致。关键Python文件语法检查与全部Shell脚本`bash -n`通过，
  源码脚本中不再残留`/home/starry/isaac-data/user/Fov-gvf`或旧
  `/tmp/fov_gvf_user_isaac_*`执行路径。使用EGO1P0独立临时目录构建3个ROS包成功；
  C++定向测试`camera_geometry_check`、`depth_geometry_check`、`angular_field_check`、
  `motion_safety_check`、`guidance_composition_check`共5/5通过。未运行Isaac动态仿真，
  未重复声明手柄动态绕障通过。
- 影响与边界：主基线`user/Fov-gvf`的算法和运行文件未改；EGO1P0不会自动跟随主基线
  后续变化。两者仍共用ROS域42和`/tmp/fov_gvf_user_navigation.lock`，因此有意禁止并行
  运行，防止同域重复控制器。
- 回滚/备份：EGO1P0本身即2026-09-17目录快照；删除该新目录和独立`/tmp`构建产物
  即可回滚，不影响主基线。
- 遗留问题：需用户现场推动四杆验证方向，并在Cloud障碍前验收动态局部避障、回中
  停车和USB断连停车。

### 2026-09-17 — EGO1P0 主入口默认切换为北通手柄控制

- 目的：按用户要求，使EGO1P0直接运行完整主脚本时采用与
  `user/Fov-gvf/MISSANDKEYBOARD`相同的北通A2P3A Mode 2手柄输入，不再需要进入
  `MISSANDKEYBOARD/`专用目录。
- 涉及文件：`scripts/run_isaac_fov_gvf_navigation.sh`、
  `scripts/isaac/run_fov_gvf_navigation.py`、`USER_MANUAL_CONTROL_GUIDE.md`、
  `EGO1P0_VERSION.md`、`DONG.md`、`performance/PERFORMANCE_METRICS.md`、
  `HISTORY/FOV_GVF_FAMILY.md`和`HISTORY/CHANGELOG.md`。
- 修改内容：主Shell入口与Python运行器的默认`ISAAC_MANUAL_INPUT_MODE`由`keyboard`
  改为`joystick`；主入口在启动ROS/Isaac前检查默认北通by-id设备是否可读，缺失时安全
  退出并提示连接设备。原键盘控制未删除，可通过
  `ISAAC_MANUAL_INPUT_MODE=keyboard`显式启用。手柄映射、速度、死区、0.5秒回中解锁、
  USB断连停车、机体系到world转换、360°水平避障和XY/Z解耦均继续复用既有实现。
- 验证：Shell与Python语法检查通过；用不存在的设备路径确认默认入口在拉起ROS前以
  状态1拒绝，用显式keyboard和不存在的ROS安装目录确认键盘分支绕过手柄检查；
  `test_manual_control_math.py`为4/4通过。主机5秒可见Isaac+ROS零输入联调成功，主入口
  打印`mode=joystick scene_mode=cloud`，识别北通8轴16按钮，控制器报告4个水平视角，
  加载Cloud ESDF，四杆为零并在0.5秒后解锁；303帧内位置保持
  `(-64,-11.2,2.8)`、yaw为0、碰撞阻断为0，随后超时且ROS节点干净退出。截图
  `/tmp/ego1p0_default_joystick.png`确认Cloud障碍场景正常显示。
- 影响与边界：这是默认输入选择和启动前安全检查的变化，没有修改FOV-GVF数学、ROS
  话题或手柄轴定义。零输入联调不能替代人工推杆验收；真实四轴方向、动态绕障、回中
  停车和拔掉USB停车仍需用户现场操作确认。
- 回滚/备份：将两个默认值恢复为`keyboard`并移除主Shell的手柄预检即可回滚；显式
  keyboard兼容路径目前可直接使用，无需回滚代码。

### 2026-09-17 — 修复右摇杆水平零输出并恢复点云输入密度

- 目的：处理“右摇杆方向在RViz可见但无人机不平移”，并恢复为
  `user/Fov-gvf/MISSANDKEYBOARD`的点云观感和传感器基线。
- 涉及文件：四相机场景准备器与两份生成USD、Isaac运行器、深度角域核心/控制节点、
  `horizontal_360_check.cpp`、版本/控制说明、自动运行性能记录，以及HISTORY状态与总账。
- 修改内容：通过状态切换诊断把请求速度、所选视角自由距离、制动/rollout限速和发布
  速度串联输出。实测四路120°相机把机体电机/桨叶投进碰撞锥，所有方向自由距离约
  `-0.33~-0.35 m`，所以安全层正确但错误地把水平速度压为0。四路改为中心间隔90°的
  90°FOV，并把每路从`160×120`恢复到`320×240`；角域相机模型同步为90°/68°。
  精确45°拼接边界在测试中复用运行时内侧像素钳位。手柄心跳增加enabled/connected。
- 验证：RViz源配置与USER逐字节相同，launch地图参数均为`depth_stride=4`、
  `map_resolution=0.08`、`publication_resolution=0.25`、`maximum_publish_points=160000`。
  ROS三包构建成功。第一次测试命令把`--log-base`放错位置而未运行；第二次未设
  `PYTHONNOUSERSITE`，导致2项Python收集受用户NumPy 2.2.6与系统SciPy不兼容影响，
  同时90°精确45°边界旧断言失败；修正测试并隔离用户site-packages后，9/9 CTest、
  汇总16项均通过。正式运行`20260917_143846`中右摇杆请求/制动/rollout均可达
  `2.000 m/s`、实际发布`1.998 m/s`，XY位置由`(-64.00,-11.20)`移动到约
  `(-29.85,-28.66)`；前向历史地图发布点数由约1922增长到52280。
  最终窗口`20260917_144635`已重新启动供继续测试；实时端点检查确认点云发布端与
  RViz订阅端均采用兼容的`BEST_EFFORT/VOLATILE` QoS。
- 影响与边界：未改轴映射、地图体素/抽样、RViz点大小和速度上限。近障碍时的安全
  减速仍保留；USB拔出停车本轮未现场执行。原USER/MISSANDKEYBOARD没有被修改。

### 2026-09-18 — 初始化EGO1P0 Git分支并准备推送

- 目的：把当前已验证的EGO1P0版本纳入用户GitHub仓库，同时不覆盖已有远程`dev`。
- 涉及文件：新增项目本地`.git/`元数据；修改本文件和`DONG.md`两处既有Markdown
  行尾空格。`build/`、`install/`、`log/`、Python缓存和测试缓存继续由`.gitignore`
  排除。
- 修改内容：以本机`Fov-gvf`历史提交`d267feb`为父节点初始化分支`ego1p0`，远程设置为
  `https://github.com/Chenwill1899/Fov-gvf.git`；提交当前源码、配置、场景、说明和性能
  记录共241个受控文件，源码快照提交为`d02b140`（`Add EGO1P0 joystick avoidance
  baseline`）。没有修改远程`dev`。
- 验证：远程分支检查确认仅有`dev`、没有`ego1p0`重名；有效文件最大约17 MB，未超过
  GitHub单文件限制；秘密模式扫描未发现私钥、GitHub令牌或常见API密钥；构建/安装/
  日志与缓存目录均显示为ignored。首次`git push -u origin ego1p0`失败，错误为HTTPS
  无法读取GitHub Username；进一步检查确认本机没有`gh`、SSH私钥、GitHub Token环境
  变量或已配置凭据助手。因此当前状态为“本地提交完成、远程推送等待用户认证”，
  不能写成已上传成功。
- 影响与边界：本轮没有修改算法、场景参数或运行方式；远程仓库尚未发生任何变化。
  完成认证后只需从本目录重新执行`git push -u origin ego1p0`。

### 2026-09-18 — 按用户确认改为直接从远程dev建立ego1p0

- 目的：满足“在dev主分支下新建分支，并把目前内容提交到新分支”的明确要求，避免
  把本机未推送的历史实验提交带入远程分支。
- 涉及文件：本地Git提交图和本工作日志；工作区其余文件内容保持当前EGO1P0快照。
- 修改内容：获取远程`dev=804f26e2e631d0571f5e1cc5088100481df1b16d`，计划以该提交
  作为`ego1p0`的直接父节点，并用当前完整文件树生成一个快照提交。此前基于本机
  `d267feb`建立但未推送的`d02b140/a24035c`不进入新的远程分支历史。
- 验证：`git fetch origin dev`成功，远程dev仍为`804f26e init`；当前工作区在重建前
  无未提交改动。GitHub页面显示`Chenwill1899`已邀请当前账号`dhmiaolovestar-hash`
  协作，但邀请尚待用户确认接受；本条记录时远程仍未创建`ego1p0`。
- 影响与边界：不改远程dev，不改算法、参数、场景或运行文件；只调整待推送分支的
  父提交和提交图。

### 2026-09-18 — ego1p0分支成功推送到GitHub

- 目的：完成当前EGO1P0版本的远程备份，并保持`dev`主分支不变。
- 涉及文件：本工作日志及本地Git配置；项目外新增专用SSH密钥
  `/home/starry/.ssh/id_ed25519_codex_github`，GitHub公钥标题为
  `Codex EGO1P0 starry 2026-09-18`。
- 修改内容：当前账号`dhmiaolovestar-hash`接受`Chenwill1899/Fov-gvf`协作者邀请后，
  使用GitHub SSH 443端口推送新分支`ego1p0`。分支首个快照提交为`ff08e31`，直接父
  提交为远程`dev=804f26e`；远程地址为
  `ssh://git@ssh.github.com:443/Chenwill1899/Fov-gvf.git`。
- 验证：SSH返回`successfully authenticated`；`git push -u origin ego1p0`成功报告
  `[new branch] ego1p0 -> ego1p0`并建立跟踪关系。推送后提交图为
  `ego1p0: ff08e31 -> dev: 804f26e`，工作区无未提交修改。此次只做版本管理，未重新
  运行构建/算法测试；待推送快照沿用2026-09-17已记录的ROS三包构建成功、9/9 CTest
  和真实手柄动态联调结果。
- 影响与边界：远程`dev`未修改、未合并、未变基；`build/install/log`和缓存继续忽略。
  未创建Pull Request，当前只是独立远程分支。

### 2026-09-23 — 北通 A2P3A BFM/XInput 双 profile 自动识别

- 目的：解决北通接收器以XInput模式枚举为`045e:028e Xbox360 Controller`时，EGO1P0
  启动器仍写死旧BFM by-id并在ROS/Isaac启动前误报“手柄不可读”的问题；同时避免只改
  路径后把XInput的LT axis 2误当成Roll，导致回中安全门无法解锁。
- 涉及文件：新增`scripts/lib/beitong_joystick.sh`和
  `MISSANDKEYBOARD/test_beitong_joystick.py`；修改通用、组合及独立三个手柄入口，
  `MISSANDKEYBOARD/beitong_joystick.py`、`KEYBOARD/keyboard_only.py`，以及版本说明、
  两份入口README、人工控制指南、Cloud人工控制说明和`DONG.md`；同步更新统一HISTORY。
- 修改内容：公共启动链现在扫描`/dev/input/js*`并用udev严格接受唯一一个北通A2P3A。
  BFM `20bc:511c`固定要求8轴16按钮，映射为Yaw/Throttle/Roll/Pitch=`0/1/2/3`；
  XInput `045e:028e`还必须匹配北通vendor/model，固定要求8轴11按钮，映射为
  `0/1/3/4`。显式`JOYSTICK_DEVICE`不能绕过身份检查，显式profile必须与检测结果一致；
  零匹配、多匹配、非字符设备、不可读设备和尺寸冲突均fail closed。轴/符号定制接口、
  四杆回中0.5秒、断连清零与键盘兼容模式保持不变。
- 验证：四个Shell文件`bash -n`通过；三份Python文件`py_compile`通过；新增4项unittest
  全通过，覆盖BFM、XInput、尺寸推断以及错误profile拒绝，且确认不能用旧expected环境
  变量绕过尺寸契约，并验证XInput空闲LT不会阻止回中解锁；`git diff --check`通过。
  本轮较早的真实设备检查中，接收器枚举为BFM `20bc:511c`，
  udev确认`/dev/input/js0`及稳定BFM by-id，ACL允许当前用户读取；解析器实测选择
  `profile=bfm`和`0/1/2/3`。把ROS安装前缀故意指向不存在目录运行主入口时，手柄门禁
  成功通过，随后按预期在ROS安装检查退出，没有启动ROS、Isaac或发布控制命令。最后
  收紧profile尺寸契约后尝试重复实机检查时，解析器按设计报告没有受支持设备；`lsusb`
  和`/dev/input`复核确认接收器已经不在系统中，因此没有把该次写成重复实机通过。
- 影响与边界：只改变北通设备发现、profile选择和对应输入轴，不修改FOV-GVF数学、
  ROS话题、场景、速度或末端安全逻辑。用户此前的`lsusb 045e:028e`说明接收器曾处于
  XInput模式；本轮最终实机检查时它已切回BFM，因此XInput路径由严格udev逻辑和单元
  测试覆盖，未写成XInput实机推杆通过。本轮未推动摇杆、未执行受控USB断连停车测试，
  也未运行完整Isaac/ROS动态避障。
- 回滚/后续：移除共享解析器和新增测试，并恢复上述启动器、Python输入层及文档的
  本次差异即可回到单一BFM基线。用户可用原命令启动完整链，现场继续核对四个方向、
  回中停车和拔出USB停车；若接收器再次显示`045e:028e`，终端应打印`profile=xinput`
  及轴映射`0/1/3/4`。

### 2026-10-16 — EGO2P3 三项方案实施开始

- 用户授权实施执行与证据连续性、辅助螺旋升降、有限风险探测与恢复；范围仅EGO2P3。
- 修改前备份：EGO2P3/performance/active_probe_implementation_20261016/before_source.tar.gz、before_sha256.json。
- 当前处于实现阶段；构建、测试和新仿真尚未执行，最终结果另行追加。

### 2026-10-16 17:28 — [EGO2P3] 实施执行/证据连续性、辅助升降与有限风险探测恢复

- 状态：三项代码、默认配置、独立开关、可重放状态和说明已交付；试用仿真验证完成。未提交/推送，无后台导航。
- 目的：按用户明确授权，在没有垂直传感器的四水平相机条件下处理空旷处因证据缺口卡停，并允许有预算地先运动获取信息。
- 项目：`/home/starry/isaac-data/EGO2P3`；P2未修改。P2的146项源码/脚本/配置SHA核对零变化，P3统一地图USD/occupancy双SHA不变。
- 修改文件：`src/pc_gvf/include/pc_gvf/paper_guidance.hpp`、`src/pc_gvf/src/{paper_guidance.cpp,active_navigation.cpp,paper_replay.cpp,depth_angular_controller_node.cpp}`、`src/pc_gvf/tools/paper_replay.cpp`、`src/pc_gvf/test/cpp/active_navigation_check.cpp`、`src/pc_gvf/{CMakeLists.txt,launch/isaac_cloud_navigation.launch.py}`、`scripts/isaac/run_fov_gvf_navigation.py`、`profiles/manual_10mps_depth20.env`、`ACTIVE_NAVIGATION.md`、`AGENTS.md`、`README.md`、`EGO2P3_VERSION.md`、`MANUAL_CONTROL_PROFILE.md`、`WORK_LOG.md`及`performance/active_probe_implementation_20261016/`实验记录；同步HISTORY总账、README和FOV_GVF_FAMILY。
- 执行/证据：新开关下由25ms梯形积分改为与实际60Hz端点速度推进一致的离散响应/制动模型；延迟内保持旧目标；独立保留最新完整命令/E证明，脱离256条空间历史淘汰，真实命中撤销冲突。跟踪余量0.015m、restart0.02m为明确试用值，未证明真实飞控或任意通信延迟最坏界。
- 辅助升降：纯升降允许两旋向圆弧和侧移，按相机投影/占据遮挡估计关键未知体积的可见性；不强制完整绕圈或自动偏航。无实际进展的小圆弧不无限优先；缺口处先侧移补视差，再认证升降。水平输入接管、松杆取消。RViz显示路径、目标、授权/模式及预算。
- 探索/恢复：先完整认证，再独立低速风险判定；已知占据、地面、过期输入/观测继续拒绝。风险不产生自由球/管段。事件预算跨重规划/松杆保存，回退包含转向/制动预留；新视角提供此前缺失目标且机体内侧完整覆盖后才补充额度。active未知时间与含暂停的总未知暴露分别记录；体积为暴露估计而非概率。
- 当前参数：Cloud入口与保存profile三项开启，C++基线默认关闭。已知空间请求上限10/垂向1与普通入口6保持；辅助水平0.4、圆弧偏好半径0.8、探测0.3/垂向0.2、未知推进3.5m/活跃未知12s/体积估计5m³、锚点3m、额外占据净距0.08m。初试1.5m/8s/3m³及半额撤回不能获得足够视差，经过中间失败与几何检查调整；阈值仍为工程试用。
- 重放：新模式S携带策略参数、预算/锚点/真实观测目标、原始点及完整提交证明；旧3—Q及本轮R兼容，关闭开关仍输出Q。13个旧快照基线决定逐项不变；新模式空旷组4探测/1恢复，近障组8拒绝；4份R解析通过。
- 构建/测试：只使用主构建Shell，最终3包通过（build_12.log）；`PYTHONNOUSERSITE=1 ctest ...`42/42；平台/工具pytest168/168；实际源/安装启动配置6/6。增加双核心预算/污染/释放/过期/占据/floor/重放/提交证明撤销断言，含四水平相机合成升降返回和解析隐藏顶板/侧墙；后者最高0.242m，逐步机体碰撞检查0。
- 实际运行：只经主运行Shell，8条33秒Isaac轨迹（4中间版本+4最终消融），全轨迹扫掠重叠0、外部保护0、探索授权预算违规0、松杆非零目标0。最终同源/产物/地图/四相机/人工偏航0的基线、仅执行、执行+恢复、三项全开最高Z分别4.696/4.681/4.681/5.728m；完整组合下降至2.113m、水平接管位移1.062m，未知推进最大1.189m、活跃未知4.383s、体积估计2.205m³。实际端点积分和离散响应误差0；完整组合avoid P95/P99/max=12.326/18.209/79.991ms，不是1ms验收。各组一次，不宣称统计稳定性或全图到达。
- 保留失败：首轮脚本未通过bash调用权限错误；重放诊断缺Eigen几何头导致链接失败；混合新旧结构布局对象及用户NumPy环境导致首次CTest失败，重编译和正确环境后通过；小圆弧无进展、回退预留不足、到锚点不退出、水平目标缺口/速度候选过粗均在中间合成/Isaac轨迹暴露并修正；一次编辑位置错误导致build_08失败并更正。ROS socket沙箱拒绝、GPU沙箱不可用、ROS查询缓存单发布者拒绝启动、两次参数查询失败均保留；获本机通信/设备访问后仍保留原共享锁和单发布者检查运行，最终4组实际参数成功读回。日志与机器清单见completion_summary.json，失败未覆盖。
- 影响与边界：原六项补充模块开关、地图和相机不变。探索剩余碰撞风险不归零；返回限直接锚点连接。没有真实飞控、真人手柄、10m/s全图或1ms稳定验收；未新增Isaac隐藏障碍场景，隐藏顶板/侧墙仅为解析传感器闭环负例。
- 备份/恢复：`performance/active_probe_implementation_20261016/before_source.tar.gz`、before_sha256.json。原运行脚本在before_scripts按新增CSV差异逆向恢复，与P2相应脚本只替换P3身份后完全一致，comparison.diff为空；最终源/产物在final_delivered_source.tar.gz和SHA清单。最终消融后只增加测试断言，实际控制器SHA不变。可用三个开关0且restart0作旧严格分支对照；正式源码回滚须核对后续修改再逐项恢复并重新主构建。
- 后续：真实执行/传感误差标定、更复杂恢复搜索、真人手柄和高速全图稳定性不在本次通过范围。详见`ACTIVE_NAVIGATION.md`与`performance/active_probe_implementation_20261016/RESULTS.md`。

### 2026-10-19 16:19 — [EGO2P5] c31 R07终点前预算卡停，c32补充低速认证返回与实际停止记账

- 状态：c31实际1次R07失败，后19槽未执行；c32源码已修改，构建中，尚未新飞行。
- 目的：继续用户授权的失败归因与优化，不增加未知预算。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：paper_guidance.hpp、active_navigation.cpp、depth_angular_controller_node.cpp、native_history_recovery_check.cpp；performance/iterative_recovery_20261019_1425/c31_validation、candidates/c31_failure_analysis.json、final_stall_audit.txt、c32_build.log；WORK_LOG/HISTORY/CHANGELOG。
- 修改内容：认证直接返回/侧向扇区加入1/.5/.25/.125速度梯度，每条仍完整检验运动/制动。节点保存求解前实际已发布指令；超时零速后以该延迟目标和实际物理包络的停止路径确认下一周期是否未知，避免被取消的候选last_unknown继续计时；既有累计距离/时间/初始体积与事件原点不回滚，真实未知停止/延迟运动仍计入。计时包含该确认开销。
- 验证：c31于151.433s自然卡停，距终点3.874m，累计实际未知距离2.976m/11.35s/4.192m³。全部原审计通过，扫掠/外部保护/限值违规0。冻结终末快照机体.595及重启.615均已知，+.03不知，单一返回速度搜索无解；1个见证未新覆盖。12个内部试探候选被运行时超时改为零速，是否影响闭环结果须新运行验证，不能将全部真实探索归因于取消记账。原录像完整2277帧验证通过，兼容MP4导出中。新增回归覆盖已知静止取消不耗探索时间、未知机体仍记账、延迟旧指令进入未知仍记账；尚待构建执行。
- 影响与边界：硬包络、10m/s请求、.20s延迟、3.5m/12s/5m³及80ms期限不改；地图/传感器/P4/服务器不改。
- 回滚/备份：c24完整基线及c26—c31源码/实际ELF/SHA/原轨迹与录像均保留；无提交推送。
- 后续事项：c32构建/63项回归、同一终末快照反事实，随后重新四路线各5次验证，若自然失败继续分析。


### 2026-10-19 16:21 — c32构建与冻结反事实补充

主构建3包通过；初轮CTest61/63，certified_recovery_check两构建模式失败，原因是测试在step已确认新指令后用新指令当旧延迟目标复核证书。修正为保存/恢复实际求解前零目标，并明确验证原.3不通但.15完整认证可通的返回梯度，非放宽几何判断。原失败日志保留。其余新记账回归通过，完整重跑中。与c31末期完全相同冻结快照，c31预算拒绝/34.808ms，c32找到ACTIVE_CERTIFIED_SIDESTEP，指令(.00654,-.07471,0)，7.431ms；只支持该局部动作，不代表闭环路线已通过。补充修改文件certified_recovery_check.cpp，最终源/实际ELF/SHA保存在candidates/c32。

2026-10-19 16:22 c32补充：最终CTest63/63通过，实际安装ELF dfbef6fbbfd792f5e27699d4b10002462f78252e08b6917fc4cfba88cf2a67d5；已启动四路线各五次串行可见/录屏验证，先R07。尚无新到达结论。

### 2026-10-19 16:51 — [EGO2P5] c32第14次R06边界振荡失败，c33加入边界恢复滞回

- 状态：c32实际14次13安全到达/1卡停，后6槽未执行；c33源码修改与构建中，尚无新飞行。
- 目的：依据本轮再次失败继续迭代，避免只退出几厘米就重新前进的边界循环。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：paper_guidance.hpp、active_navigation.cpp、paper_replay.cpp、depth_angular_controller_node.cpp、native_history_recovery_check.cpp；本任务c32_validation、candidates/c32_failure_analysis.json/失败审计/录像/静态核心、c33构建日志；WORK_LOG/HISTORY。
- 修改内容：新增持久边界方向，水平输入进入上/下边界15cm带后持续内退，退出35cm带才交回正常导航；若当前.635m可靠内部已证明，以实际当前位置替换旧上沿返回点，保留固定探索原点和全部累计债务。松杆保留边界状态，明确纯升降接管回到原辅助策略；新X重放记录方向并兼容3—W。无可靠返回点时的1Hz完整搜索同时清除核心旧角域/持有目标和节点提名/方向迟滞，恢复真实已发布延迟指令。
- 验证：c32 R06/4于287.733s自然卡停，位置(100.527,2.582,19.0287)、距157.601m，时间债务11.6167s；最高Z19.0696，反复内退后又回到同一上沿区域，原审计/源冻结/样本完整通过，扫掠/保护/限值违规0。前13次成功均保留，含R09/3长时间低速循环后284.517s才到达（均值11.537/最大806.805ms），不能把可到达称无卡顿。c32全部14原录像完整核验和失败MP4导出正在进行。新增滞回单测覆盖跨过入口后继续内退、X重放一致、深退后不追旧高锚点；准备与c32保存旧核心负回归。
- 影响与边界：只扩大退出空间余量，硬包络、未知预算3.5m/12s/5m³、10m/s请求、.20延迟、传感器/地图/80ms期限不变；没有清账。
- 回滚/备份：c24完整基线及c26—c32逐版源/实际ELF/SHA，c32静态核心用于负回归，所有6次中间失败和未跑槽保留。无P4/服务器改动，无提交推送。
- 后续事项：构建/回归后重新优先R06，四路线各5次验证；若自然失败继续归因优化。


### 2026-10-19 16:57 — c33最终源构建/回归并启动可见验证

边界模式补充禁止朝外重新入轨/返回候选，未知许可要求向内高度进展；允许完整认证的侧移取得视差。新增“重启壳未证明但机体已知、有限风险内退”用例通过。最终全部源完成后明确重编核心/重链并安装，63/63回归通过；源/实际手柄ELF（62381e07922729d5740e84a8366d39f1d2f291d9977a1899d46c46fca0a3083c）/重放器/配置SHA保存在candidates/c33。旧c32保存核心执行新增滞回回归明确失败，信息boundary recovery exited after only crossing its entry band；新c33通过。已启动R06/R07/R09/R01各5次串行可见/原机位录屏验证，尚无c33飞行结论。c32全部14原录像完整解码通过；失败兼容MP44059帧通过，首次导出核对因先于原始录像验证结束缺少对应JSON而失败，依赖完成后重试通过，原日志保留。P4/服务器/地图不改，风险与硬限值不改，无提交推送。

### 2026-10-19 17:17 — [EGO2P5] c33六次全到达但恢复严重低效，c34修复模式切换饥饿并调整返回足迹

- 状态：c33实际6次全安全到达，14槽未执行；并非新增失败。当前飞行自然结束后停止批次作效率修订，未截断任何轨迹。c34构建/回归中，尚无c34飞行。
- 目的：用户要求判断优化提升；不以“最终到达”掩盖332/347s的恢复循环。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：navigation_progress.hpp、depth_angular_controller_node.cpp、active_navigation.cpp、navigation_progress_check.cpp、native_history_recovery_check.cpp；c33_validation原轨迹/审计/录像、candidates/c33_efficiency_analysis.json、c34构建及旧核心负回归；WORK_LOG/HISTORY/CHANGELOG。
- 修改内容：普通/局部恢复切换不再清空目标进展监视；仅明确verified_return期间给返回动作独立完成窗口，之后保留原2s/1Hz搜索规则。卡停可从普通或局部恢复请求验证返回。返回点由最近.35..3m改为优先约1.5m可靠实际足迹，允许.12..3m，只有保存几何满足.635m且新鲜完整复核的点才可选；最多复核12个候选，避免搜索无界。原固定探索原点、累计债务、真实已发布延迟目标保留，足迹只是提名不授予运动，后续完整认证/风险许可保持。
- 验证：c33 R06两次332.083/346.633s到达，均值10.674/15.015ms，R07两次68.683/50.550s到达，R09 43.833s/R01 72.5s到达。全部源冻结、原审计、样本完整通过且扫掠/保护/限值违规0。第二次R06末段5m附近重复普通/局部恢复，300多秒只有少量PROGRESS_SEARCH，源代码确认每次active_frontend清空2s计时。c34新增交替模式不抹除停滞、显式验证返回后新窗口、近锚点可用及更有位移足迹优先的回归；尚待构建执行。停止标记写入时R06已结束并启动下一条R07，因此让该R07也自然到达后停止，共6条，无人工中断。
- 影响与边界：返回提名距离是本次优化参数，可靠半径.635、硬包络/风险预算/10m/s请求/.20延迟/相机/原地图/80ms期限不变。c33六次成功不能当20次验收，效率退化明确保留。
- 回滚/备份：c24完整基线及c26—c33逐版源/实际ELF/SHA、c33旧静态核心、原6次中间失败和全部实际成功/未执行槽保留；无P4/服务器改动，无提交推送。
- 后续事项：c34构建/回归后优先R06重新四路线各5次，若自然失败继续归因。


2026-10-19 17:19 c34构建3包、CTest63/63通过；c33旧核心执行新增近返回点回归明确失败，日志保留；新c34通过近点/远点优先、模式切换进展、W/X重放等回归。c33全部6原录像完整解码通过。c34源/实际ELF/SHA保存，实际手柄ELF9b5345d422fa6220eef773b2859066e588fbd577e6c4e451c37ac1608ce881c9，开始R06/R07/R09/R01各五次串行可见验证，无新飞行结论。

### 2026-10-19 17:30 — [EGO2P5] c34 R07认证侧移反复超时，c35复用短时主动恢复提名

- 状态：c34实际2次1安全到达/1失败，18槽未执行；c35源码修改和构建中，尚无c35飞行。
- 目的：直接处理“每次都算出合法侧移但每次都被计算期限改零”的计算饥饿。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：paper_guidance.hpp、active_navigation.cpp、paper_replay.cpp、certified_recovery_check.cpp；c34_validation原轨迹/样本/审计/现场录像、candidates/c34_failure_analysis.json/同指令诊断/静态核心、c35构建日志；WORK_LOG/HISTORY。
- 修改内容：只保存一个短时主动恢复候选，TTL.3s/位移.75m/意图角/恢复模式与锚点一致才可提名；每次仍以当前深度、实际已发布延迟目标完整重证名义运动和紧急制动，先过真实机体障碍检查。被80ms取消的候选只保留为不可信方向，真实发布历史继续改零；复用不续原TTL，失败失效，松杆或选新返回点失效，未知许可不存入该候选。新Y格式保存候选及其捕获时空/意图/模式，兼容旧3—X。
- 验证：c34 R06首轮54.767s安全到达/均值1.666ms；R07首轮107.667s自然停滞、距9.188m、最后未知距离/时间均0，体积.882m³。最后一分钟189个ACTIVE_CERTIFIED_SIDESTEP均被超时取消（另41个缓存命令未带来有效运动），不是预算已耗尽。完全相同终末快照，原完整搜索199.532ms，同一个(.01424,-.03469,0)m/s指令的完整运动/制动证明仅.05763ms且有效。七次中间失败全部保留；原审计、冻结身份、样本完整通过，扫掠/外部保护/限值违规0；2原录像完整解码通过，失败MP4导出已完成。新增回归覆盖超时后实际零发布仍能优先重证、TTL不续、Y重放一致及真实障碍仍硬拒绝；尚待构建执行。
- 影响与边界：缓存只提名，不复用执行许可、不写入未知自由、不清账；.595+.02硬半径、3.5m/12s/5m³、10m/s/.20延迟与80ms期限不改。新主动提名独立于原goal_proposal_cache参数，普通缓存默认开关值不改。
- 回滚/备份：c24完整基线、c26—c34逐版源/实际ELF/SHA/全部成功失败与未执行槽、c34静态核心与原单指令计时诊断。无P4/服务器/地图改动，无提交推送。
- 后续事项：c35最终构建/回归、相同旧快照下一周期零发布反事实，随后重新四路线各五次验证。


2026-10-19 17:32 c35主构建3包、最终有实际时间前进的原生障碍否决用例重建、CTest63/63通过。旧c34核心执行新候选重放回归失败（active nomination omitted from replay），日志保留。相同c34 R07终末快照，首次搜索195.462ms仍会超时；模拟实际零发布后50ms的下一周期，ACTIVE_CACHED_CERTIFIED完整当前重证3.62655ms，(.01424,-.03469,0)m/s，未知时间0/体积.882347保持。新Y快照用实际安装标准paper_replay返回相同状态/指令，这是离线下一周期反事实，不冒称闭环。源/实际ELF/SHA保存c35；即将重新R07/R06/R09/R01各5次可见验证。

### 2026-10-19 17:45 — [EGO2P5] c35 R06边界无路与一次启动崩溃保留，c36补充受约束水平视差

- 状态：c35三次启动含一次飞行前RUNTIME_ERROR，重试后2次实际飞行1安全到达/1卡停，18计划槽未执行；c36源码修改/构建中，尚无c36飞行。
- 目的：继续处理真实失败，不将新的“无候选”错误归为已认证指令超时。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：paper_guidance.hpp、active_navigation.cpp、paper_replay.cpp、native_history_recovery_check.cpp；c35_validation原全部尝试/启动日志与崩溃转储/2原现场录像/失败MP4、candidates/c35_failure_analysis.json、c36构建和旧核心负回归；WORK_LOG/HISTORY。
- 修改内容：保持15/35cm边界滞回，先尝试朝内下降/上升，受测量障碍阻挡后允许水平有限风险视差（向锚点及±30/60/90度），仍禁止朝外撤销高度内退，所有路径继续完整原生障碍/工作域/制动/返回/3.5m-12s-5m³检验。原严格“必须朝内有垂速”排除了唯一可用水平运动，现改为禁止朝外。已认证主动候选允许边界模式内短时复用，捕获边界方向并匹配当前模式/方向，退出模式不再复用该候选；Z记录边界标签，兼容3—Y。
- 验证：c35 R07首轮48.333s安全到达/均值1.732ms；R06在(121.345,17.952,18.9474)卡停103.667s、距140.629m，未知时间仅1.15s/距离.132m/体积1.029m³。最后一分钟584个内部PROBE_NO_USEFUL_PATH，528被总计算期限改零；不是上一轮已认证侧移的相同根因。停止处离物理.48机体的真值净空.17085m；在线仅用原生观测，下方障碍阻挡内退而水平约束过严。新增物理一致合成用例：下方命中在其历史来源相机FOV内、当前壳未知、可靠锚点不包含障碍，内退失败时允许原预算水平视差；旧c35核心负回归已失败，待新c36回归。首次R06启动Isaac crash/visualizer -11在零指导样本前发生，原RUNTIME_ERROR、日志和runtime/kit转储保留，另一次启动重试才生成上述飞行；不算两次飞行。2原录像完整解码通过，失败MP4导出完成。
- 影响与边界：未放松.595+.02硬包络、真实障碍、风险速度与累计预算，水平动作仍EXPLORATORY而非自由证明。内部边界候选每次新证书、TTL不续，.3s/.75m保持。地图/P4/服务器不改，无提交推送。
- 回滚/备份：c24完整基线及c26—c35逐版源/实际ELF/SHA，c35旧静态核心，8次实际中间失败和1次无飞行启动失败全部保留。
- 后续事项：c36最终源重建/回归、相同旧失败快照检验，再重新四路线各五次验证。


### 2026-10-19 18:02 — c36真实冻结状态补充诊断与恢复可选缓冲标定

初版c36新增侧移及Z状态后63/63通过，但同一c35 R06末期快照仍PROBE_NO_USEFUL_PATH（87.613ms），因此未启动新批次。离线逐候选检查显示所有初次拒绝均为COMFORT，距离.622865–.624857m，差于.615硬半径外另加1cm的.625m软阈值；去掉该恢复额外1cm后，纯向内运动仍因.6139–.6149m真实硬约束被拒绝，而水平侧移(-.10724,.28018,0)m/s通过全部.615硬约束/制动/返回/原预算，输出EXPLORATORY_RECOVER，旧账本1.15s/.1322m/1.0294m³不变。该带诊断打印版本88.127ms，尚不能作为可实时闭环验收。

按用户算法/参数优化授权，将仅恢复附加舒适内侧1cm→0；正常探测8cm保留，硬包络.595+.02、风险.3/垂向.2、3.5m/12s/5m³、80ms期限均不改。源码active_navigation.cpp保留对clearance<=hard_radius的直接否决，恢复不再要求额外1cm；不是缩小硬机体。同步将纯几何边界模式选择前置，实际机体/轨迹检查仍在activeStep完整执行；恢复记忆增长只检查独立4cm可靠内部，不再每帧为可选10cm扩张做4次二分，已有证明与当前深度照常参与运动认证，失败扩张不生成自由空间。paper_guidance.cpp/native_history_recovery_check.cpp相应修改，合成下方障碍点改至有效包络+.025m，保证朝内候选真正触碰硬约束、水平仍可行。最终策略正在重新完整构建，尚未宣称修复真实闭环或通过最后回归。此条补充/细化此前“侧移缺失”归因，原单元通过与真实重放无解两项证据均保留。

### 2026-10-19 18:10 — c36原生障碍整曲线粗筛与碰撞不漏检验证补充

恢复额外舒适0、模式选择前置、可选记忆增长缩减后的主构建/63回归通过，但同一旧失败状态核心仍80.825ms，虽已输出原预算水平EXPLORATORY_RECOVER却不够运行时余量，因此仍未新启动飞行。继续在active_navigation.cpp/proof_query_memo.hpp加入整条分段曲线的保守胶囊包络，仅用于排除绝不可能相交的远原生点；可能相交的所有原点继续逐段精查，世界域/动态/共享检查仍每段保留，不能用粗包络相交直接判碰撞。这样直线返回和零位移制动不再对每个60Hz小段重复扫描全部原生点。proof_query_memo_check.cpp增加200条随机12段变半径路径、含闭环/退化/两档缓冲、48000个按构造必在实际胶囊中的点不被粗筛漏掉、远点确实剔除与非法输入不剔除的性质检查。最终完整源重编/安装/回归与精查参考版差分尚待执行，原80.825ms结果与所有前述失败日志保留。记录涉及源、测试、performance工作诊断和WORK_LOG/HISTORY；硬.615与风险限值、地图/P4/服务器保持。

### 2026-10-19 18:15 — c36最终源验证并重启完整路线复测

最终完整源重编/重链/安装成功，63/63回归通过（其中胶囊包络两个构建模式各48000个构造碰撞点不漏检）。同一c35 R06失败状态，c36生产核心接受EXPLORATORY_RECOVER(-.10724,.28018,0)m/s，核心34.5085ms，实际安装标准重放器与计时诊断输出完全相同，原1.15s/.1322m/1.0294m³账本保留。隔离粗筛开关、其它c36策略及恢复额外缓冲0完全相同的逐段精查参考版与生产粗筛版，32份真实旧冻结状态状态/指令全等，中位45.7107→27.50715ms；这是离线计算验证，尚无c36闭环成绩。旧c35执行最终合成下方障碍回归仍明确失败，负回归日志保留。c36活动源/实际ELF/SHA存档，手柄ELFdfe4802de80de6b8ae301e08aa97204432bbf54c53d6d83dbc429c808f67ad33；即将R06/R07/R09/R01各5次串行可见录屏验证。硬.615、普通探测舒适.08、原风险/速度/传感器/域/期限保持；仅恢复额外舒适1cm→0按前条记录。

### 2026-10-19 18:23 — [EGO2P5] 按用户范围排除顶/底界斜角围困并补实验

- 状态：排除口径已登记；c36首轮R06原始USER_EXIT保留，正式20次补跑待执行。
- 目的：遵照用户连续明确要求，到顶或到底进入斜角卡死不作为本次局部避障优化目标，剔除后补新实验。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：performance/iterative_recovery_20261019_1425/exclusion_policy.json、c36_validation/exclusion_review.json、plan.json、supplement01_schedule.json、work/run_trials.py及旧脚本备份；WORK_LOG/HISTORY。
- 修改内容：仅实验调度增加显式剩余槽计划与拒绝覆盖旧结果，顶/底斜角围困经轨迹及录像人工复核另账剔除，同算法/参数/起终点补足每路线5次；不是自动把所有近边界或失败列为排除。生产源码/参数/ELF未改。
- 验证：c36 R06原始USER_EXIT、172.483仿真秒、距163.675m，最后60秒顶界附近微动，原状态不冒称自然超时或自然卡停。轨迹/冻结/全样本/探测审计通过，扫掠/保护/限值违规0；原无人机+RViz录屏2668帧完整解码。新调度脚本AST校验通过，补跑尚未执行。
- 影响与边界：只改变当前任务验收纳入范围；原基线14/20及全部中间失败不重写，后续对比明确纳入范围差异。未证明此几何全局不可解；不针对该用户排除工况继续优化。
- 回滚/备份：run_trials_before_user_exclusion.py.saved；全部原始尝试/录像保留。
- 后续事项：同一c36补实验并分别给出原始与排除后的统计。

2026-10-19 18:24 排除补跑配套记录：work/aggregate_formal.py新增同c36跨批次正式/排除/全部原始飞行分别汇总，要求逐批冻结身份完全相同、每路线不超过5次，任何剔除必须在明确exclusion_policy中且引用真实记录；work/compare_batches.py避免聚合摘要重复计数并显式输出排除数/纳入范围变化；work/verify_final.py按每次来源批次完整核验录像，额外检查排除记录保留，最终20次仍要求原审计accepted及安全到达才能宣称全部通过。全部工作脚本AST通过；最终汇总和核验尚未执行，c36_supplement01补跑正在进行，生产冻结文件未改。

2026-10-19 18:28 补充复核：c28/c29/c32/c35共4次早期R06未到达，最后60秒高度分别19.181、19.084–19.085、19.029、18.947m，净位移0–.003m，已复核保留的现场图像与对应原角度录像入口，按用户新范围追加historical_exclusions，原技术归因与原审计/失败状态保留。当前c36另1次USER_EXIT按明确工况排除；历史排除不混入当前c36正式次数。未观察到新的底界斜角样本，规则已覆盖底界但不能声称实测该情形。make_report/update_status已适配正式与原始统计及基线纳入范围差异，AST通过，最终报告尚未生成。原候选24基线14/20原始统计保持。

2026-10-19 18:30 补跑整理：prepare_remaining.py只根据正式分账输出缺失槽，拒绝覆盖旧批次/旧计划；update_status.py最终总账采用顶部追加并保留旧条目，make_report.py逐次账本显式正式纳入/用户工况排除标志，所有候选原始状态与无飞行启动失败继续分别记录。工作脚本AST通过；这些最终报告/状态工具尚未执行。当前补跑已完成R06第一轮227.6s与R07第一轮41.083s，两者安全到达；R06长时间中层微动仍计入正式统计，没有按斜角工况剔除。

2026-10-19 18:31 报表口径进一步明确：compare_batches/make_report同时展示原始候选24、c36全部原始飞行、用户范围c36正式实验三列，分路线图使用各自实际分母及到达比例，避免剔除导致的20/20被误读为原始无条件结果。原始与正式计算均值分列；最终结论明确基线与正式纳入范围不同，不能把变化全部归因于算法。工作脚本AST通过，最终报告生成仍等待完整复测。

2026-10-19 18:39 正式明细补全：aggregate_formal的逐次表包含安全到达、用时、距终点、全程计算均值/P95/最大、期限改零样本及原轨迹/现场录像；主报告附现行手柄入口与本轮相同R07双窗口命令，明确自动意图验证不等于真人手柄操纵。AST通过，最终报告尚未生成。运行入口run_manual.sh的旧“候选24/尚无本机飞行”文字待全部飞行后单独改为当前版本文档引用，仅更新显示文字并另存前后来源，运行期间不改冻结源。

2026-10-19 18:46 收尾核验工具准备：verify_all_evidence.py汇总所有真实批次（跳过正式聚合，避免双计数），逐次重核原录像SHA、既有完整解码/握手/视角记录、全样本与冻结身份和探测审计，所有未到达飞行兼容MP4帧数/SHA单列，飞行前启动失败另账；尚未执行。update_manual_banner.py只允许全部正式20次通过、所有来源批次结束后替换一行旧printf，verify_final相应仅接受该固定文字的前后SHA与逐字差异，仍拒绝任何其它源码变化。工作脚本AST通过；当前生产入口及全部算法源在飞行期间仍未改，16/20正式实验已安全到达。

### 2026-10-19 18:51 — [EGO2P5] 飞行结束后更正手柄入口旧版本显示文字

- 状态：完成，bash -n通过；未新增真人手柄飞行。
- 目的：避免当前控制器已为c36，终端仍显示候选24/本机尚未飞行的旧提示。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：deployment/local_ubuntu/run_manual.sh；本任务post_validation_before/post_validation_display_change.json/manual_banner_bash_n.log；WORK_LOG/HISTORY。
- 修改内容：只替换一行printf为当前版本与验收范围见EGO2P5_VERSION.md，参数/环境/执行命令/算法源码/实际ELF均不改。
- 验证：所有来源批次已结束且正式20/20安全到达后才编辑；原脚本SHA与飞行冻结身份匹配，新旧文件除该一行完全相同；bash -n返回0。源码冻结核验单列此结束后的显示变更，不冒称飞行使用了改后脚本。
- 影响与边界：仅启动时文本，未因该显示变更重跑仿真，原飞行证据保持。
- 回滚/备份：post_validation_before中的完整旧脚本及前后SHA。
- 后续事项：最终来源核验、报告与交付。

### 2026-10-19 18:54 — [EGO2P5] 本机c36四路线各五次完整验证与迭代交付

- 状态：完成本次优化和比较验证；最终同版按用户排除顶/底斜角工况补跑后正式20/20安全到达；原始结果另列，仍未达到稳定1ms。
- 目的：落实用户“按判断优化、仿真判断提升、失败继续归因”的连续授权。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：src/pc_gvf/include/pc_gvf/paper_guidance.hpp、proof_query_memo.hpp、navigation_progress.hpp；src/paper_guidance.cpp、active_navigation.cpp、depth_angular_controller_node.cpp、paper_replay.cpp；test/cpp/native_history_recovery_check.cpp、proof_query_memo_check.cpp、navigation_progress_check.cpp、certified_recovery_check.cpp；本任务performance/iterative_recovery_20261019_1425原数据/逐版源ELF/SHA/失败视频/报表/补丁/脚本；README/EGO2P5_VERSION/ACTIVE_NAVIGATION/AGENTS/CURRENT_CHECKPOINT/WORK_LOG与HISTORY/README/FOV_GVF_FAMILY/CHANGELOG。
- 修改内容：精确几何查询及重复路径复用、首次认证速度即返回、边界15cm进入/35cm退出滞回内退及受阻后原预算水平视差、旧提名清除、跨普通/局部恢复的目标进展监视与优先1.5m/.12–3m验证足迹返回、真实未观测壳见证、四档认证返回、主动恢复候选.3s/.75m按边界标签优先完整重证、原生点整曲线保守包络粗筛及近点逐段精查、超时取消后按实际延迟停止路径提交未知标志；W/X/Y/Z重放保留返回/边界/主动候选模式。恢复附加舒适1cm→0，正常探测8cm保持；恢复记忆不做可选10cm半径4次二分，只查询可靠4cm。具体参数和逐次原因见完整报告及各版日志。硬包络.595+.02、未知预算3.5m/12s/5m³、10m/s请求/.20s延迟/16份历史/四水平相机/原地图/80ms期限不改。
- 验证：最终c36 R01/R06/R07/R09各5/5，原始基线14/20、用户新范围正式20/20（纳入范围不同，不能全部归因于算法）；当前c36原始21次，按用户范围排除1次，原状态保留；逐次计算均值再平均9.545852→1.647868ms（原始基线包含失败，后值为排除工况后的正式实验，每次同权，范围差异明确披露）。原审计20/20 accepted，扫掠重叠/外部保护/全部限值违规0，源安装有效参数及端点/地图绑定一致；20原现场双窗口录像完整解码、开始早于仿真、机位调整0。全部72次新仿真含9次未到达（含工况排除/原始USER_EXIT）与各版未执行槽完整保留，另1次无飞行启动失败单列，共73次启动尝试；失败飞行MP4逐帧解码核对。主构建3包、最终CTest63/63通过；c29编译失败、c32初轮61/63测试前发布目标假设纠正及重跑63/63、c35 Isaac启动崩溃均保留，未冒称重跑174工具。详情performance/iterative_recovery_20261019_1425/RESULTS.md、final_verification.json、ALL_ATTEMPTS.csv、candidate24_to_final.patch。
- 影响与边界：自动目标意图进入手柄策略use_fixed_goal=false，非真人操纵/实机；没有全图真值在线注入，纯升降和任意路线长期可靠性不在此次范围。中间c32版R09第三次284.517s才到达，最终版仍有单帧长尾；无卡顿与稳定1ms未验收，到达率改善不等于每路线均更快。最终没有热改、后台续跑、提交或推送，P4和服务器不改。
- 回滚/备份：baseline_candidate24含原完整活动源/实际ELF/静态库及SHA，c26—c36逐版源/ELF/SHA和全部失败保留；最小差异补丁可审查，文档旧状态备份documentation_before_final并保留原时间含义。
- 后续事项：无本轮自动续跑；若后续要求更严格实时性/长期手柄，再以当前冻结结果为基线单独验证。


### 2026-10-19 18:55 — [EGO2P5] 补齐手柄配置与部署说明的当前c36状态

- 状态：完成文档状态同步，最终冻结核验待按新增文档差异复核；无新飞行。
- 目的：消除MANUAL_CONTROL_PROFILE与deployment README仍把候选24/1cm/无本机飞行写为当前的歧义。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：MANUAL_CONTROL_PROFILE.md、deployment/local_ubuntu/README.txt；work/verify_final.py/make_report.py/publish_outputs.py、post_validation_documentation_change.json与备份；WORK_LOG/HISTORY。
- 修改内容：保留完整旧文档并明确历史时点，前置c36当前配置、20次正式与21次原始的不同口径、用户顶/底界斜角工况范围及未真人手柄/稳定1ms边界；部署README仅追加状态前缀，旧启动命令逐字不改。最终核验对这一固定README路径只接受前缀+原文件、前后SHA与旧冻结身份完全对应；其它算法/参数变化仍拒绝。交付脚本准备只复制完成报告/完整账本/全部9份未到达兼容录像，原始数据保留；尚未执行交付。
- 验证：前置文档之前所有飞行已结束，原部署README SHA与211项飞行冻结身份匹配；工作Python AST通过。已有全部72条原录像/9未到达MP4 SHA及完整解码、73条尝试账本核验通过。
- 影响与边界：文档与一行printf发生在飞行结束后，不冒称飞行使用了改后文档/脚本；算法/ELF/实参不改。
- 回滚/备份：documentation_before_final和post_validation_before中的完整旧文件与JSON前后SHA。
- 后续事项：更新最终核验、报告与交付副本。

### 2026-10-19 18:57 — [EGO2P5] c36最终证据与交付完成，无后台续跑

- 状态：完成。
- 目的：交付用户范围内的四路线各五次正式复测与全部失败/排除记录。
- 项目路径：/home/starry/isaac-data/EGO2P5。
- 涉及文件：performance/iterative_recovery_20261019_1425/RESULTS.md、c36_formal摘要/明细、ALL_ATTEMPTS.csv、comparison.json、exclusion_policy.json、final_verification.json、all_evidence_verification.json、cleanup_verification.json、delivery_manifest.json/delivery_verification.json、最小补丁、工作核验/报表/交付脚本；MANUAL_CONTROL_PROFILE及其它已记录状态文件；输出/home/starry/Documents/Codex/2026-10-18/ego2p5/outputs/算法迭代优化_20261019。
- 修改内容：完成正式/原始/用户工况排除分账、三列对比图、每次计算与到达明细、全部9份未到达MP4交付；明确当前c36原始21次20到达/1顶界工况排除，正式20/20，各路线5/5；早期4次顶界围困另作历史排除标注，底界规则已覆盖但本轮无新增底界样本。
- 验证：211项冻结输入核验通过，飞行期间零变更；飞行结束后仅run_manual一行printf与部署README状态前缀两项源码树内显示/文档变化，原文件SHA与冻结输入对应、其它文件仍完全相同。20原到达/传感/实参/起终点审计accepted、全轨迹及66511完整指导样本有效；21原始飞行扫掠重叠/保护/限值违规0。全任务73启动尝试=72实际飞行+1飞行前启动失败，全部72条原现场录像完整解码/握手/双窗口/机位0记录及SHA复核、9条未到达MP4全帧数/SHA核验通过。最终核心63/63沿用本次c36最终构建已执行结果，未在收尾重复编译/174工具；入口仅文字改动bash -n通过。正式计算均值1.647868ms、全部c36原始1.937124ms，R01由1.158→1.451ms略退化，最慢R06 227.6s、最大1668.216ms，稳定1ms/无停顿未通过。24份交付副本（含9视频）与2份生成文件SHA、73行账本及全部报告本地链接检查通过。
- 影响与边界：本机仿真/自动意图进入手柄策略，未新增真人手柄或实飞。旧基线14/20与用户范围正式20/20纳入不同，报告同时列原始20/21，不能把差异全部因果归于算法。只复制输出、未删除原始证据/失败/启动崩溃/旧候选。宿主进程只读检查本任务仿真/ROS/批次/防休眠均已退出，导航锁available，实际控制器ELF仍dfe4802de80de6b8ae301e08aa97204432bbf54c53d6d83dbc429c808f67ad33；无后台续跑，无无关进程操作，P4/服务器未改。
- 回滚/备份：完整candidate24基线、逐版候选源/ELF/SHA、原始全部尝试与文档/入口显示前后快照保留，交付含最小补丁和完整证据本地链接。
- 后续事项：无本轮自动任务；用户可自行启动当前手柄入口，当前验收不涵盖任意环境。

### 2026-10-19T19:22 — [EGO2P5→EGO2P6] 保存c36并开始除录像外完整复制

- 状态：进行中，完整清单/复制与新目录验证尚未完成。
- 目的：落实用户“保存当前版本，全部除记录视频外复制到EGO2P6，并说明算法项目进展与状态”。
- 项目路径：/home/starry/isaac-data/EGO2P5 → /home/starry/isaac-data/EGO2P6。
- 涉及文件：P5 SAVED_VERSION_20261019.json/WORK_LOG；HISTORY/snapshots/EGO2P5_c36_saved_20261019保存点清单及后续工作脚本；HISTORY/CHANGELOG；P6待建立。
- 修改内容：P5仅增加保存说明/本地记录，算法、场景、参数及原录像保留；完整SHA/元数据清单含所有视频，P6排除常见视频文件扩展名，保留轨迹、CSV、JSON、日志、图片、Git、构建和原始二进制重放。仅P6适配入口路径/项目标识并独立构建；历史记录身份不批量改写。
- 验证：当前目录约41GiB，初步扫描97145普通文件、137视频约6.66GB，非视频约36.70GB；新目录不存在。实际P5控制器SHA=dfe4802de80de6b8ae301e08aa97204432bbf54c53d6d83dbc429c808f67ad33，来源c36正式20/20与原始20/21沿用原报告，不算P6新飞行；本次尚未构建或飞行。
- 影响与边界：不自动提交/推送、不改P4或服务器。只跳过录像文件，不删除来源或原始重放。若为节省空间需在P6内部对相同历史只读数据去重，保留全部路径/内容/元数据并单独记录；P5与P6绝不硬链接普通文件。
- 回滚/备份：P5原目录完整保留，保存点将含全目录SHA与活动源码/实际ELF压缩包。
- 后续事项：完成完整复制校验、新目录路径与版本文档、独立构建回归及双重历史记录。

### 2026-10-19 19:32 — [EGO2P6] 完整非录像副本与独立入口适配

- 状态：原始复制与SHA完成，P6路径/身份适配完成；独立构建与回归待执行。
- 目的：建立当前c36的新开发目录，保留P5保存点。
- 项目路径：/home/starry/isaac-data/EGO2P6。
- 涉及文件：performance/copy_from_ego2p5_20261019清单/备份/适配记录；scripts、tools、src的launch及节点默认shared_robot_id、profiles、deployment/local_ubuntu、AGENTS；继承runtime归档；WORK_LOG/HISTORY。
- 修改内容：复制97009普通文件/5419目录/118链接，排除137视频6656557086字节，全部其它内容与路径初次核验完整。P6内部不可变历史同内容去重71084路径节省4404317996字节，70777路径时间戳受同inode规范化影响，原时间戳及异常逐条另存；P5/P6共享普通inode0。原runtime/local_ubuntu完整迁入runtime/imported_local_ubuntu_from_ego2p5，准备新建独立活动runtime，避免复用旧CMakeCache或P5安装前缀。只改当前运行路径/身份，benchmark保留P5及其它历史算法选择、增加P6，新默认机器人名称P6，几何/运动/风险数值不改。
- 验证：原始102546非录像路径/类型/普通文件内容SHA、权限与链接目标核对通过；源完整清单含全部视频，活动源码/实际ELF压缩保存于HISTORY/snapshots/EGO2P5_c36_saved_20261019；目标空间仍有约2.82GB，未删源数据。适配Python AST通过，未构建、未导航。
- 影响与边界：历史performance/服务器部署/备份身份不改；继承20/20是P5排除口径来源成绩，不是P6新飞行。P5仅本任务保存说明/日志追加，算法与录像未改。
- 回滚/备份：P5原目录、P6 EGO2P5_*原入口文档、adaptation_before逐文件原版与前后SHA、runtime_relocation及源全目录清单保留。
- 后续事项：独立构建/回归/参数与离线重放、P6项目算法状态说明及最终源完整性复核。

### 2026-10-19 19:52 — [EGO2P6] 当前c36非录像完整复制、算法状态说明和独立验证完成

- 状态：完成。
- 目的：完成用户保存P5当前版本、全部除录像外复制到新P6并说明代码算法项目状态的要求。
- 项目路径：/home/starry/isaac-data/EGO2P5 → /home/starry/isaac-data/EGO2P6。
- 涉及文件：P5 SAVED_VERSION_20261019.json/WORK_LOG保存记录；HISTORY/snapshots/EGO2P5_c36_saved_20261019原/最终SHA清单、Git状态、源码与实际ELF压缩包、复制工具；P6当前README/EGO2P6_VERSION/ACTIVE_NAVIGATION/MANUAL_CONTROL_PROFILE/AGENTS/CURRENT_CHECKPOINT/WORK_LOG、19项当前入口/路径/标识适配及备份、独立runtime/local_ubuntu、完整原runtime归档、performance/copy_from_ego2p5_20261019所有报告与核验；HISTORY/README/PROJECT_INDEX/FOV_GVF_FAMILY/CHANGELOG。
- 修改内容：97,009非录像普通文件/5,419目录/118链接完整复制，137录像6,656,557,086字节只跳过复制，源录像不删；代码/配置/场景/Git/日志/图片/CSV/JSON/原始二进制重放/旧构建/缓存都保留。P6不可变历史71,084路径内部同内容共用新inode，节省4,404,317,996字节，70,777路径mtime例外及各原值完整另账；活动源不去重，P5/P6共享普通inode0。原local_ubuntu完整移入imported_local_ubuntu_from_ego2p5再新建P6运行安装；只改项目身份/路径/结果目录和机器人默认标识，保留早期benchmark选择及历史身份。新文档说明混合局部避障构成、c36优化、全部保存参数、P5原始20/21与用户口径正式20/20及未过事项，明确不是P6新飞行。
- 验证：源全部97,146普通文件含137视频最终SHA复核通过，目录条目保持；仅源保存元数据和完成日志2项变化，P5实际控制器仍dfe4802de80de6b8ae301e08aa97204432bbf54c53d6d83dbc429c808f67ad33。P6原始102,546非视频路径初次核验、最终97009来源普通内容SHA/备份/归档与所有头文件和核心算法核对通过，节点仅机器人名变化；地图/占据SHA保持cfcc2269/e4e7e430。独立构建3包成功、CTest63/63、工具平台174/174、源安装参数8组、RViz4/显示选项6/真实R07 Shell截获、6真实状态×手柄/goal共12对重放全等、AST72/Bash7/CLI3通过；本次新导航/真人飞行0。首次参数检查漏加载本机环境尝试写只读~/.ros/log而未完成，保留初检并补环境复核通过；首次最终清单检查缺WORK_LOG原文单列备份而未完成，旧原文完整前缀按原SHA恢复备份后重做全量通过，失败日志均保留。
- 影响与边界：复制不新增飞行成绩，不声称稳定1ms/无卡顿/任意路线。目标算法几何/运动/预算数字不变，源P5与全部原录像保留；仅P6新增配置身份，P4/服务器不改，无提交推送或自动续跑。磁盘当前剩余约2.32GiB，后续长实验前需安排容量；未为腾空间删除来源。共享导航锁状态available，所有本次Node.execute调用0。
- 回滚/备份：P5原目录、保存点全清单与源码/4实际ELF压缩包、P6各EGO2P5_*旧文档、adaptation_before原件、旧runtime完整归档、所有原始/失败/时间戳和去重路径清单。
- 后续事项：无排队导航。用户可从P6 README运行手柄或R07双窗口入口，后续新算法需另做新飞行验证。

### 2026-10-19 20:00 — c36同步5090：独立部署准备

用户明确要求把最新c36同步至5090，覆盖先前“服务器来源保持”的默认范围。本机P6数值算法不改；5090仍为candidate24，确认Ubuntu24.04/Jazzy/RTX5090、264GB空闲且无导航进程。新建远端EGO2P6与独立runtime/rtx5090_c36，保留EGO2P4并先保存其活动源/实际产物身份。准备deployment/rtx5090_c36环境/入口/构造验证，记录performance/sync_to_rtx5090_20261019_2000。尚未上传、构建或飞行，来源20/20仍属P5本机排除口径。普通SSH因沙箱网络受限、非沙箱BatchMode缺认证未连通，已通过既有SSH控制会话成功只读核对。
