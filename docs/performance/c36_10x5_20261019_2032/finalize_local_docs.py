from pathlib import Path
import json,csv,datetime,shutil
R=Path('/home/starry/isaac-data/EGO2P6');T=R/'performance/c36_10x5_20261019_2032';H=R.parent/'HISTORY'
s=json.loads((T/'completion_summary.json').read_text());v=json.loads((T/'final_verification.json').read_text());assert s['complete'] and v['passed'] and s['formal_flights']==50
now=datetime.datetime.now().astimezone();stamp=now.strftime('%Y-%m-%d %H:%M');rows=json.loads((T/'records.json').read_text());ex=json.loads((T/'exclusions.json').read_text());failed=[x for x in rows if x['has_trajectory'] and not x['safe_arrival'] and x['label'] not in ex]
failtext='；'.join(f"{x['route'].upper()}/原始第{x['repeat']}次 {x['status']} {x['sim_s']:.3f}s" for x in failed)
B=T/'local_documentation_before_final';B.mkdir(exist_ok=True)
for n in ['README.md','EGO2P6_VERSION.md','CURRENT_CHECKPOINT.json','WORK_LOG.md']:shutil.copy2(R/n,B/n)
report='performance/c36_10x5_20261019_2032/RESULTS.md'
note=f'''{stamp} — **5090 c36十路线各5次正式实验完成：{s['formal_safe_arrivals']}/50安全到达，{s['formal_failed']}次普通失败。** 原始{s['raw_flights']}次飞行，顶界斜角围困{s['excluded']}次按用户既定范围复核排除并同路补跑，另{s['startup_attempts']}次无飞行初始化挂起留原失败重试。普通失败：{failtext}。全程避障逐次均值再平均{s['all_formal_flight_mean_avoid_ms']:.3f}ms，正式最坏{s['worst_avoid_ms']:.3f}ms；原始含排除均值{s['raw_mean_avoid_ms']:.3f}ms/最坏{s['raw_worst_avoid_ms']:.3f}ms。无画面/无RViz/无视频、同一源码与安装/参数冻结；所有原轨迹与审计留服务器，本机保留简单表和紧凑证据。本轮没有优化算法或重建；不是稳定1ms/任意路线验收。[逐次数据]({report.replace('RESULTS.md','formal_results.csv')}) · [本轮汇总]({report})。
'''
for name in ['README.md','EGO2P6_VERSION.md']:
 p=R/name;head,rest=p.read_text().split('\n',1);p.write_text(head+'\n\n'+note+'\n'+rest.lstrip('\n'))
p=R/'CURRENT_CHECKPOINT.json';j=json.loads(p.read_text());j['latest_rtx5090_experiment']=dict(updated_at=now.isoformat(),algorithm='c36',planned=50,formal_flights=50,formal_safe_arrivals=s['formal_safe_arrivals'],formal_failed=s['formal_failed'],raw_flights=s['raw_flights'],raw_attempts=s['raw_attempts'],excluded=s['excluded'],startup_attempts=s['startup_attempts'],all_samples_complete=s['all_samples_complete'],frozen_unchanged=s['all_frozen_unchanged'],mean_avoid_ms=s['all_formal_flight_mean_avoid_ms'],max_avoid_ms=s['worst_avoid_ms'],headless=True,report=report);p.write_text(json.dumps(j,ensure_ascii=False,indent=2)+'\n')
limits={k:sum(x.get('audit',{}).get('limits_violations',{}).get(k,0) for x in rows) for k in ['speed','vertical','acceleration','applied_command']}
entry=f'''### {stamp} — [EGO2P6] 5090 c36十路线各5次正式实验与简单数据记录完成

- 状态：完成，未安排后续导航。
- 目的：完成用户5090运行10组×5次并简单记录的要求。
- 项目路径：服务器/home/ubuntu5090/dhm/EGO2P6，本机/home/starry/isaac-data/EGO2P6/performance/c36_10x5_20261019_2032。
- 涉及文件：本批plan/run_batch/progress/compact_progress/summarize/final_check/finalize_local_docs、schedule/frozen_identity、全部records/state/queue/console、formal_results/all_attempts/failures_and_exclusions/results.csv、RESULTS/completion_summary/final_verification、启动诊断与两例顶界复核证据、服务器原始performance/p6_speed_20261019/rtx5090_c36/c36_10x5_*轨迹/流式样本/审计/冻结源/实际产物；本机及远端README/EGO2P6_VERSION/CURRENT_CHECKPOINT/WORK_LOG与各更新前备份；HISTORY/README/FOV_GVF_FAMILY/CHANGELOG。
- 修改内容：同一已部署c36，原R01–R10约300m/Z10五轮串行、每次新进程；10m/s请求/垂向1/a16/tau.10/delay.20/depth20/H.5/cache.3/.75/history16、默认不绑核，headless/无RViz/无视频。未改算法/数值/安装或运行中重建，未同步本机精简。完整原50槽加两次R06排除补跑，正式50条；只对经复核顶界斜角排除，普通失败不额外补成成功。数据表记录状态、sim/wall/process时间、路程/平均与峰值速度、全程避障mean/P95/max、超时帧、碰撞/扫掠/限值及源/传感器身份。
- 验证：实际{s['raw_attempts']}次尝试/{s['raw_flights']}次有轨迹飞行，原始{s['raw_safe_arrivals']}安全到达；正式50条每路线5条，{s['formal_safe_arrivals']}安全到达/{s['formal_failed']}普通失败。普通失败{failtext}。{s['excluded']}顶界例原状态不改写：R06/n02 ABORTED_EXPLORATION_STALL162.467s（末60秒静止、距既有排除位置0.509m），R06/n05 TIMEOUT600s（末60秒XYZ跨度.3315/.2430/.2135m、距既有位置.176m、净推进.00464m），同地图SHA与完整当前轨迹复核；新请求简记无新视频，参照既有同地图轨迹/录像复核工况。R03/n02及R05/n04共{s['startup_attempts']}次无起飞futex初始化挂起分别178.63/194s，只终止各自已核对Isaac进程并留原失败补跑。
- 计时与安全：正式逐次全程避障均值再平均{s['all_formal_flight_mean_avoid_ms']:.6f}ms、最坏{s['worst_avoid_ms']:.6f}ms、期限样本{s['deadline_samples']}；原始含排除均值{s['raw_mean_avoid_ms']:.6f}ms/最坏{s['raw_worst_avoid_ms']:.6f}ms。全流式样本与最终guidance数量全部一致，原始审计均保留；冻结源/实际安装/地图{v['source_install_map_files_unchanged']}项最终SHA不变，协议起终点、10/a16/tau.10/.20、帧积分及真实传感器核验通过。扫掠重叠{s['swept_overlap_segments']}、外部碰撞保护{s['collision_blocks']}、限值{json.dumps(limits)}。本轮未重跑核心/工具单元测试，沿用前次已验证安装，进行了上述实际仿真及数据核验。
- 影响与边界：正式是用户既定顶/底斜角排除范围；原始失败与两次无飞行启动异常完整另账，R02/R05等普通失败不能剔除。本机P5/P4/原录像和旧服务器P4不改。数据不代表稳定1ms、任意路线或真人手柄/实飞通过；无新算法优化、无提交推送。
- 回滚/备份：既有c36/旧P4保存点保持；本批各次内容寻址冻结源/实际ELF/协议/相机与原始轨迹计时/审计、独立排除/启动诊断、文档更新前原件和最终清单保持。
- 退出核验：初次收尾检查仍发现本批首轮创建的空闲ROS查询daemon，停止在写最终文档前；按运行ID确认后只SIGTERM自身PID2080733，留runtime_cleanup证据，再核验无本任务导航/查询进程。未触碰其它项目进程。
- 后续事项：无本任务后台导航；后续是否优化按用户新指令执行。

'''
p=H/'CHANGELOG.md';p.write_text(entry+p.read_text())
with (R/'WORK_LOG.md').open('a') as f:f.write('\n'+entry)
headline=note.replace('(performance/','(../EGO2P6/performance/')
for name in ['README.md','FOV_GVF_FAMILY.md']:
 p=H/name;head,rest=p.read_text().split('\n',1);p.write_text(head+'\n\n'+headline+'\n'+rest.lstrip('\n'))
(T/'final_local_record.json').write_text(json.dumps(dict(updated_at=now.isoformat(),status='complete',formal=s['formal_flights'],arrived=s['formal_safe_arrivals'],failed=s['formal_failed'],raw_flights=s['raw_flights'],raw_attempts=s['raw_attempts'],limits=limits,report=report),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(formal=50,arrived=s['formal_safe_arrivals'],failed=s['formal_failed'],raw=s['raw_flights'],report=str(T/'RESULTS.md')),ensure_ascii=False))
