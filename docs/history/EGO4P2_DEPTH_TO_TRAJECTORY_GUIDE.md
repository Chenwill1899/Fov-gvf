# EGO4P2 入门讲解：从四张深度图到三维飞行参考

**读者**　第一次接触机器人感知、路径规划或控制的同学。可以先读每节开头和例子，再看公式与源码。

**范围**　以 2026 年 11 月 9 日核对的 EGO4P2 源码为准，覆盖四路深度输入、球面调和场、角域曲线、三维空间参考、净空检查、固定参考跟踪和滚动续证。第 9 节对照 FLARE 论文，第 10 节说明模型与 Isaac Sim 已验证的范围。核心实现位于 `candidates/ldlt_parallel/new_controller`，由 `src/new_controller` 的 `navigation_input` 接入。 [构建接线](../../src/new_controller/CMakeLists.txt#L21) · [当前项目检查点](../../../EGO4P3/docs/CURRENT_CHECKPOINT.json)

## 1. 先理解机器人要回答的三个问题

设无人机正在向东飞，操作者希望它转向北方，前方有一面墙。

1. **附近往哪些方向看过去有足够证据？** 四张深度图分别告诉系统视野里首先看到了什么。把信息投到一个以机体为中心的单位球面上，得到可以用于规划的方向区域。
2. **应该怎样连续改变运动方向？** 在这个区域里给每个方向格分配一个势值，让势值从低处过渡到出口。沿势值上升的方向逐小步前进，得到从当前航向出发的方向曲线 \(Q\)。
3. **实际飞行的空间位置和下一步速度是什么？** 给 \(Q\) 安排每米可以转多少角度，把方向当成曲线的切向，再累加成三维位置。用原始深度图检查这条曲线附近的连续空间，最后依据实际运动状态计算速度命令候选。

```text
四张深度图 + 标定 + 观测位姿          实际状态 + 期望方向
                │                              │
                ▼                              │
       图像补值、摘要、虚拟环视                  │
                │                              │
                ▼                              ▼
       球面可用方向域 Ω ────────► 粗场、细场、方向曲线 Q
                                              │
                                              ▼
                                  曲率安排 + 空间积分
                                              │
                                              ▼
                             三维参考 γ + 原图连续净空检查
                                              │
                                              ▼
                                跟踪、滚动续证、命令候选
```

这里有三个容易混淆的对象。**势** \(\phi\) 是每个方向上的一个数，类似等高线图上的高度。**方向场** \(g\) 是每个方向上的一支箭头，由势的梯度算出。**空间参考** \(\gamma\) 是一串以米为单位、可连续查询的位置。后面会依次说明这三次转换。

### 阅读公式所需的约定

| 符号 | 含义 |
| --- | --- |
| \(C,B,W\) | 相机坐标系、机体坐标系、世界坐标系。\(B_o\) 专指拍摄这组深度图时的机体坐标系。 |
| \(R_{BC},R_o\) | 分别把相机方向转到机体系、把观测时的机体方向转到世界系。方向只旋转，点的位置还要加平移。 |
| \(\lVert a\rVert,\ a^\top b,\ a\times b\) | 向量的长度、点积、叉积。两个单位向量的点积等于夹角的余弦，叉积给出垂直于二者的旋转轴方向。 |
| \(S^2,\Omega\) | \(S^2\) 是三维空间中的单位球面。 \(\Omega\) 是其中有足够观测证据、允许参与本次规划的方向格集合。 |
| \(\zeta,\ s,\ t\) | 累计转角、沿空间参考走过的距离、时间。单位分别为弧度、米、秒。 |
| \(V,L,r_{\rm env}\) | 正常跟踪目标速度 \(12\,\mathrm{m/s}\)，局部观测范围 \(4\,\mathrm m\)，占用包络半径 \(0.30\,\mathrm m\)。 |

公式用弧度进行三角计算。网格参数和讲解示例也用度表示角度，以便直观理解。 [基础参数](../../src/new_controller/ldlt/include/new_controller/base_config.hpp#L5) · [导航参数](../../src/new_controller/ldlt/include/new_controller/navigation_config.hpp#L5)

## 2. 四路深度图怎样成为可计算的方向数据

### 2.1 深度图、时间戳和相机标定

普通照片给每个像素记录颜色，**深度图**给像素记录距离。本项目输入的 \(Z\) 是沿相机光轴的深度，单位是米。同一个 \(Z\) 下，画面边缘的物体沿视线量过去会比画面中心更远。

四路图像按相同的仿真时间戳组成一组，并等待该时刻的里程计位姿。**位姿**包含位置和朝向。程序还单独读取当前实际运动状态，避免用拍照时的速度代替现在的速度。图像身份包含组号、相机时间戳和标定版本。配置版本、当前仿真时间、参考版本及失效原因也通过统一接口传递。缺相机、时间戳不一致、缺位姿、标定或配置不匹配、数据来自未来或过期时，输入会被拒绝。当前观测和实际状态的龄期上限为 10 ms 仿真时间。 [四路接收与对时](../../src/new_controller/src/navigation_input.cpp#L24) · [输入身份与校验](../../src/new_controller/ldlt/include/new_controller/navigation_pipeline.hpp#L69)

### 2.2 把像素变成射线

**内参**矩阵 \(K_c\) 描述第 \(c\) 台相机的焦距、图像中心等成像参数。**外参**描述相机装在机体哪里、朝向哪里。对像素 \((u,v)\)，令

$$
b_c=K_c^{-1}[u,v,1]^\top,\qquad
P_C=Zb_c,\qquad
\rho=Z\|b_c\|,\qquad
d_B=\frac{R_{BC}b_c}{\|b_c\|}.
$$

\(P_C\) 是相机系中的三维点，\(\rho\) 是沿该射线的距离，\(d_B\) 是长度为 1 的机体系方向。举例来说，如果 \(b_c=(0.5,0,1)\) 且 \(Z=2\,\mathrm m\)，则 \(P_C=(1,0,2)\,\mathrm m\)，而射线距离为 \(\sqrt5\,\mathrm m\)。

求场使用**虚拟共心环视**。相机朝向保留，方向查表把四台相机的光心近似放在机体中心，忽略安装平移。用方位角 \(\alpha\) 表示水平绕转，用仰角 \(\beta\) 表示上下抬俯，则球面方向为

$$
d_B(\alpha,\beta)=
(\cos\beta\cos\alpha,\ \cos\beta\sin\alpha,\ \sin\beta).
$$

代码预计算射线系数，并从球面格中心反查它落进哪些相机的哪些像素块。后面检查空间管道时会改用真实相机位置和朝向。 [射线查表](../../src/new_controller/ldlt/include/new_controller/depth_preprocess.hpp#L49) · [虚拟环视查表](../../src/new_controller/ldlt/include/new_controller/depth_virtual_panorama.hpp#L45)

### 2.3 原始图、补值图和两级摘要

对异常像素，包括 NaN、无穷、零和负值，取**原始图** \(3\times3\) 邻域内有限正值的均值。令 \(\mathcal V_p\) 是像素 \(p\) 的该邻域中有效的原始像素集合，仅当 \(p\) 异常且 \(\mathcal V_p\) 非空时计算 \(Z_{\rm filled}(p)=\sum_{j\in\mathcal V_p} Z_{\rm raw}(j)/|\mathcal V_p|\)。邻域没有有效值时保留原来的异常值。有效像素不改动，补出的数不会继续传给下一处异常像素。

每台相机有一个常驻工作线程，分别完成补值、块摘要、全分辨率查询摘要和本相机的方向数据。一级把图像分成 \(4\times4\) 像素块，二级再合并相邻 \(3\times3\) 个一级块。边缘可以出现不足整块的区域。每级保留近障碍判定需要的最小射线距离和原始有效像素数。二级的补值深度均值按一级有效像素数加权，空块不参加均值或最小值：

$$
\bar Z_2=\frac{\sum_{j:\,n_j>0} n_j\bar Z_j}{\sum_j n_j},
\qquad
\rho_{\min,2}=\min_{j:\,n_j>0}\rho_{\min,j}.
$$

这里的 \(n_j\) 是补值后仍为有限正值的像素数。原始有效数、原始最小距离和总像素数另行保存。原始图与补值图也各自有区域最小值索引，后面的管道检查仍能在全分辨率图像上问“这个矩形里的原始像素是否全部有效，最浅的点在哪里”。四路工作结束后，协调线程合并方向域，并运行后续求场与曲线阶段。 [邻域补值](../../src/new_controller/ldlt/include/new_controller/depth_preprocess.hpp#L19) · [两级摘要和区域查询](../../src/new_controller/ldlt/include/new_controller/depth_observations.hpp#L204)

## 3. 障碍如何在方向球面上挡住一片区域

### 3.1 障碍膨胀和三种状态

把无人机近似成半径 \(r_{\rm body}=0.10\,\mathrm m\) 的球，再预留 \(0.20\,\mathrm m\) 的跟踪误差预算，得到统一包络 \(r_{\rm env}=0.30\,\mathrm m\)。**障碍膨胀**可以理解为给障碍画一圈安全边。这样即便机体中心没有撞到物体，机体和预留余量可能触及的方向也会被排除。

从观察中心看，距离 \(\rho>r_{\rm env}\) 的点周围画一个半径为 \(r_{\rm env}\) 的球，球的视角半径为 \(\arcsin(r_{\rm env}/\rho)\)。代码以 5 cm 为单位分档，在对应档的下界 \(\rho_{\rm lo}\) 上计算：

$$
\delta_{\rm base}=
\begin{cases}
\pi,&\rho_{\rm lo}\le r_{\rm env},\\
\arcsin(r_{\rm env}/\rho_{\rm lo}),&\rho_{\rm lo}>r_{\rm env}.
\end{cases}
$$

取更近的档下界会扩大排除范围。查表还加入网格量化余量，近障碍排除另加像素块最大角半径，用保守角窗口标记。参与此排除的近块满足补值后的最小射线距离 \(\rho_{\min}\le L=4\,\mathrm m\)。 [近块判定](../../src/new_controller/ldlt/include/new_controller/depth_local_field.hpp#L20) · [分档与角窗口](../../src/new_controller/ldlt/include/new_controller/depth_virtual_panorama.hpp#L60)

四路合并时，每格保留所有覆盖相机的块中最小的有效 \(\rho_{\min}\)。一格有**原始完整覆盖**，表示至少一路相机在该格对应的块内原始像素全部有效。随后逐格判定：

| 状态 | 程序含义 |
| --- | --- |
| `Unavailable` | 有深度观测，且该方向被近障碍的膨胀区域排除。 |
| `Available` | 中心有观测且未被排除。查询窗口内每一格都有原始完整覆盖，融合最小射线距离严格大于 \(L+r_{\rm env}=4.30\,\mathrm m\)。 |
| `Unknown` | 剩余情况。包括看不到、原始覆盖不完整、深度证据不足等。 |

\(\Omega\) 只收录 `Available`。虚拟环视适合快速筛方向，米制参考还要单独接受真实相机的原图验证。方向筛选和空间检查分别使用同一个 \(0.30\,\mathrm m\) 包络定义，后者直接查原图，未在已经膨胀的 \(\Omega\) 上再膨胀一次。 [四路合并和三态判定](../../src/new_controller/ldlt/include/new_controller/depth_local_field.hpp#L80)

### 3.2 为什么叫二维方向域

球面上的点虽然有三个坐标，但它满足 \(x^2+y^2+z^2=1\)，两个角就能定位，因此球面是**二维曲面**。所谓二维调和场，是在这个曲面上分配势值，仍然能表达向上爬升、向下俯冲等三维方向。

细网格是 \(1^\circ\times1^\circ\)，粗网格以 \(3^\circ\times3^\circ\) 格为基础。它们的单位是角度。上节的 \(4\times4\) 和再 \(3\times3\) 是图像像素块尺度。这两种网格在代码中分别配置。 [球面网格](../../src/new_controller/ldlt/include/new_controller/harmonic_solver.hpp#L8) · [粗格观测来源](../../src/new_controller/ldlt/include/new_controller/harmonic_planner.hpp#L134)

## 4. 二维球面上怎样求粗场和细场

### 4.1 调和势和边界条件

想象有一张弯曲的薄膜。某一侧固定为低势 0，出口固定为高势 1，中间的值根据相邻格的影响自行平衡。在本项目中这个数叫**调和势**。在连续数学模型中，内部满足

$$
\Delta_{S^2}\phi
=\frac{1}{\cos^2\beta}\frac{\partial^2\phi}{\partial\alpha^2}
+\frac{1}{\cos\beta}\frac{\partial}{\partial\beta}
\left(\cos\beta\,\frac{\partial\phi}{\partial\beta}\right)=0.
$$

\(\Delta_{S^2}\) 叫 **Laplace–Beltrami 算子**，可以理解为“把平面上的局部平衡规则改写到曲面上”。\(\cos\beta\) 出现在式中，因为越靠近极点，同样的方位角差对应的球面距离越短。

**Dirichlet 边界**直接固定势值，例如低势 0 和出口 1。**零 Neumann 边界**规定穿过该边界的法向通量为零。程序对障碍、未知以及当前求解域外的面不加连接边。这个边界规则约束求场，后续积分和占用检查还会各自验证是否越界。 [球面权重](../../src/new_controller/ldlt/include/new_controller/harmonic_solver.hpp#L14) · [域外面处理](../../src/new_controller/ldlt/include/new_controller/grounded_system.hpp#L19)

### 4.2 起点、连通分量、出口

常规起点 \(q_0\) 来自实际运动方向。设 \(v_W\) 是当前世界系速度，则

$$
q_0=\frac{R_o^\top v_W}{\|v_W\|}.
$$

速度无法给出有效方向时，先尝试上次有效世界速度，再使用初始期望方向。滚动更换参考时，连接点的旧参考切向覆盖这一常规选择。这里用 \(R_o^\top\) 是因为方向域绑定在拍照时的机体坐标系。

程序先确认 \(q_0\) 所在格可用，然后沿四邻格搜索。**连通分量**就是从起点逐格走能到达的所有可用格。若期望方向所在格属于该分量，直接选它为出口中心。否则在该分量中选格中心与期望方向点积最大的格，并记录出口被调整。粗层低势中心选在分量内与已选出口中心夹角最大的一侧。粗场高、低势帽的角半径取配置的 \(3^\circ\) 和两中心角距的 \(0.24\) 倍中的较小值，高势标签优先。低势位置和 \(q_0\) 分别选取。 [起点来源](../../src/new_controller/ldlt/include/new_controller/navigation_pipeline.hpp#L102) · [连通域与出口](../../src/new_controller/ldlt/include/new_controller/harmonic_planner.hpp#L51) · [边界帽](../../src/new_controller/ldlt/include/new_controller/harmonic_adaptive.hpp#L32)

### 4.3 粗细两层各自做什么

- **粗层**看整个起点连通分量。一个 \(3^\circ\) 粗格只有在二级图像摘要支持粗格可用、九个 \(1^\circ\) 子格都在同一连通分量且标签一致时才合并。其他粗格保留其中属于该分量的细节点。聚合格之间的权重由穿过聚合边界的细面权重相加。
- **细层**只在局部通道内求解。程序从出口沿搜索的父节点链回溯到起点，形成一条格路径，默认向可用邻格扩三层。默认分支继承粗图的高势出口标签，独立选一个远侧低势格，通道侧边为零通量。如果通道全是出口标签，退化分支从非起点格中选低势格。符合条件的人工通道边界阻挡可触发扩通道，总共最多三次细场尝试。
- **可选耦合分支**在人工通道截断处使用所属粗节点的势值作为边界。活动默认配置是局部零通量分支，粗场通过可达域和出口标签影响细场。

这样的划分能在大范围了解方向连通性，同时限制细场求解的节点数。 [粗格聚合](../../src/new_controller/ldlt/include/new_controller/harmonic_adaptive.hpp#L41) · [局部通道与重试](../../src/new_controller/ldlt/include/new_controller/harmonic_planner.hpp#L70) · [细场边界分支](../../src/new_controller/ldlt/include/new_controller/harmonic_planner.hpp#L162)

### 4.4 计算机怎样把“平衡”算出来

**有限体积法**把球面分成小格，令一个内部格流出各面的总通量为零。方位面的权重用格中心仰角 \(\beta_i\)，仰角面的权重用共用边所在的仰角 \(\beta_{\rm face}\)：

$$
w_\alpha=\frac{\Delta\beta}{\cos\beta_i\,\Delta\alpha},
\qquad
w_\beta=\frac{\cos\beta_{\rm face}\,\Delta\alpha}{\Delta\beta},
\qquad
\sum_{j\in N(i)}w_{ij}(\phi_i-\phi_j)=0.
$$

\(N(i)\) 是和格 \(i\) 通过有效面相连的域内邻格。方位角跨 \(-180^\circ/180^\circ\) 周期相连。上下极点没有跨极面的邻格。把已知边界势值移到等号右边，得到

$$
A\phi=b,\qquad
A_{ii}=\sum_{j\in N(i)}w_{ij},\qquad
A_{ij}=-w_{ij}\ \text{（相连的未知格）},\qquad
b_i=\sum_{j\in N(i)\cap D}w_{ij}\phi_j,
$$

其中 \(D\) 是固定势值的格集合，同一对节点的多条边权合并。矩阵只在邻格间有非零项，因此称为**稀疏矩阵**。粗、细两层用 Eigen 的稀疏 LDLT 求解，LDLT 是把适用的对称矩阵分解成三角矩阵和对角矩阵来解线性方程的方法。源码还核对解的有限性和势值范围，并要求

$$
\max_{i\ {\rm 为未知格}}\frac{|b_i-(A\phi)_i|}{A_{ii}}\le 10^{-7}.
$$

这个**残差**衡量代回离散方程后还差多少。严格缓存会比对边界标签和值、求解容差及图的相关连接和权重，命中后直接复用已经验证的解。两层各有这样的复用路径。 [离散权重](../../src/new_controller/ldlt/include/new_controller/harmonic_solver.hpp#L14) · [稀疏装配](../../src/new_controller/ldlt/include/new_controller/grounded_system.hpp#L14) · [LDLT 后端](../../src/new_controller/ldlt/src/sparse_trial_backend.cpp) · [严格缓存与残差](../../src/new_controller/ldlt/include/new_controller/trial_field_solver.hpp#L78)

## 5. 从势值求箭头，再沿箭头走成方向曲线

### 5.1 梯度是从标量势得到的方向场

求解输出是格上的数字。为了知道格内任意方向上该朝哪边走，代码用周围有效格估计角点处的势值和导数，再做二维 **Hermite 插值**。这种插值用端点的值和斜率构造平滑函数。普通区域对得到的 \(\tilde\phi\) 求球面梯度：

$$
g(d)=\nabla_{S^2}\tilde\phi(d)
=\frac{\partial_\alpha\tilde\phi}{\cos\beta}e_\alpha
+\partial_\beta\tilde\phi\,e_\beta,\qquad
\hat g(d)=\frac{g(d)}{\|g(d)\|}.
$$

\(e_\alpha,e_\beta\) 是沿方位和仰角的单位切向。\(g(d)\) 垂直于球面的半径 \(d\)，所以它指示“往哪里转”，方向取势值上升的一侧。只有梯度模长大于 \(10^{-7}\) 时才归一化。这里“调和”与“谐波”都来自英文 `harmonic`。代码做的是从标量势求梯度，粗细两次求解属于空间层级。离散残差检验网格解，插值为积分提供连续查询。

靠近极点时，\(\cos\beta\) 很小。代码在 \(|\beta|>84^\circ\) 时从完整极冠拟合仿射势 \(\phi_{\rm cap}=a_0+a^\top(d-d_{\rm pole})\)，其梯度在球面切平面上的投影为 \(g_{\rm cap}=(I-dd^\top)a\)。在 \(84^\circ\) 到 \(86^\circ\) 之间用 \(w=3u^2-2u^3\) 混合两种势，\(u\) 是这两度范围内的归一化仰角绝对值。对混合势求导时也包含 \(w\) 的导数：

$$
g=(1-w)g_{\rm H}+w\,g_{\rm cap}
  +\partial_\beta w\,(\phi_{\rm cap}-\phi_{\rm H})e_\beta.
$$

极冠不完整或拟合失败时，这个方向的查询失败。 [Hermite 插值和极区分支](../../src/new_controller/ldlt/include/new_controller/harmonic_direction_field.hpp#L10)

### 5.2 球面中点积分

**积分曲线**是沿方向场的箭头一路走出的线。从 \(q_0\) 出发，在球面上的积分目标是

$$
\frac{\mathrm dQ}{\mathrm d\zeta}=\hat g(Q),
\qquad Q(0)=q_0.
$$

程序以**球面中点法**近似这个方程。对单位方向 \(q\) 及其单位切向 \(v\)，\(\operatorname{Exp}_q(hv)=q\cos h+v\sin h\) 表示沿大圆走角度 \(h\)。先走半步估计中点，再把中点梯度投回起点切平面、归一化并走全步：

$$
\begin{aligned}
q_m&=\operatorname{Exp}_{q_k}\bigl(\tfrac h2\hat g(q_k)\bigr),\\
v_m&=\frac{(I-q_kq_k^\top)\hat g(q_m)}
           {\|(I-q_kq_k^\top)\hat g(q_m)\|},\\
q_{k+1}&=\operatorname{Exp}_{q_k}(h v_m).
\end{aligned}
$$

初始尝试步长最多 \(0.8^\circ\)。程序检查起点到中点、中点到终点、起点到终点三个球面短弧所穿过的格，同时检查势值 \(\tilde\phi(q_{k+1})>\tilde\phi(q_k)-10^{-12}\) 与最小角进度。失败会尝试把步长减半。累计转角用每个成功短弧的角长度相加，单段用 \(\operatorname{atan2}(\|q_k\times q_{k+1}\|,q_k^\top q_{k+1})\) 计算。只有落入出口标记格才接收曲线。弱梯度、未知、障碍或其他失败都可能使曲线提前停止。起点已在出口格时，也可能得到零转角的有效结果。

回到向东飞的例子，\(Q\) 现在是一列方向，如“向东、略偏东北、继续向北”。这些点在单位球面上，还没有告诉无人机应该在离墙多远的位置开始转弯。 [球面积分与接受条件](../../src/new_controller/ldlt/include/new_controller/harmonic_curve.hpp#L84) · [角长度计算](../../src/new_controller/ldlt/include/new_controller/harmonic_curve_evidence.hpp#L75)

## 6. 怎样把方向曲线变成三维空间参考

### 6.1 先把离散方向接成连续曲线

把刚才的离散 \(Q\) 用三次 Hermite 段连接。代码用相邻方向估计公共节点处的单位切向 \(t_i\)。一段的两个端点方向 \(q_i,q_{i+1}\) 相距球面角 \(h_i\)，对应四个 Bézier 控制点：

$$
(b_0,b_1,b_2,b_3)=
\bigl(q_i,\ q_i+\tfrac{h_i}{3}t_i,\
q_{i+1}-\tfrac{h_i}{3}t_{i+1},\ q_{i+1}\bigr).
$$

**Bézier 控制点**像用来牵引曲线形状的手柄。中间两点让曲线以指定切向离开起点、进入终点。对段内参数 \(u\in[0,1]\)，先计算三次曲线 \(B(u)\)，再把它归一化到球面：

$$
B(u)=(1-u)^3 b_0+3u(1-u)^2b_1+3u^2(1-u)b_2+u^3b_3,
\qquad q(u)=\frac{B(u)}{\|B(u)\|}.
$$

代码数值积分 \(\int_0^u\|q'(\xi)\|\,\mathrm d\xi\)，加上前面各段的角长度，再反查给定累计角 \(\zeta\) 对应的 \(u\)。把这个连续版本记为 \(\widehat Q(\zeta)\)。按球面弧长定义，非零长度段满足 \(\|\widehat Q'(\zeta)\|=1\)，实现使用数值求积和带误差界的查询。过近的重复点会跳过，反向歧义、尖点和不满足正则条件的插值会被拒绝。

插值段可能走过原积分短弧以外的方向，所以程序对平滑后的**整段方向曲线**重新检查 \(\Omega\) 覆盖。它用归一化三次曲线的控制点限定每段所在的方向范围，必要时细分，再检查范围覆盖的格。 [连续方向曲线](../../src/new_controller/ldlt/include/new_controller/spatial_reference.hpp#L23) · [连续角域检查](../../src/new_controller/ldlt/include/new_controller/reference_occupancy.hpp#L20) · [检查接线](../../src/new_controller/ldlt/include/new_controller/navigation_pipeline.hpp#L172)

### 6.2 曲率规定“每走一米转多少角度”

**切向**是曲线上当前位置的前进方向。**曲率**是沿曲线每走一米，单位切向改变多少。以恒速 \(V\) 沿半径 \(R\) 的圆弧运动时，曲率为 \(1/R\)，法向加速度为 \(V^2/R\)。本项目用 \(80\,\mathrm{m/s^2}\) 作为参考曲线的转向设计值，因此

$$
\kappa(s)=\frac{\mathrm d\zeta}{\mathrm ds},\qquad
\kappa_{\max}=\frac{80}{V^2}=\frac59\,\mathrm{m^{-1}},
\qquad
R_{\min}=\frac{1}{\kappa_{\max}}=1.80\,\mathrm m.
$$

以 \(V=12\,\mathrm{m/s}\) 和 \(0.10\,\mathrm s\) 的渐变时间设计满幅渐变段，得到 \(\ell_r=1.20\,\mathrm m\)。设 \(\zeta_e\) 为**平滑后**的累计球面转角，\(a\) 为单侧渐变段长度，\(b\) 为恒曲率段长度。代码使用的曲率轮廓，从零线性上升，在需要时保持峰值，再线性回到零。对于正转角，轮廓可以写成

$$
\begin{gathered}
a=\min\!\left(\ell_r,\sqrt{\frac{\zeta_e\ell_r}{\kappa_{\max}}}\right),
\qquad
\kappa_p=\kappa_{\max}\frac{a}{\ell_r},
\qquad
b=\max(0,\zeta_e/\kappa_p-a),\\[3pt]
\kappa(s)=
\begin{cases}
(\kappa_p/a)s,&0\le s<a,\\
\kappa_p,&a\le s<a+b,\\
(\kappa_p/a)(2a+b-s),&a+b\le s<2a+b,\\
0,&s\ge 2a+b.
\end{cases}
\end{gathered}
$$

曲率轮廓下的面积就是累计转角 \(\zeta_e\)。因此转向段长度为

$$
L_{\rm turn}=2a+b=
\begin{cases}
2\sqrt{\zeta_e\ell_r/\kappa_{\max}},&
\zeta_e<\kappa_{\max}\ell_r,\\
\zeta_e/\kappa_{\max}+\ell_r,&
\zeta_e\ge\kappa_{\max}\ell_r.
\end{cases}
$$

小转角只有两个较短的渐变段，峰值低于 \(\kappa_{\max}\)。代码将小于 \(10^{-12}\) rad 的转角视作零长度转向段。举例来说，若 \(\widehat Q\) 的累计转角为 \(90^\circ\)，则 \(L_{\rm turn}\approx4.027\,\mathrm m\)，以 \(12\,\mathrm{m/s}\) 匀速走完约需 \(0.336\,\mathrm s\)。如果方向曲线先上抬再回落，累计转角可能比起终方向之间的夹角更大，公式用的是前者。 [曲率轮廓](../../src/new_controller/ldlt/include/new_controller/spatial_reference.hpp#L161) · [设计参数](../../src/new_controller/ldlt/include/new_controller/navigation_config.hpp#L5)

### 6.3 把切向积分成世界系位置

沿参考走到弧长 \(s\) 时，先算已走过的方向角，再把该方向旋转到世界系：

$$
\begin{aligned}
\zeta(s)&=\int_0^s \kappa(u)\,\mathrm du,\\
T_W(s)&=R_o\widehat Q(\zeta(s)),\\
\gamma_W(s)&=p_a+\int_0^s T_W(u)\,\mathrm du,\\
\mathbf K_W(s)&=\frac{\mathrm dT_W}{\mathrm ds}
=R_o\widehat Q'(\zeta(s))\,\kappa(s).
\end{aligned}
$$

\(p_a\) 是空间锚点，第一次通常取当前位置，切换时可取旧参考的连接位置。\(\mathbf K_W\) 是**曲率向量**，同时说明转弯有多急、向哪边转。位置、切向和曲率来自同一套连续几何。位置用数值积分近似。代码建立 \(0.05\,\mathrm m\) 的位置表，另外插入曲率分段连接点，每个区间用 16 点复合中点法积分，并保存位置误差界。查表点之间仍可查询。

转完后沿末端切向接直线，正常空间参考没有强制末端停车。**\(4\,\mathrm m\) 是观测证据范围，\(L_{\rm turn}\) 是转向段弧长。** 它们分别回答“能检查到哪里”和“完成这次转向要走多少米”。\(\gamma_W(Vt)\) 可用于演示从锚点开始的名义恒速时间轨迹，实际运行的 \(s\) 由状态反馈和投影更新。 [位置表与连接点](../../src/new_controller/ldlt/include/new_controller/spatial_reference.hpp#L183) · [位置、直线尾段与曲率查询](../../src/new_controller/ldlt/include/new_controller/spatial_reference.hpp#L228) · [积分和误差界](../../src/new_controller/ldlt/include/new_controller/spatial_reference.hpp#L296)

## 7. 怎样用原始深度验证空间管道

### 7.1 检查整小段曲线，而非仅检查端点

一条细线够宽并不代表整架无人机能过去。这里的**占用管道**指以参考曲线为中心、半径 \(r_{\rm env}\) 的空间区域。对一小段弧长 \([s_0,s_1]\)，令 \(h=s_1-s_0\)，端点位置为 \(p_0,p_1\)。代码用一个球包住这段连续管道：

$$
c_{\rm cover}=\frac{p_0+p_1}{2},\qquad
r_{\rm cover}
=r_{\rm env}+\frac{\|p_1-p_0\|}{2}
+\frac{K_{\rm bound}h^2}{8}+\epsilon_{\rm pos}.
$$

弦的一半长度包住整条端点连线，\(K_{\rm bound}h^2/8\) 包住曲线偏离弦的距离，\(\epsilon_{\rm pos}\) 取两端位置误差界的较大值再加舍入余量。实现令 \(K_{\rm bound}\) 为 \(\kappa_{\max}\) 乘参考旋转的范数上界。在计入累计角长误差后已确认进入直线尾段时，弯曲项取零。球通常比精确管道更大，可能拒绝一些实际可以通过的窄空间。 [连续覆盖球](../../src/new_controller/ldlt/include/new_controller/reference_occupancy.hpp#L87)

### 7.2 将覆盖球投回真实相机

先把球心从世界系变换到观测机体系，再依据每台相机的真实外参转到相机系。球心变换可写作

$$
c_B=R_o^\top(c_{\rm cover}-p_o),\qquad
c_C=R_{BC}^\top(c_B-t_{BC}),
$$

其中 \(p_o\) 是观测时的位置，\(t_{BC}\) 是相机光心在机体中的安装位置。代码对变换中的数值范数留余量。把机体系球半径记作 \(r_B\)，若 \(\|c_B\|+r_B>4\,\mathrm m\)，整球没有落在有效观测距离内，直接返回超范围。

投影得到一个保守的 **ROI**，即图像上需要读取的矩形像素区域。设变换到相机系后的半径为 \(r_C\)，球的光轴深度范围是 \([c_{C,z}-r_C,c_{C,z}+r_C]\)，把两端记作 \(Z_{\rm ball,near}\) 和 \(Z_{\rm ball,far}\)。球触及或跨过相机光心所在的深度平面，即 \(c_{C,z}\le r_C\) 时，不做这个投影。

投影边界取相机视线与球相切处的像素坐标。对图像的一个坐标轴，令 \(a\) 为相机内参该行前两个系数，\(k_0\) 为该行主点偏移，\(m=a^\top(c_{C,x},c_{C,y})^\top\)，\(D=c_{C,z}^2-r_C^2\)。该轴的两端是

$$
\mathrm{pixel}_{\pm}=k_0+
\frac{m c_{C,z}\ \pm\ r_C\sqrt{\max(0,m^2+\|a\|^2D)}}{D}.
$$

横、纵轴各算一次，向外取整成包住投影的像素窗口。落在画面外的部分被裁掉，同时记录投影是否完整。

- **净空证据。** 至少一路相机必须完整看见这个球的投影。该相机的**近裁面**，即它能提供证据的最近光轴深度，必须已知且为有限正数，球的近端须位于近裁面或更远处。ROI 内原始深度须全部有效，且满足 \(\min_{\rm ROI}Z_{\rm raw}>Z_{\rm ball,far}\)。
- **疑障碍否决。** 任何相机只要成功投影，包括部分落在画面内的相机，若补值 ROI 的最小深度 \(\min_{\rm ROI}Z_{\rm filled}\le Z_{\rm ball,far}\)，就优先返回疑似障碍。
- **未知。** 没有否决、也没有任何一路满足完整净空条件时，结果为未知。两台相机各看见一部分，不能在这里拼成完整净空证明。

因此空间参考构造成功、某一小段有净空、以及下一步运动已通过检查，是三个不同状态。全管道诊断按计算出的观测检查范围逐段输出状态。滚动执行只检查当下需要的局部前缀。 [真实相机球投影](../../src/new_controller/ldlt/include/new_controller/depth_local_geometry.hpp#L98) · [原图净空与否决规则](../../src/new_controller/ldlt/include/new_controller/reference_occupancy.hpp#L58) · [全管道诊断接线](../../src/new_controller/ldlt/include/new_controller/navigation_pipeline.hpp#L184)

## 8. 从空间参考到速度候选

### 8.1 外层跟踪：定位自己，并在维持目标速率时纠偏

**跟踪**指根据当前位置和速度，不断修正运动以贴近参考。正常首次接入要求实际速度模长距 \(12\,\mathrm{m/s}\) 不超过 \(0.001\,\mathrm{m/s}\)，相对参考的接入位置偏差不超过 \(0.10\,\mathrm m\)，方向偏差不超过 \(5^\circ\)。起步加速和异常停车属于另外的执行问题。 [跟踪接入](../../src/new_controller/ldlt/include/new_controller/fixed_reference_tracker.hpp#L34) · [几何接入检查](../../src/new_controller/ldlt/include/new_controller/spatial_reference.hpp#L280)

外层从上一弧长 \(s_{\rm prev}\) 附近找位置，窗口为 \([\max(0,s_{\rm prev}-0.05),\,\min(s_{\rm cert},s_{\rm prev}+0.25)]\)，单位为米。\(s_{\rm cert}\) 是证据允许的最远弧长。先找最接近实际位置 \(x\) 的查表线段，再按连续曲线最多做一次 Newton 修正。Newton 修正用当前残差和斜率估计更好的根。令 \(r=x-\gamma_W(s)\)，局部最近点应满足 \(r\cdot T=0\)，于是修正为

$$
s_{\rm new}=\operatorname{clamp}_{\rm window}\!
\left(s+\frac{r\cdot T}{1-r\cdot\mathbf K}\right).
$$

代码检查分母、修正后的正交残差和横向误差。这里的“横向”是垂直于三维切向的平面，可能同时含左右和上下偏差。设 \(I\) 是三维单位矩阵，则

$$
e_\perp=(I-TT^\top)r,\qquad
w=-k_e e_\perp,\qquad
v_d=\sqrt{V^2-\|w\|^2}\,T+w,\qquad
k_e=2\,\mathrm{s^{-1}}.
$$

\(w\) 把无人机拉回参考，且与单位切向 \(T\) 垂直，所以这两个分量合起来的目标速率为 \(V\)。程序要求横向误差加位置误差界不超过 \(0.20\,\mathrm m\)。外层更新由累计实际物理时间触发，目标周期为 5 ms。 [外层更新](../../src/new_controller/ldlt/include/new_controller/fixed_reference_tracker.hpp#L66) · [投影与纠偏公式](../../src/new_controller/ldlt/include/new_controller/fixed_reference_tracker.hpp#L146)

### 8.2 内层控制：平滑地转到新的速度方向

令 \(d=v_d/V\) 为期望运动方向，\(h=v_k/\|v_k\|\) 为当前速度方向，\(\dot d\) 是程序对外层跟踪式做解析求导得到的方向变化率。内部最近点满足投影条件时，求导使用 \(\dot s=(v_k\cdot T)/(1-r\cdot\mathbf K)\) 和 \(\dot T=\mathbf K\dot s\)。当投影卡在参考起点且实际位置仍在起点后方时，代码令 \(\dot s=0\)。

对应的求导链如下。记 \(V_\parallel=\sqrt{V^2-\|w\|^2}\) 为前一节的沿切向速率：

$$
\begin{aligned}
\dot r&=v_k-T\dot s,\\
\dot e_\perp&=\dot r-
    (\dot r\cdot T+r\cdot\dot T)T-(r\cdot T)\dot T,\\
\dot w&=-k_e\dot e_\perp,\qquad
\dot V_\parallel=-\frac{w\cdot\dot w}{V_\parallel},\\
\dot d&=(\dot V_\parallel T+V_\parallel\dot T+\dot w)/V.
\end{aligned}
$$

令 \(\operatorname{clip}_M(a)\) 表示保方向并把向量模长限制在 \(M\)。外层先算角速度目标，内层在实际步长 \(\Delta t\) 内限制角速度变化，再旋转当前航向：

$$
\begin{aligned}
\omega^*&=\operatorname{clip}_{10}\bigl(d\times\dot d
                   +20\,h\times d\bigr),\\
\omega_k&=\omega_{k-1}
       +\operatorname{clip}_{100\Delta t}(\omega^*-\omega_{k-1}),\\
v^*_{k+1}&=V\,\operatorname{Rot}(\omega_k\Delta t)\,h.
\end{aligned}
$$

\(d\times\dot d\) 是**前馈**，它根据参考和当前投影预先给出要转的趋势。\(20\,h\times d\) 是**反馈**，它根据实际与期望方向之差继续修正。\(\operatorname{Rot}\) 表示绕角速度方向转过 \(\|\omega_k\|\Delta t\)，代码用 Rodrigues 旋转公式实现。两个限值的单位分别为 \(10\,\mathrm{rad/s}\) 与 \(100\,\mathrm{rad/s^2}\)。 [方向变化率和前馈反馈](../../src/new_controller/ldlt/include/new_controller/fixed_reference_tracker.hpp#L182) · [角加速度限幅和旋转](../../src/new_controller/ldlt/include/new_controller/fixed_reference_tracker.hpp#L82)

### 8.3 执行模型的逆补偿

执行模型的速度不会瞬间跳到输入值。在正常非零指令且执行加速度限制未触发的分支，模型下一步向输入 \(u_k\) 靠近一部分：

$$
v_{k+1}=v_k+\lambda(u_k-v_k),\qquad
\lambda=\min(1,\Delta t/\tau),\qquad \tau=0.10\,\mathrm s.
$$

希望下一步达到 \(v^*_{k+1}\)，就反解出内部补偿输入：

$$
u_k=v_k+\frac{v^*_{k+1}-v_k}{\lambda}.
$$

因此目标速率为 \(12\,\mathrm{m/s}\) 时，内部补偿输入模长可以超过 12。代码检查下一步所需速度变化不超过 \(150\,\mathrm{m/s^2}\) 乘实际步长，并检查补偿输入模长不超过 \(17\,\mathrm{m/s}\)、相邻输入变化率不超过 \(450\,\mathrm{m/s^2}\)。超限或补偿输入接近零会拒绝正常跟踪步。参考曲率所用的 \(80\,\mathrm{m/s^2}\) 是几何设计值，\(150\,\mathrm{m/s^2}\) 是执行模型的能力校验值。旧的 \(20\,\mathrm{m/s^2}\) 参考指令限幅是另一配置项。 [逆补偿与拒绝条件](../../src/new_controller/ldlt/include/new_controller/fixed_reference_tracker.hpp#L82) · [模型反馈夹具](../../src/new_controller/ldlt/test/fixed_tracking_cases.hpp#L44) · [参数定义](../../src/new_controller/ldlt/include/new_controller/navigation_config.hpp#L5)

### 8.4 滚动续证、参考切换和下一步安全复核

**滚动续证**是用新深度重新证明旧参考的一小段仍有净空。每个新深度组先更新图像数据，随后从当前参考进度 \(s\) 往后退 \(0.05\,\mathrm m\)、对齐到位置网格且不小于零的起点查起，最多查到 \(s+0.35\,\mathrm m\)。遇到第一段未证明净空的区间就截断。期望方向相对当前参考的已采纳意图变化不超过 \(1^\circ\)，且证据覆盖到 \(s+V\Delta t\) 时，可以跳过求场和空间参考重建。证据最长使用 10 ms 仿真时间。重复的状态时间戳可以更新证据，不重复执行控制步。滚动层以相邻实际状态时间戳的差作为 \(\Delta t\)，首步取配置的 5 ms。间隔超过一个配置物理步、状态过期或时钟回退时，进入相应保护或重置流程。

若确需新参考，程序检查新旧连接处的**位置、切向和曲率向量**，再核对接入和新参考的净空。提交时延续累计进度、上一步角速度与补偿输入。新生成的曲线以零曲率开始，所以旧参考还处在非零曲率弯道时会等待可拼接位置。若旧参考也已缺证，进入保护。

即使参考前缀够长，也要核对实际运动候选。程序以 \(p_1=x+v^*_{k+1}\Delta t\) 预测下一步的机体中心。**扫掠**指机体沿这一小段移动时占据的全部空间。代码要求至少一个已认证的覆盖球同时包住两个端点的机体球：

$$
\max\bigl(\|x-c_{\rm cover}\|,\ \|p_1-c_{\rm cover}\|\bigr)
+r_{\rm body}\le r_{\rm cover}.
$$

同一个球满足这个条件时，利用球的凸性，它也包住沿两端点连线的整步机体扫掠。若检查失败，就返回保护状态。这个结果在 ROS `--track-rolling` 入口用于记录候选和状态，运动发布数为零。 [续证窗口和切换验收](../../src/new_controller/ldlt/include/new_controller/rolling_navigation.hpp#L29) · [扫掠计算](../../src/new_controller/ldlt/include/new_controller/rolling_navigation.hpp#L99) · [调度与保护](../../src/new_controller/ldlt/include/new_controller/rolling_navigation.hpp#L173) · [ROS 输出](../../src/new_controller/src/navigation_input.cpp#L200)

## 9. 对照 FLARE：同样先求势再沿场走，求解的空间不同

FLARE 论文第 III 节把局部障碍膨胀后，从三维工作空间中去掉这些障碍区域，得到粗、细自由域。论文在这个流程中没有把 ESDF 作为建域步骤。**ESDF** 是给空间位置存储到最近障碍的带符号欧氏距离的表示，论文中提及它是在讨论其他方法。 [FLARE 论文第 III 节、图 2–3 与式 (2)–(11)](/home/starry/桌面/2027ICRA_FLARE.pdf)

| 对比环节 | 三维 FLARE | 当前 EGO4P2 |
| --- | --- | --- |
| 场所在的位置 | 三维自由空间，变量 \(\xi\) 是一个米制位置。 | 二维单位球面，变量 \(d\) 是一个三维单位方向。 |
| 粗细边界 | 粗场外边界编码期望方向，细场自由耦合边界继承粗势值。 | 粗层用低势 0 与出口 1。默认细场用局部通道、独立低势和继承的出口标签。 |
| 积分起点 | 空间锚点 \(a_r\)。 | 实际速度方向或连接切向 \(q_0\)。 |
| 梯度积分输出 | 直接得到米制空间参考的几何路径。 | 得到累计转角参数下的方向曲线，还要通过曲率安排和位置积分生成空间参考。 |
| 跟踪纠偏 | 用穿过锚点的等势面建立横向坐标。 | 在上次进度附近投影到空间曲线，做横向反馈、方向限幅和执行模型补偿。 |

### 9.1 FLARE 的势方程与积分曲线

用 \(W^c\) 表示粗工作空间，\(\Gamma_{cf}^{\rm free}\) 表示细场与粗场的自由耦合边界，\(q_{\rm cmd}\) 表示命令方向。论文的两层边界值问题为

$$
\begin{aligned}
\Delta_{\mathbb R^3}\phi^c&=0
&&\text{在粗自由域内},&
\phi^c|_{\partial W^c}&=q_{\rm cmd}^{\top}\xi,\\
\Delta_{\mathbb R^3}\phi^f&=0
&&\text{在细自由域内},&
\phi^f|_{\Gamma_{cf}^{\rm free}}&=\phi^c|_{\Gamma_{cf}^{\rm free}},
\end{aligned}
$$

粗、细两层各自暴露的障碍边界都满足 \(\nabla\phi^{c,f}\cdot n=0\)，其中 \(n\) 是边界的法向。场更新后，在该次更新区间使用 \(u_r(\xi)=\nabla_\xi\phi_r^f(\xi)\)。空间锚点 \(a_r\) 选出一条穿过它的积分曲线。梯度非零处，用单位梯度只改变沿曲线前进的参数速度，仍沿同一条几何曲线。EGO4P2 在方向球面上执行相似的梯度积分，所得的球面点需要通过第 6 节的米制积分才能确定位置。 [FLARE 式 (2)–(6)](/home/starry/桌面/2027ICRA_FLARE.pdf)

### 9.2 FLARE 的 flow-box 跟踪

**等势面**是势值相同的空间点组成的面。**flow-box** 可以理解为沿一束局部流线划分坐标。沿每条流线追溯到穿过锚点的等势面，用交点在这个面上的两个坐标表示横向偏差。论文用 \(\Psi_r(\xi)\) 表示从 \(\xi\) 所在流线找到的交点，\(E_r\) 的两列是等势面切平面的正交单位基，定义

$$
\chi_r(\xi)=E_r^\top\bigl(\Psi_r(\xi)-a_r\bigr),\qquad
\eta_r(t)=\chi_r(x(t)),\qquad J_r=D\chi_r.
$$

\(J_r\) 是 **Jacobian 矩阵**，描述空间位置的微小变化怎样改变两个横向坐标。\(J_r^\dagger\) 是论文定义的右伪逆，用来把横向纠偏变回三维速度。令 \(K_f\) 表示论文中的正定反馈增益矩阵，\(\hat u_r=u_r/\|u_r\|\)，则

$$
v_{\rm cmd}=v_t\hat u_r-J_r^\dagger K_f\eta_r,\qquad
J_r^\dagger=J_r^\top(J_rJ_r^\top)^{-1}.
$$

论文在局部坐标成立、参考固定且采用理想速度执行模型时推出 \(\dot\eta_r=-K_f\eta_r\)。当前 EGO4P2 采用第 8 节的投影误差 \(e_\perp\) 和带限值的离散执行模型。两者都包含沿参考前进与横向纠偏，误差坐标和控制方程各自定义。 [FLARE 式 (7)–(11)](/home/starry/桌面/2027ICRA_FLARE.pdf)

## 10. 如何介绍实验成果和当前边界

**闭环**指执行后的实际位置和速度会返回给控制器，用于计算下一次命令。**FIFO5** 指这组测试中五个算法线程采用 Linux `SCHED_FIFO` 优先级 5，四路相机与协调线程分开绑定 CPU。它描述调度方式。**P99** 指 99% 样本位于该耗时及以下的统计位置。讲耗时要同时说清样本来自哪里、计时包含哪些阶段、最大值是否超过预算。

| 证据 | 已有结果 | 这组证据覆盖的范围 |
| --- | --- | --- |
| 第 5 步模型闭环与 FIFO5 | 8,400 个模型周期中，8,328 个跳过求场，续证加跟踪平均 \(0.858\) ms，P99 \(1.230\) ms，最大 \(1.506\) ms。72 个含规划周期平均 \(4.293\) ms，最大 \(6.467\) ms，12 次超过 5 ms。 | 模型用几何观测谓词提供净空证据。12 次超时均在首次规划。 |
| ROS 原始深度回放 | 实测接收 10 个完整组，经历一次时钟回退。近场原图净空不足时进入保护，运动发布数为零。 | 验证输入链、保护和重置行为。 |
| R05 Isaac Sim 隔离引导飞行 | \(41.50\) 仿真秒，\(308.925\,\mathrm m\)，独立检查 \(8{,}300\) 步扫掠零重叠，终点误差 \(0.169\,\mathrm m\)，最终速度为零。 | 使用历史路线做几何引导、四路实时深度、独立碰撞复核，以及实验专用起停和引导控制。只给终点的自主对照未完成。 |
| R05 滚动分支计时 | 552 个有完整分段计时的周期平均 \(1.175\) ms，P99 \(3.618\) ms，最大 \(4.914\) ms，0 次超过 5 ms。 | 五线程普通调度。其余 7,648 个引导控制周期和外层安全复核缺对应的完整分段计时。整轮飞行的 5 ms 算法时限仍待完整测量。 |

R05 的 **5 ms 是仿真物理步长**。实际墙钟平均约 18.60 ms 每飞行步，包含渲染、通信、控制与独立碰撞复核。该实验的 Isaac 深度入口还有专用无命中适配。在核对各相机远裁面大于 \(4.5\,\mathrm m\) 后，才将该渲染器报告的 `+Inf` 无命中值映射为 \(4.5\,\mathrm m\) 光轴深度，局部净空查询仍截在 \(4\,\mathrm m\)。这一行为属于实验入口，第 2 节描述的是通用深度预处理。视频是已验收 R05 飞行轨迹在原 Isaac Sim 场景中的重渲染回放，飞行指标取自原运行与独立审计。 [第 5 步实现和计时](../../../EGO4P3/docs/performance/step5_rolling_20261107/REPORT.md) · [R05 实验、计时和视频](../../../EGO4P3/docs/performance/r05_flight_20261107/REPORT.md)
