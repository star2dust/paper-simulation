---
title: '精确分布式一阶优化方法(Exact Distributed First-Order Method)'
date: 2020-12-26 16:41:26
tags: [Optimization]
published: true
hideInList: false
feature: 
isTop: false
---
@[TOC]

# 写在前面

本文简单的对已有的分布式一阶优化方法做个综述。虽然原论文以及大部分引文提出的是离散时间系统分布式优化方法，但由于我本人研究的课题主要针对连续时间系统，所以本文所述的分布式优化方法主要以连续时间形式呈现。

基于上述原因，本文中不再细述原论文关于离散时间系统的证明，主要是分享现有的分布式一阶优化方法，以及设计连续时间方法中我的一些想法和思考。

# 问题描述

考虑分布式优化问题
$$
\begin{aligned}\min&\quad f(x)=\sum_{i=1}^n f_i(x_i)\\\operatorname{s.t.} &\quad x_1=x_2=\cdots=x_n，\end{aligned}
$$

和单积分器系统
$$
\dot x_i = u_i,
$$
设计控制器使得$x$实现一致性的同时收敛到$f(x)$的最小值。

# 分布式梯度下降(DGD)

提到分布式一阶优化方法，最早的就是**分布式梯度下降**(distributed gradient desent, DGD)。该方法连续时间形式给出如下：
$$
\dot x=-Lx-\alpha\nabla f(x),
$$
其中$\alpha$为优化步长，$\nabla f(x)=[\nabla f_1^T(x_1),\cdots,\nabla f_n^T(x_n)]^T$。该方法满足$\frac{1}{n}1_n^T\dot x=-\frac{1}{n}\alpha 1_n^T\nabla f(x)$，即$x$的均值一直在沿着$f(x)$梯度下降的方向运动。

容易看出，当$\alpha$为**固定步长**(fixed step)时，该方法的收敛是不精确的。令$V=\alpha f(x)+x^TLx$，可得
$$
\dot V=-\|Lx+\alpha \nabla f(x)\|^2\leq 0,
$$
但不能保证$Lx=0$和$1_n^T\nabla f(x)=0$同时成立。因为若要$Lx=0$，则需要$\nabla f(x)=0$，但是每一个$\nabla f_i(x_i)=0$的解通常不是同一个$x$，所以平衡点处$Lx=0$不一定成立。为了使分布式梯度下降能够精确收敛于最优解，只能将$\alpha$设置为**递减步长**(diminishing step)，使得当$t\to\infty$时$\alpha\to 0$，但是这样做牺牲了收敛速度。

# 精确一阶算法(EXTRA)

为了保证最优解的精确性，同时不牺牲收敛速度，Shi 2015给出了**精确一阶算法**(exact first-order algorithm, EXTRA)的离散时间形式[^shi2015extra]。

<img src="https://star2dust.github.io/post-images/1608976269228.PNG" style="zoom: 67%;" />

Kia 2015给出该方法的连续时间形式[^Kia2015distributed]如下：
$$
\begin{aligned}
\dot x&=-Lx-\alpha \nabla f(x)+y\\
\dot y&=-Lx, 
\end{aligned}
$$
其中$\alpha$是固定正值，且$1_n^Ty(0)=0$。可以看出，$y$是$-Lx$的积分项，作为对$Lx=0$精确性的补偿。只有当$Lx=0$时，$\dot y=0$，补偿停止。同时，初值条件$1_n^Ty(0)=0$和$1_n^T\dot y=0$联合保证了$\frac{1}{n}1_n^T\dot x=-\frac{1}{n}\alpha 1_n^T\nabla f(x)$，因此能够收敛到最优解。

# 原始对偶算法

将分布式优化问题写成如下形式
$$
\begin{aligned}\min&\quad f(x)=\sum_{i=1}^n f_i(x_i)\\\operatorname{s.t.} &\quad D^Tx=0.\end{aligned}
$$

构建增广拉格朗日函数
$$
\mathcal L(x,y)=f(x)+\frac{1}{\alpha }y^TD^Tx+\frac{1}{2\alpha }x^TLx,
$$
就可以使用经典的**原始对偶算法**(primal-dual algorithm)解决：
$$
\begin{aligned}
\dot x&=-\alpha \nabla_x \mathcal L(x,y)\\
\dot y&=\alpha \nabla_y \mathcal L(x,y),
\end{aligned}
$$
即
$$
\begin{aligned}
\dot x&=-Lx-\alpha \nabla f(x)-Dy\\
\dot y&=D^Tx, 
\end{aligned}
$$
其中$D$是图的**关联矩阵**(incidence matrix)，满足$L=DD^T$。上面第二个式子用$\dot y=D^T(x+\dot x)$也可以，参考Dusan 2019[^dusan2019unification]。

可以看出，原始对偶算法和EXTRA算法等价[^Mokhtari2016decentralized]。将EXTRA算法中的$y$替换为$-Dy$，即可得到原始对偶算法。注意原始对偶算法无需初始条件限制，因为$1_n^TDy(0)=0$恒成立。

[^shi2015extra]: Shi, W., Ling, Q., Wu, G., & Yin, W. (2015). Extra: An exact first-order a lgorithm for decentralized consensus optimization. SIAM Journal on Optimization, 25(2), 944–966. https://doi.org/10.1137/14096668X
[^Kia2015distributed]: Kia, S. S., Cortés, J., & Martínez, S. (2015). Distributed convex optimization via continuous-time coordination algorithms with discrete-time communication. In Automatica (Vol. 55, pp. 254–264). Elsevier Ltd. https://doi.org/10.1016/j.automatica.2015.03.001

[^Mokhtari2016decentralized]: Mokhtari, A., & Ribeiro, A. (2016). DSA: Decentralized double stochastic averaging gradient algorithm. J. Mach. Learn. Res., vol. 17, pp. 1–35.

[^dusan2019unification]: Jakovetić, D. (2019). A Unification and Generalization of Exact Distributed First-Order Methods. IEEE Transactions on Signal and Information Processing over Networks, 5(1), 31–46. https://doi.org/10.1109/TSIPN.2018.2846183

# 分布式梯度跟踪(DGT)

为了保证最优解的精确性，同时不牺牲收敛速度，Qu 2018[^qu2018harnessing]给出了另一个思路，即**分布式梯度跟踪**(distributed gradient tracking, DGT)。该方法的连续时间形式如下：
$$
\begin{aligned}
\dot x&=-Lx-\alpha y\\
\dot y&=-Ly+\nabla^2f(x)\dot x,
\end{aligned}
$$
其中$\alpha$是固定正值，且$y(0)=\nabla f(x(0))$。可以看出，$y_i$作为梯度均值$\frac{1}{n}1_n^T\nabla f(x)$的估计，平衡点处梯度均值趋于0，从而保证$Lx=0$。

值得一提的是，该方法在梯度跟踪部分引入了$\dot x$，而$\dot x=0$反过来保证$Ly=0$，因此所有$y_i$相等且为$\frac{1}{n}1_n^T\nabla f(x)$。这个思路在Dusan 2019中也有使用，相当于在原始对偶算法中的拉格朗日乘子部分引入了$\dot x$，而$\dot x$最终趋近于0，因此不影响最终结果。

[^qu2018harnessing]: Qu, G., & Li, N. (2018). Harnessing smoothness to accelerate distributed optimization. IEEE Transactions on Control of Network Systems, 5(3), 1245–1260. https://doi.org/10.1109/TCNS.2017.2698261



