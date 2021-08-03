---
title: '【论文笔记】连续时间梯度跟踪算法收敛性分析'
date: 2021-02-01 16:57:31
tags: [Optimization]
published: true
hideInList: false
feature: 
isTop: false
---
@[TOC]

# 写在前面

原论文：Convergence Analysis of a Continuous-Time Distributed Gradient Descent Algorithm

本文是Zhang 2021[^zhang2021convergence]的笔记，原论文将Qu 2018[^qu2018harnessing]的梯度跟踪算法扩展到连续时间版本，并对收敛性进行分析。我感觉原文变量定义方式不太主流，可能有些错误的地方，因此证明部分自己又重新推导了一遍。欢迎读者检查本文推导部分，如果发现有错误，请评论告诉我，谢谢！

[^zhang2021convergence]: M. Zhang, X. Liu, and J. Liu, “Convergence Analysis of a Continuous-Time Distributed Gradient Descent Algorithm,” IEEE Control Syst. Lett., vol. 5, no. 4, pp. 1339–1344, Nov. 2021.
[^qu2018harnessing]: G. Qu and N. Li, “Harnessing smoothness to accelerate distributed optimization,” IEEE Trans. Control Netw. Syst., vol. 5, no. 3, pp. 1245–1260, Sep. 2018.


# 分布式优化问题

## 问题描述和假设

问题描述
$$
\min_{x\in\mathbb R^d} f(x)=\frac{1}{n}\sum_{i=1}^n f_i(x)
$$

> 假设1：所有$f_i$是$\mu$-强凸函数，即$f_i(x)\geq f_i(y)+\nabla f_i(y)^T(x-y)+\frac{\mu}{2}\|x-y\|^2$对全部$x,y\in\mathbb R^d$成立；且为$L$-光滑，即$\|\nabla f_i(x)-\nabla f_i(y)\|\leq L\|x-y\|$对任意$x,y$成立。

> 假设2：图无向连通。

强凸隐含条件：$\|\nabla f_i(x)-\nabla f_i(y)\|\geq \mu\|x-y\|$对任意$x,y$成立。

## 算法和变量定义

定义$x_i,s_i\in\mathbb R^{d}$，其中$s_i$是每一个智能体对$\frac{1}{n}\nabla f_i(x_i)$的估计。令$x=[x_1^T,\cdots,x_n^T]^T\in\mathbb R^{nq}$。

定义$W=-\beta (L\otimes I_d)$。连续时间梯度跟踪算法的更新律如下：
$$
\begin{aligned}
\dot x&=Wx-s,\\
\dot s&=Ws+\nabla^2,
\end{aligned}\qquad (1)
$$
其中$\nabla^2:=\operatorname{blkdiag}(\nabla^2 f_1,\cdots,\nabla^2f_n)\dot x$，初始值为任意$x_i(0)$和$s_i(0)=\nabla f_i(x_i(0))$。因此$\sum_i s_i(t)=\sum_i \nabla f_i(x_i(t))$恒成立。（同样的后面的$w$也满足此性质。）

可以看出，式(1)直接由Qu 2018中的离散形式推导而来。

为了简化证明，定义$w=-s=\dot x-Wx$和$q=\dot x$。式(1)简化为
$$
\begin{aligned}
\dot x&=w+Wx,\\
\dot w&=Ww-\nabla^2,
\end{aligned}\qquad (2)
$$
 和
$$
\begin{aligned}
\dot x&=q,\\
\dot q&=2W q-W^2x-\nabla^2.
\end{aligned}\qquad (3)
$$

# 收敛性分析

理论分析分为以下四步。前三步证明了收敛性，最后一步证明了最优性。（由于我这里向量定义为列向量，原文向量均为行向量，最后计算出来一些系数不太相同。）

## 速度递减

令$Q=\frac{1}{2}\|q\|^2\geq 0$，$X=\frac{1}{2}\|Wx\|^2\geq 0$。直觉上看，当$x$收敛于一个固定值时，$\dot x$也就是$q$必然收敛于0。利用半正定函数$Q$和$X$，以下引理证明了$\|q\|$的指数收敛性。

> 引理1：在假设1、2下，有$\|q\|\leq \sqrt{2Q(0)+2X(0)}e^{-\mu t}$。

证明：考虑类李雅普诺夫函数$V=Q+X$。对时间求导得到
$$
\begin{aligned}
\dot V&=q^T\dot q+(Wx)^TW\dot x\\
&=2q^TWq-q^TW^2x-q^T\nabla^2+x^TW^2q\\
&=2q^TWq-q^T\nabla^2\leq -\mu q^Tq=-2\mu Q.
\end{aligned}
$$
因此，$\dot Q+\dot X\leq -2\mu Q$，即
$$
\begin{aligned}
Q(t)&\leq -X(t)+X(0)+Q(0)+\int_0^t-2\mu Q(r) dr\\
&\leq X(0)+Q(0)+\int_0^t-2\mu Q(r) dr
\end{aligned}
$$
由[**格朗沃尔不等式**](https://baike.baidu.com/item/Gronwall%E4%B8%8D%E7%AD%89%E5%BC%8F/19134627)(Grönwall's inequality)可得
$$
Q(t)\leq (X(0)+Q(0))e^{-2\mu t},
$$
即$\|q\|\leq \sqrt{2Q(0)+2X(0)}e^{-\mu t}$。

> 格朗沃尔不等式：假设$\alpha,\beta,u$为定义在实数区间$I=[a,b]$($b$可以为$\infty$)上的连续实函数，则有
>
> （a） 如果$\beta$非负，且$u$满足如下积分不等式：
> $$
> u(t)\leq \alpha(t)+\int_a^t\beta(s)u(s)ds,\quad t\in I,
> $$
> ​			那么
> $$
> u(t)\leq \alpha(t)+\int_a^t \alpha(s)\beta(s)\exp(\int_s^t\beta(r)dr)ds,\quad t\in I.
> $$
> （b）如果在之前的条件下，$\alpha$是一个常数，那么
> $$
> u(t)\leq \alpha\exp(\int_a^t\beta (s)ds),\quad t\in I.
> $$

## 梯度递减

令$\bar x=\frac{1}{n}\sum_i x_i$，$\bar w=\frac{1}{n}\sum_i w_i$和$\bar \nabla^2=\frac{1}{n}\sum_i \nabla^2 f_i(x_i)\dot x_i$。

> 引理2：在假设1、2下，有$\|\bar w\|\leq\sqrt{\frac{2}{n}(Q(0)+X(0))}e^{-\mu t}$。

证明：由式(2)得到，
$$
\begin{aligned}
\dot {\bar x}&=\bar w\\
\dot {\bar w}&=-\bar \nabla ^2
\end{aligned}
$$
令$\Pi=\frac{1}{n}11^T\otimes I_d$。由于$\bar q=\dot {\bar x}=\bar w=-\frac{1}{n}\sum_i \nabla f_i(x_i)$，有
$$
\begin{aligned}
n\|\bar w\|^2=n\|\bar q\|^2=\|\Pi q\|^2\leq \|q\|^2.
\end{aligned}
$$

故$\|\bar w\|\leq \sqrt{\frac{2}{n}(Q(0)+X(0))}e^{-\mu t}$。

## 均值一致

定义误差矩阵$\delta_w=w-\Pi w$和$\delta_x=x-\Pi x$。定义相应的李雅普诺夫函数$\Delta_w=\frac{1}{2}\|\delta_w\|^2$和$\Delta_x=\frac{1}{2}\|\delta_x\|^2$。

> 引理3（Olfati-Saber 2004[^olfati2004consensus]）：令图$\mathcal G$是无向图，其拉普拉斯矩阵为$L$。那么$\lambda_2=\min_{1^T\delta=0,\delta\neq 0}(\delta^TL\delta/\delta^T\delta)$是$L$的第二小特征根。

[^olfati2004consensus]: R. Olfati-Saber and R. M. Murray, “Consensus problems in networks of agents with switching topology and time-delays,” IEEE Trans. Autom. Control, vol. 49, no. 9, pp. 1520–1533, Sep. 2004.

> 引理4：在假设1、2下，$w,x$以指数速率达成一致，即
> $$
> \begin{aligned}
> \|\delta_w\|&\leq C_1(e^{-\beta \lambda_2 t}+(1+t)e^{-\mu t}),\\
> \|\delta_x\|&\leq C_2((1+t)e^{-\beta \lambda_2 t}+(1+t)^2e^{-\mu t}),
> \end{aligned}
> $$
> 其中$C_1,C_2$为正常数。

证明：（1）根据定义，有$\dot \delta_w+\Pi \dot w=W\delta_w-\nabla^2$，和
$$
\begin{aligned}
\dot \Delta_w&=\delta_w^T(W\delta_w-\nabla^2)+\delta_w^T\Pi\nabla^2\\
&\leq -2\beta\lambda_2\Delta_w-\delta_w^T\nabla^2\\
&\leq-2\beta\lambda_2\Delta_w+L\|\delta_w\|\|q\|\\
&=-2\beta\lambda_2\Delta_w+2L\sqrt{\Delta_w}\sqrt{Q(0)+X(0)}e^{-\mu t}.
\end{aligned}
$$
即$\|\delta_w\|'\leq -\beta\lambda_2\|\delta_w\|+L\sqrt{2Q(0)+2X(0)}e^{-\mu t}$。

注意到$(\|\delta_w\|'+\beta\lambda_2\|\delta_w\|)e^{\beta \lambda_2t}=(\|\delta_w\|e^{\beta\lambda_2 t})'\leq L\sqrt{2Q(0)+2X(0)}e^{\beta\lambda_2t-\mu t}$，可得
$$
\begin{aligned}
\|\delta_w\|&\leq C_{11}(e^{-\beta\lambda_2 t}+e^{-\mu t}),\quad \beta\lambda_2\neq \mu,\\
\|\delta_w\|&\leq C_{12}(te^{-\mu t}+e^{-\mu t}),\quad \beta\lambda_2\neq \mu.\\
\end{aligned}
$$
取$C_1=\max\{C_{11},C_{12}\}$，可得
$$
\|\delta_w\|\leq C_1(e^{-\beta \lambda_2 t}+(1+t)e^{-\mu t}).
$$
（2）根据定义，有$\dot \delta_x=w+W\delta_x-\Pi w$，和
$$
\begin{aligned}
\dot \Delta_x&=\delta_x^T(w+W\delta_x-\Pi w)\\
&\leq -2\beta\lambda_2\Delta_x+\delta_x^T\delta_w\\
&\leq -2\beta\lambda_2\Delta_x+\|\delta_w\|\sqrt{2\Delta_x}
\end{aligned}
$$
由前面的结论，可得
$$
\|\delta_x\|'\leq-\beta\lambda_2\|\delta_x\|+C_1(e^{-\beta \lambda_2 t}+(1+t)e^{-\mu t}).
$$
同理可得
$$
\|\delta_x\|\leq C_2((1+t)e^{-\beta \lambda_2 t}+(1+t)^2e^{-\mu t}).
$$

## 最优误差

最优性由最优状态误差给出，该误差指数收敛。

> 定理1：在假设1、2下，最优状态误差以指数速率收敛
> $$
> \begin{aligned}
> \|x-1\otimes x^*\|&\leq C_3((1+t)e^{-\beta\lambda_2t}+(1+t)^2e^{-\mu t}),\\
> \end{aligned}
> $$
> 其中$C_3$是正常数。

证明：定义$g=\sum_i\nabla f_i(x_i)=-n\bar w$和$\bar g=\sum_i\nabla f_i(\bar x)$，有
$$
\begin{aligned}
\|g-\bar g\|^2&=\|\sum_i (\nabla f_i(x_i)-\nabla f_i(\bar x))\|^2\\
&\leq \sum_i\|\nabla f_i(x_i)-\nabla f_i(\bar x)\|^2\\
&\leq \sum_iL\|x_i-\bar x\|^2 = L\|\delta_x\|^2.
\end{aligned}
$$
作为结果，我们得到
$$
\begin{aligned}
\|x^*-\bar x\|^2&\leq 1/\mu^2\|\nabla f(x^*)-\nabla f(\bar x)\|^2\\
&=1/\mu^2\|\nabla f(\bar x)\|^2=1/(n\mu)^2\|\bar g\|^2\\
&\leq1/(n\mu)^2(\|g-\bar g\|+\|g\|)^2
\end{aligned}
$$
注意到$\|g-\bar g\|$可用$\|\delta_x\|$表示，$\|g\|$可用$\|\bar w\|$表示。因此由引理2和引理5得到
$$
\begin{aligned}
\|x-1\otimes x^*\|&\leq \|\delta_x\|+\|1\otimes (x^*-\bar x)\|\\
&\leq C_3((1+t)e^{-\beta \lambda_2 t}+(1+t)^2e^{-\mu t}).
\end{aligned}
$$