---
title: '【论文笔记】利用平滑度加速分布式优化——梯度跟踪法(Gradient Tracking)'
date: 2021-01-05 17:13:51
tags: [Optimization]
published: true
hideInList: false
feature: 
isTop: false
---
@[TOC]

# 写在前面

原论文：Harnessing Smoothness to Accelerate Distributed Optimization.

本文是Qu 2018[^qu2018harnessing]的笔记，主要是对离散系统分布式梯度跟踪算法证明过程的总结。

[^qu2018harnessing]: Qu, G., & Li, N. (2018). Harnessing smoothness to accelerate distributed optimization. IEEE Transactions on Control of Network Systems, 5(3), 1245–1260. https://doi.org/10.1109/TCNS.2017.2698261

# 问题描述和算法

问题描述
$$
\min_{x\in\mathbb R^d} f(x)=\frac{1}{n}\sum_{i=1}^n f_i(x)
$$
假设1：$f_i$是$\alpha$强凸和$\beta$光滑的函数。即
$$
\begin{aligned}
f_i(y)-f_i(x)&\geq \nabla f_i(x)^T(y-x)+\frac{\alpha}{2}\|y-x\|^2\\
\|\nabla f_i(x)-\nabla f_i(y)\|&\leq \beta\|x-y\|
\end{aligned}
$$
离散系统分布式优化算法
$$
\begin{aligned}
x_i(t+1)&=\sum_{i=1}^nw_{ij}x_j(t)-\eta s_i(t)\\
s_i(t+1)&=\sum_{j=1}^nw_{ij}s_j(t)+\nabla f_i(x_i(t+1))-\nabla f_i(x_i(t))
\end{aligned}\qquad (1)
$$
其中$x_i(t)\in \mathbb R^{1\times d}$，$s_i(t)\in \mathbb R^{1\times d}$写成行向量形式，同时满足$s_i(0)=\nabla f_i(x_i(0))$。可以看出，$s_i(t)$用来跟踪梯度的均值，即$\frac{1}{n}\nabla f_i(x_i(t))$。上述算法称为**分布式梯度跟踪法**(distributed gradient tracking, DGT)。

# 收敛性证明

给定$W$为**双随机**(doubly stochastic)矩阵。算法(1)矩阵形式
$$
\begin{aligned}
x(t+1)&=W x(t)-\eta s(t)\\
s(t+1)&=W s(t)+\nabla(t+1)-\nabla (t)
\end{aligned}\qquad (2)
$$
其中$s(0)=\nabla (0)$，$\nabla\in\mathbb R^{n\times d}$是梯度$\nabla f_i(x_i(t))$以行向量形式堆叠的矩阵，同样的$x\in\mathbb R^{n\times d}$，$s\in\mathbb R^{n\times d}$也是以行向量形式堆叠的矩阵。

令$\bar x(t)=(1/n)1_n^T x(t)$，$\bar s(t)=(1/n)1_n^T s(t)$，$g(t)=(1/n)1_n^T \nabla(t)$。

> 引理1(Lemma 7)：以下等式成立：
> - $\bar s(t+1)=\bar s(t)+g(t+1)-g(t)=g(t+1)$
> - $\bar x(t+1)=\bar x(t)-\eta \bar s(t)=\bar x(t)-\eta g(t)$

证明：式(2)中两边同乘$(1/n)1_n^T$可得$g(t)=\bar s(t)=(1/n)1_n^T s(t)$，因为$s(0)=\nabla (0)$。同样的，$\bar x(t+1)=\bar x(t)-\eta g(t)$。

下面证明收敛性，证明思路是分别证明**梯度跟踪误差**、**一致性误差**和**最优解误差**的收敛性，最后证明**目标误差**的收敛性。

第一步，先看**梯度跟踪误差**$\|s(k)-1_ng(k)\|$，有
$$
s(k)-1_ng(k)=[W s(k-1)-1_ng(k-1)]+[\nabla(k)-\nabla(k-1)]-1_n[g(k)-g(k-1)].\qquad (3)
$$
令$\sigma\in(0,1)$是$W-(1/n)1_n1_n^T$的**谱范数**(spectral norm)。对任意$\omega\in \mathbb R^n$，都有
$$
\|W\omega-(1/n)1_n1_n^T \omega\|=\|(W-(1/n)1_n1_n^T)(\omega-(1/n)1_n1_n^T \omega)\|\leq \sigma\|\omega-(1/n)1_n1_n^T \omega\|.\qquad (4)
$$
由式(4)和引理1，式(3)可以取范数得到
$$
\begin{aligned}
\|s(k)-1_ng(k)\|&\leq \|Ws(k-1)-1_ng(k-1)\|+\|[\nabla(k)-\nabla(k-1)]-1_n[g(k)-g(k-1)]\|\\
&\leq \sigma\|s(k-1)-1_ng(k-1)\|+\|[\nabla(k)-\nabla(k-1)]-1_n[g(k)-g(k-1)]\|.
\end{aligned}\qquad (5)
$$
式(5)后半部分可得
$$
\begin{aligned}
\|[\nabla(k)-\nabla(k-1)]-1_n[g(k)-g(k-1)]\|^2&=\|\nabla(k)-\nabla(k-1)\|^2+\|(1/n)1_n1_n^T(\nabla (k)-\nabla(k-1))\|^2\\
&\quad -(\nabla(k)-\nabla(k-1))^T(1/n)1_n1_n^T(\nabla (k)-\nabla(k-1))\\
&\leq \|\nabla(k)-\nabla(k-1)\|^2.
\end{aligned}
$$
代入(3)再结合假设1得到
$$
\|s(k)-1_ng(k)\|\leq \sigma\|s(k-1)-1_ng(k-1)\|+\beta\|x(k)-x(k-1)\|.\qquad (6)
$$
第二步，同样的，**一致性误差**由式(4)可得
$$
\begin{aligned}
\|x(k)-1_n\bar x(k)\|&= \|Wx(k-1)-\eta s(k-1)-\bar x(k-1)+\eta1_n g(k-1)\|\\
&\leq \sigma\|x(k-1)-\bar x(k-1)\|+\eta\|s(k-1)-1_ng(k-1)\|
\end{aligned}\qquad (7)
$$
定义$f=\frac{1}{n}\sum_{i=1}^n f_i$在$\bar x(t)$的梯度向量为$h(t)=\nabla f(\bar x(t))\in\mathbb R^{n\times d}$。

第三步，再看**最优解误差**$\|\bar x-x^*\|$，得到
$$
\bar x(k)=\bar x(k-1)-\eta h(k-1)-\eta[g(k-1)-h(k-1)].\qquad (8)
$$

> 引理2(Lemma 3.11 [^bubeck2014convex])：如果$f:\mathbb R^d\to\mathbb R$满足假设1，那么$\forall x,y\in\mathbb R^d$，有
> $$
> (\nabla f(x)-\nabla f(y))^T(x-y)\geq \frac{\alpha\beta}{\alpha+\beta}\|x-y\|^2+\frac{1}{\alpha+\beta}\|\nabla f(x)-\nabla f(y)\|^2.
> $$

[^bubeck2014convex]: Bubeck, S. (2015). Convex optimization: Algorithms and complexity. Foundations and Trends in Machine Learning, 8(3–4), 231–357. https://doi.org/10.1561/2200000050.

> 引理3：$\forall x\in\mathbb R^d$，定义$x^+=x-\eta \nabla f(x)$，其中$0<\eta<\frac{2}{\beta}$且$f$满足假设1，那么
> $$
> \|x^+-x^*\|\leq \lambda \|x-x^*\|
> $$
> 其中$\lambda =\max(|1-\eta \alpha|,|1-\eta \beta|)$。

证明：如果$0<\eta\leq \frac{2}{\alpha+\beta}$且$\alpha< \beta$，那么$\frac{2}{\eta}-\alpha\geq\beta$且$|1-\eta\alpha|\geq |1-\eta\beta|$。可知$f$也是$\frac{2}{\eta}-\alpha$光滑的函数，我们有
$$
\begin{aligned}
\|x-x^*-\eta \nabla f(x)\|^2&=\|x-x^*\|^2+\eta^2\|\nabla f(x)\|^2-2\eta \nabla f(x)^T(x-x^*)\\
&\leq \left(1-2\eta\frac{\alpha(\frac{2}{\eta}-\alpha)}{\alpha+(\frac{2}{\eta}-\alpha)}\right)\|x-x^*\|^2+\left(\eta^2-\frac{2\eta}{\alpha+(\frac{2}{\eta}-\alpha)}\right)\|\nabla f(x)\|^2\\
&=(1-\eta \alpha)^2\|x-x^*\|^2\\
&=\lambda^2\|x-x^*\|^2
\end{aligned}
$$
如果$\frac{2}{\alpha+\beta}<\eta<\frac{2}{\beta}$且$\alpha\geq\beta$，同样可以进行上面的分析。

由引理3和式(8)可得
$$
\begin{aligned}
\|\bar x(k)-x^*\|&\leq \lambda \|\bar x(k-1)-x^*\|+\eta \|g(k-1)-h(k-1)\|\\
&\leq \lambda \|\bar x(k-1)-x^*\|+(\eta\beta/\sqrt{n}) \|x(k-1)-1_n\bar x(k-1)\|
\end{aligned}\qquad(9)
$$
其中$\lambda =\max(|1-\eta \alpha|,|1-\eta \beta|)$。

第四步，看$\|x(k)-x(k-1)\|$的有界性，由假设1得到
$$
\|h(k-1) \|=\|\nabla f(\bar x(k-1)) \|\leq \beta\|\bar x(k-1)-x^* \|.
$$
结合式(9)的方法
$$
\begin{aligned}
\|s(k-1)\|&\leq \|s(k-1)-1_ng(k-1) \|+\|1_ng(k-1)-1_n h(k-1) \|+\|1_n h(k-1)\|\\
&\leq \|s(k-1)-1_ng(k-1) \|+\beta\|x(k-1)-1_n \bar x(k-1) \|+\beta \sqrt{n}\|\bar x(k-1)-x^*\|.
\end{aligned}
$$
因此
$$
\begin{aligned}
\|x(k)-x(k-1) \|&=\|Wx(k-1)-x(k-1)-\eta s(k-1) \|\\
&=\|(W-I)(x(k-1)-1_n\bar x(k-1))-\eta s(k-1) \|\\
&\leq 2\|(x(k-1)-1_n\bar x(k-1))\|+\eta\|s(k-1) \|\\
&\leq \eta\|s(k-1)-1_ng(k-1) \|\\
&\quad+(\eta\beta+2)\|x(k-1)-1_n \bar x(k-1) \|+\eta\beta \sqrt{n}\|\bar x(k-1)-x^*\|.
\end{aligned}
$$
将上式代入(6)得到
$$
\begin{aligned}
\|s(k)-1_n g(k)\|&\leq (\sigma+\beta \eta)\|s(k-1)-1_n g(k-1) \|\\
&\quad +\beta(\eta \beta+2)\|x(k-1)-1_n \bar x(k-1) \|+\eta\beta^2 \sqrt{n}\|\bar x(k-1)-x^*\|
\end{aligned}\qquad (10)
$$
结合(7)(9)和(10)得到

<img src="https://star2dust.github.io/post-images/1609914428009.PNG" style="zoom:67%;" />

因为$z(k)$和$G(\eta)$非负，可以直接迭代展开得到
$$
z(k)\leq G(\eta)^kz(0).
$$
因为$G(\eta)$非负，$G(\eta)^2$为正，所以$G(\eta)^k$每一项都是$O(\rho(G(\eta))^k)$[^horn2012matrix]。$z(h)$每一项以$\rho(G(\eta))^k$速度收敛。

[^horn2012matrix]: R. A. Horn and C. R. Johnson. (2012). Matrix Analysis. Cambridge. U.K.: Cam- bridge Univ. Press.

