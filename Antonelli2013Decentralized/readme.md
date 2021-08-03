---
title: '【论文笔记】分布式加权质心跟踪(Weighted Centroid Tracking)的观测控制器设计'
date: 2021-02-25 11:23:05
tags: [Control Theory,Matlab]
published: true
hideInList: false
feature: 
isTop: false
---
@[TOC]

# 写在前面

原论文：A Decentralized Controller-Observer Scheme for Multi-Agent Weighted Centroid Tracking。

本文是Antonelli 2013[^antonelli2013decentralized]的总结，主要介绍了一种控制器和观测器结合的分布式加权质心跟踪方法，其中每一个智能体都通过观测器估计所有智能体的状态。

[^antonelli2013decentralized]: G. Antonelli, F. Arrichiello, F. Caccavale, and A. Marino, “A decentralized controller-observer scheme for multi-agent weighted centroid tracking,” IEEE Trans. Automat. Contr., vol. 58, no. 5, pp. 1310–1316, May 2013.

# 问题描述

定义$x=[x_1^T,\cdots,x_n^T]^T$。系统模型
$$
\dot x=u\qquad (1)
$$
控制目标是加权质心跟踪：
$$
\sigma(x)=\sum_{i=1}^nw_ix_i\to\sigma_d
$$

# 状态观测器

令$\Pi_i\in\mathbb R^{nd\times nd}$为选择矩阵，即$\Pi_i=\operatorname{diag}\{O_d,\cdots,I_d,\cdots,O_d\}$，仅有第$i$块矩阵为单位阵，其余均为全零矩阵，满足$\sum_{i=1}^n\Pi_i=I_{nd}$。

智能体$i$通过如下观测器估计整个系统的状态：
$$
^i\dot {\hat x}=k_o\left(\sum_{j\in\mathcal N_i}({^j\hat x}-{^i\hat x})+\Pi_i(x-{^i\hat x})\right)+{^i\hat u}\qquad (2)
$$
其中$k_o>0$，且
$$
^i\hat x=\begin{bmatrix}
^i\hat x_{1}\\
^i\hat x_{2}\\
\vdots\\
^i\hat x_{n}
\end{bmatrix},\quad ^i\hat u=\begin{bmatrix}
^i\hat u_{1}\\
^i\hat u_{2}\\
\vdots\\
^i\hat u_{n}
\end{bmatrix}\in\mathbb R^{nd}.
$$
**注意到，每一个智能体只用到了局部信息，因为$\Pi_i$只会选择第$i$个智能体(即自身)的状态。**

简洁起见，将(2)写成矩阵形式
$$
\dot {\hat {\boldsymbol x}}=-k_o\boldsymbol L \hat {\boldsymbol x}+k_o\boldsymbol \Pi\tilde {\boldsymbol x}+\hat {\boldsymbol u}\qquad (3)
$$
其中$\boldsymbol L=L\otimes I_{nd}$，$\boldsymbol \Pi=\operatorname{diag}\{\Pi_1,\cdots,\Pi_n\}$，且**估计状态误差**和**估计控制输入**定义如下
$$
\tilde {\boldsymbol x}=\begin{bmatrix}
^1\tilde x\\
\vdots\\
^n\tilde x
\end{bmatrix}=\begin{bmatrix}
x-{^1\hat x}\\
\vdots\\
x-{^n\hat x}
\end{bmatrix}=1_n\otimes x-\hat {\boldsymbol x},\quad \hat {\boldsymbol u}=\begin{bmatrix}
^1\hat u\\
\vdots\\
^n\hat u
\end{bmatrix}\in\mathbb R^{n^2d}
$$
定义$u$为真实控制输入。**估计状态误差动力学**为
$$
\dot {\tilde {\boldsymbol x}}=-k_o(\boldsymbol L+\boldsymbol \Pi)\tilde {\boldsymbol x}+1_n\otimes u-\hat {\boldsymbol u}\qquad (4)
$$
**注意到，$\boldsymbol L+\boldsymbol \Pi$对于连通无向图以及强连通平衡图是正定矩阵，这个性质在后面会用到。**

## 证明$\boldsymbol L+\boldsymbol \Pi$正定性

显然两个矩阵都是半正定的，那么相加也同样是半正定的。如果是正定，除非他们的零空间不相交，即
$$
\operatorname{null}(\boldsymbol L)\cap\operatorname{null}(\boldsymbol \Pi)=\{0_{n^2d}\}.
$$
注意到$\boldsymbol L,\boldsymbol \Pi\in\mathbb R^{n^2d\times n^2d}$，而$\operatorname{rank}(\boldsymbol L)=nd(n-1)$，则$\dim(\operatorname{null}(\boldsymbol L))=nd$。那么$\operatorname{null}(\boldsymbol L)$可以如下参数化
$$
\operatorname{null}(\boldsymbol L)=\operatorname{span}(1_n\otimes I_{nd})
$$
其中的向量满足
$$
v_L=[\nu^T,\cdots,\nu^T]^T\in\mathbb R^{n^2d},\quad \forall \nu\in\mathbb R^{nd}
$$
同样地，我们有$\operatorname{rank}(\boldsymbol \Pi)=nd$，则$\dim(\operatorname{null}(\boldsymbol \Pi))=nd(n-1)$。那么$\operatorname{null}(\boldsymbol \Pi)$可以如下参数化
$$
\operatorname{null}(\boldsymbol \Pi)=\operatorname{span}( I_{n^2d}-\boldsymbol \Pi)
$$
其中的向量满足
$$
v_\Pi=[v_1^T,\cdots,v_n^T]^T\in\mathbb R^{n^2d}\\
v_i=[v_{i,1}^T,\cdots,v_{i,n}^T]\in\mathbb R^{nd}: \forall v_{i,j}\in\mathbb R^d,v_{i,i}=0_d
$$
比较$v_L$和$v_\Pi$可知，非空的$v_L$不可能属于$\operatorname{null}(\boldsymbol \Pi)$，反之非空的$v_\Pi$也不可能属于$\operatorname{null}(\boldsymbol L)$。

## 经典consensus算法

网络化系统中，只要有一个节点知道参考信息，而拓扑又是连通的，那么就可以利用经典consensus算法使得参考信息传递到整个网络中，有兴趣可以比较一下，这里不继续讨论。
$$
^i\dot {\hat x}_k=\frac{1}{{^ib}_k+\sum_{j=1}^n a_{ij}}\left(\sum_{j\in\mathcal N_i} a_{ij}({^j\dot {\hat x}}_k-\gamma({^i\hat x}_k-{^j\hat x}_k))+{^ib}_k(\dot x_k-\gamma({^i\hat x_k}-x_k))\right)
$$

其中$^ib_k=1$如果$i=k$，否则$^ib_k=0$，$k=1,\cdots,n$。

# 加权质心跟踪算法

定义$w=[w_1,\cdots,w_n]^T$，$\boldsymbol w=w\otimes I_d$。那么$\dot \sigma=\boldsymbol w^T\dot x$，有集中式算法($u_0\in\mathbb R^{nd}$)
$$
u_0=(\boldsymbol w^T)^\dagger(\dot \sigma_d-k_c(\sigma(x)-\sigma_d))\qquad (5)
$$
其中$k_c>0$，$(w^T)^\dagger=(1/\|w\|^2)w$，满足$w^T(w^T)^\dagger=I_n$。

受(5)启发，可以分布式设计智能体$i$的真实控制输入($u_i={^i\hat u_i\in\mathbb R^d}$)
$$
u_i={^i\hat u_i}=\frac{w_i}{\|w\|^2}(\dot \sigma_d-k_c(\sigma({^i\hat x})-\sigma_d)\qquad (6)
$$
由于智能体$i$对自身的估计是真实的，故可以直接设置为自身的控制输入。

相应的，智能体$i$对其他智能体控制输入的估计为
$$
^i\hat u_j=\frac{w_j}{\|w\|^2}(\dot \sigma_d-k_c(\sigma({^i\hat x})-\sigma_d)\qquad (7)
$$

# 闭环动力学

定义$u=[u_1^T,\cdots,u_n^T]^T=[{^1\hat u_1^T},\cdots,{^n\hat u_n^T}]^T$。因为
$$
^j\hat u_j-{^i\hat u_j}=k_c\frac{w_j}{\|w\|^2}\boldsymbol w^T({^i\hat x}-{^j\hat x})=k_c\frac{w_j}{\|w\|^2}\boldsymbol w^T({^j\tilde x}-{^i\tilde x})\qquad (8)
$$
所以如下等式成立：
$$
u-{^i\hat u}=-k_c\frac{\boldsymbol w\boldsymbol w^T}{\|w\|^2}{^i\tilde x}+k_c\frac{\operatorname{diag}(w)\otimes\boldsymbol w^T}{\|w\|^2}\tilde {\boldsymbol x}=:-A_o{^i\tilde x}+B_o\tilde {\boldsymbol x}\qquad(9)
$$
其中$A_o\in\mathbb R^{nd\times nd}$，$B_o\in\mathbb R^{nd\times n^2d}$。

将**估计状态误差动力学**(4)重写为
$$
\dot {\tilde {\boldsymbol x}}=-k_o(\boldsymbol L+\boldsymbol \Pi)\tilde {\boldsymbol x}-(\boldsymbol A_o-\boldsymbol B_o)\tilde {\boldsymbol x}\qquad (10)
$$
其中$\boldsymbol A_o=I_n\otimes A_o$半正定，$\boldsymbol B_o=1_n\otimes B_o$不定。

由(6)可得，
$$
\begin{aligned}
\dot {\tilde \sigma}&=\dot \sigma_d-\dot \sigma=\dot \sigma_d-\sum_{i=1}^n w_i\dot x_i\\
&=\dot \sigma_d-\sum_{i=1}^n\frac{w_i^2}{\|w\|^2}(\dot \sigma_d-k_c(\sigma({^i\hat x})-\sigma_d)\\
&=-k_c\tilde \sigma-\frac{k_c}{\|w\|^2}\sum_{i=1}^n w_i^2\boldsymbol w^T(x-{^i\hat x})\\
&=-k_c\tilde \sigma-\frac{k_c}{\|w\|^2}\boldsymbol w^T\sum_{i=1}^n w_i^2{^i\tilde x}
\end{aligned}
$$
因此，**跟踪误差动力学**可写为
$$
\dot {\tilde \sigma}=-k_c\tilde \sigma-\boldsymbol B_c\tilde {\boldsymbol x}\qquad(11)
$$
其中$\boldsymbol B_c=\frac{k_c}{\|w\|^2}\boldsymbol w^T(\boldsymbol w^T\operatorname{diag}\{\boldsymbol w\}\otimes I_n)\in \mathbb R^{d\times n^2d}$。

# 收敛性分析

定义李雅普诺夫函数
$$
V=V_o+\delta V_c=\frac{1}{2}\tilde {\boldsymbol x}^T\tilde {\boldsymbol x}+\frac{\delta}{2}\tilde \sigma^T\tilde \sigma\qquad (12)
$$
其中$\delta>0$不需要设计，仅用在证明中。

易知$V$满足如下不等式
$$
c_m\left\|\begin{bmatrix}
\tilde {\boldsymbol x}\\
\tilde \sigma
\end{bmatrix}\right\|^2\leq V\leq c_M\left\|\begin{bmatrix}
\tilde {\boldsymbol x}\\
\tilde \sigma
\end{bmatrix}\right\|^2\qquad (13)
$$
对任意$c_m\leq \min\{1,\delta\}/2$和$c_M\geq \max\{1,\delta\}/2$成立。

由(10)，$V_o$对时间求导得
$$
\dot V_o=-k_o\tilde {\boldsymbol x}^T(\boldsymbol L+\boldsymbol \Pi)\tilde {\boldsymbol x}-\tilde {\boldsymbol x}(\boldsymbol A_o-\boldsymbol B_o)\tilde {\boldsymbol x}\qquad (14)
$$

由于$\boldsymbol L+\boldsymbol \Pi$和$\boldsymbol A_o$都正定，有
$$
\dot V_o\leq -\lambda_o\|\tilde {\boldsymbol x}\|^2+\tilde {\boldsymbol x}^T\boldsymbol B_o\tilde {\boldsymbol x}\qquad (15)
$$
其中$\lambda_o=k_o\lambda_m$，$\lambda_m$是$\boldsymbol L+\boldsymbol \Pi$的最小特征根，仅由网络拓扑决定。

由(8)(9)，不等式(15)计算得到
$$
\begin{aligned}
\dot V_o&\leq  -\lambda_o\|\tilde {\boldsymbol x}\|^2+\frac{k_c}{\|w\|^2}\tilde {\boldsymbol x}^T(1_n\otimes(\operatorname{diag}(w)\otimes\boldsymbol w^T))\tilde {\boldsymbol x}\\
&= -\lambda_o\|\tilde {\boldsymbol x}\|^2+\frac{k_c}{\|w\|^2}\sum_{j=1}^n\sum_{i=1}^nw_j{^i\tilde { x}}_j^T\boldsymbol w^T{^j\tilde { x}}\\
&\leq -\lambda_o\|\tilde {\boldsymbol x}\|^2+\frac{k_c}{\|w\|^2}\sum_{j=1}^n|w_j|\| {^i\tilde { x}}\|\sqrt{\lambda_M(ww^T)}\|{^j\tilde { x}}\|\\
&\leq -\lambda_o\|\tilde {\boldsymbol x}\|+\frac{k_c}{\|w\|}\sum_{i=1}^n\sum_{j=1}^n|w_j|\|{^i\tilde { x}}\|\|{^j\tilde { x}}\|\\
&\leq -\lambda_o\|\tilde {\boldsymbol x}\|+k_c\sum_{i=1}^n\sum_{j=1}^n\|{^i\tilde { x}}\|\|{^j\tilde { x}}\|\\
&\leq -\lambda_o\|\tilde {\boldsymbol x}\|+k_c\sum_{i=1}^n\sum_{j=1}^n\|{^i\tilde { x}}\|^2\\
&\leq -(\lambda_o-nk_c)\|{\tilde {\boldsymbol x}}\|^2
\end{aligned}
$$
由(11)，$V_c$对时间求导得
$$
\begin{aligned}
\dot V_c&=\tilde \sigma^T\dot {\tilde \sigma}=\tilde \sigma^T(-k_c\tilde \sigma-\boldsymbol B_c\tilde {\boldsymbol x})\\
&=\tilde \sigma^T\left(-k_c\tilde \sigma-\frac{k_c}{\|w\|^2}\boldsymbol w^T\sum_{i=1}^nw_i^2{^i\tilde { x}}\right)\\
&\leq -k_c\|\tilde \sigma\|^2+\frac{k_c}{\|w\|}\|\tilde \sigma\|\sum_{i=1}^nw_i^2\|{^i\tilde { x}}\|\\
&\leq -k_c\|\tilde \sigma\|^2+n k_c \|w\| \|\tilde \sigma\|\|{\tilde {\boldsymbol x}}\|\\
\end{aligned}
$$
因此，李雅普诺夫函数(12)对时间求导得
$$
\dot V\leq -\begin{bmatrix}
\|\tilde {\boldsymbol x}\|\\
\|\tilde {\sigma}\|
\end{bmatrix}^T\begin{bmatrix}
\lambda_o-nk_c&-\delta n k_c\|w\|/2\\
-\delta n k_c\|w\|/2&\delta k_c
\end{bmatrix}\begin{bmatrix}
\|\tilde {\boldsymbol x}\|\\
\|\tilde {\sigma}\|
\end{bmatrix}\qquad (16)
$$
则$\dot V$负定，如果$\lambda_o=k_o\lambda_m>nk_c$，即$k_o>nk_c/\lambda_m$，且
$$
\delta<4\frac{k_o\lambda_m -n k_c}{n^2k_c\|w\|^2}.
$$
此时(16)中间的矩阵正定，说明系统指数稳定。

# MATLAB仿真

这里简单做了下仿真，蓝色圆圈(bo)是智能体状态，红色圆圈(ro)是质心参考轨迹，蓝色方块(bs)是智能体加权质心，其他同色菱形(d)是所有智能体对某一智能体状态的估计。

仿真结果如图所示。

<img src="https://star2dust.github.io/post-images/1615987067526.gif" style="zoom:50%;" />

MATLAB代码和以前一样上传到我的GitHub项目[paper-simulation](https://github.com/star2dust/paper-simulation)，运行“weighted_centroid_tracking.m”即可。
