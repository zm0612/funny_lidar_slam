# LOAM代价函数求导

## 前言

LOAM对于激光SLAM的发展起到了举足轻重的作用，他提出了点到线和点到面的误差函数，通过优化的方法，得到了非常不错的激光里程计效果。我猜测作者Zhang Ji很可能是从点到线和点到面的ICP算法中找到的灵感。

在LOAM的论文中以及Zhang Ji早期开源的代码中，对于代价函数求解使用的是欧拉角的方式进行求导解算的。一方面由于未采用矩阵的形式进行推导，导致整个推导过程非常复杂，另一方面在代码实现中，有大量的中间运算过程，实际上对效率也带来了一部分影响。

在后续研究中F LOAM，采用了更加优雅的方式，在SE(3)的理论基础之上推导出了更加规整的雅克比矩阵，并借用ceres进行了实现，也确实对于精度和速度有一定的提升。

LOAM这种使用欧拉角的方式进行优化的方式，一直继承了下来，在LEGO-LOAM、LIO-SAM中均能看到。

在这篇博客中，我将在SO(3)的基础之上进行雅克比矩阵的详细推导，它与F LOAM的SE(3)推导区别并不大，只是我更加喜欢使用SO(3)方式。另外在国内的网站上有较少博客去做这件事，所以我将我的一些理解，向你分享。如果有理解偏颇的地方，烦请斧正。

## 预备知识

本篇博客会比较偏理论一些，需要你有一些先验知识：

（1）简单的矩阵运算；

（2）向量、矩阵求导；

（3）熟悉最优化理论；

（4）了解LOAM的误差函数的意义，本篇重点是探索误差函数的求解，所以不会去介绍误差函数的由来。

## 1. 点到直线(Point to Edge)

### 1.1 点到直线误差函数

$$
d_e = \|(Rp_s + t -p_t) \times \vec n_e\|_2 \tag{1}
$$

> $d_e$：点到直线的距离（标量）；
> $p_t$：目标（地图）点云中的角点；
> $p_s$：源（当前帧）点云中的角点；
> $\vec n_e$：近邻角点组成的直线对应的单位向量。

### 1.2 误差函数对R(旋转)和t(平移)的求导结果

$$
\frac{\partial d_e}{\partial R} = (\vec n_e \times (Rp_s)^{\wedge})^T * \frac{(Rp_s + t -p_t) \times \vec n_e}{\|(Rp_s + t -p_t) \times \vec n\|_2} \\
\frac{\partial d_e}{\partial t} = (-\vec n_e \times I_{3\times3})^T *\frac{(Rp_s + t -p_t) \times \vec n_e}{\|(Rp_s + t -p_t) \times \vec n_e\|_2} \tag{2}
$$

> $I_{3\times 3}$：单位矩阵；
> $\times$：该运算符表示向量叉乘；
> $*$：该运算符表示一般的向量、矩阵之间的乘法。

### 1.3 预备数学公式

在推导公式（1）之前，先解释一下用到的一些数学公式：

**二范数的导数**
$$
\frac{ \partial\|x\|_2 } {\partial x} = \frac{x}{\|x\|_2} \tag{3}
$$

> 其中：$x = [x_0,x_1,x_2,\cdot]^{T}$，$\|x\|_2 = \sqrt{x_0^2+x_1^2+\cdots}$

**标量对向量求导的链式法则**

例如：$z$是关于$y$的函数，$y$是关于$x$的函数，它们的传递过程是：$x -> y->z$. 其中$z$是标量，$y$是向量，$x$是向量，则求导的链式法则如下：
$$
\frac{\partial z}{\partial x} = (\frac{\partial y}{\partial x})^T \frac{\partial z}{\partial y} \tag{4}
$$

**叉乘的交换性质**
$$
u \times v = -v\times u \tag{5}
$$
**旋转矩阵左扰动求导**

由于旋转矩阵不满足加法性质，所以采用扰动模型进行求导，这里采用的是对其左乘一个微小转动量$\delta R$，其对应的李代数$\phi$，于是得到如下结论：（由于求导推导比较简单，并且资料较多，这里只给出结论）
$$
\frac{\partial Rp}{\partial \phi} = -(Rp)^{\wedge} \tag{6}
$$

> $\wedge$：该符号表示向量的反对称矩阵

### 1.4 误差函数对R和t的求导推导

根据以上数学性质，我们开始完成loam代价函数公式的求导，从而得到雅克比矩阵：

**关于R的导数**
$$
\frac{\partial d_e}{\partial R} = \frac{\|(Rp_s + t -p_t) \times \vec n_e\|_2}{\partial R} \tag{7}
$$
对（7）式使用（4）式对应的链式求导法则：
$$
\frac{\partial d_e}{\partial R} = \frac{\|(Rp_s + t -p_t) \times \vec n_e\|_2}{\partial R} \\
= (\frac{\partial((Rp_s + t -p_t) \times \vec n_e)}{\partial R} )^{T} *  \frac{(Rp_s + t -p_t) \times \vec n_e}{\|(Rp_s + t -p_t) \times \vec n\|_2} \tag{8}
$$
对于（8）式中第二行前半部分的偏导数计算：
$$
\frac{\partial((Rp_s + t -p_t) \times \vec n_e)}{\partial R} \\
=  \frac{\partial( -\vec n_e  \times(Rp_s + t -p_t))}{\partial R} \\ 
= \frac{\partial( -\vec n_e  \times(Rp_s))}{\partial R} \\
=  -\vec n_e  \times(-(Rp_s)^{\wedge})  \tag{9}
$$
整理之后,得到关于点到线的误差函数关于R的导数：
$$
\frac{\partial d_e}{\partial R} = (\vec n_e \times (Rp_s)^{\wedge})^T * \frac{(Rp_s + t -p_t) \times \vec n_e}{\|(Rp_s + t -p_t) \times \vec n\|_2} \tag{10}
$$
**关于t的导数**
$$
\frac{\partial d_e}{\partial t} = \frac{\|(Rp_s + t -p_t) \times \vec n_e\|_2}{\partial t} \\
= (\frac{\partial((Rp_s + t -p_t) \times \vec n_e)}{\partial t} )^{T} *  \frac{(Rp_s + t -p_t) \times \vec n_e}{\|(Rp_s + t -p_t) \times \vec n\|_2} \tag{11}
$$
对于（11）式第二行左半部分的偏导数计算：
$$
\frac{\partial((Rp_s + t -p_t) \times \vec n_e)}{\partial t}  \\ 
= \frac{ -\vec  n_e \times  \partial(t)}{\partial t}  \\ 
= -\vec  n_e \times I_{3\times 2} \tag{12}
$$
整理之后，得到关于点到线的误差函数关于t的导数：
$$
\frac{\partial d_e}{\partial t} = (-\vec n_e \times I_{3\times3})^T *\frac{(Rp_s + t -p_t) \times \vec n_e}{\|(Rp_s + t -p_t) \times \vec n_e\|_2} \tag{13}
$$

## 2. 点到平面(Point to Surface)

### 2.1 点到平面误差函数

$$
d_p = \| (Rp_s + t - p_t)^T * \vec n \|_2 \tag{14}
$$

> $p_t$：目标（地图）点云中的平面点；
> $\vec n_s$：近邻平面点组成的平面对应的法向量。

### 2.2 误差函数对R(旋转)和t(平移)的求导结果

$$
\frac{\partial d_p}{\partial R} = (-(Rp_s)^{\wedge})^T*\vec{n}* \frac{ (Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2}\\
\frac{\partial d_p}{\partial t} = I_{3\times3} *\vec{n} * \frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} \tag{15}
$$

### 2.3 误差函数对R和t的求导推导

根据第1节的推导过程，对于点到平面误差函数的推导就非常容易了！

**关于R的导数**
$$
\frac{\partial{d_p}}{\partial R} = \frac{\partial(\|(Rp_s + t - p_t)^T * \vec n \|_2)}{\partial R} \\ 
= ( \frac{\partial((Rp_s + t - p_t)^T * \vec n)}{\partial R})^T * \frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} \\
= ( \frac{\partial((Rp_s)^T * \vec n)}{\partial R})^T * \frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} \\
=(-(Rp_s)^{\wedge})^T*\vec{n}* \frac{ (Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} \tag{16}
$$
**关于t的导数**
$$
\frac{\partial{d_p}}{\partial t} = \frac{\partial(\|(Rp_s + t - p_t)^T * \vec n \|_2)}{\partial t} \\ 
=  \frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2}* \frac{\partial((Rp_s + t - p_t)^T * \vec n)}{\partial t}  \\
= \frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} * \frac{\partial(t)^T * \vec n}{\partial t} \\
= \frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} *  I_{3\times3} *\vec{n} \tag{17}
$$

## 3. 对于loam代价函数的总结

**loam的总体代价函数**

如下：
$$
d = \sum_N d_e + \sum_M d_s  \\
= \sum_N  \|(Rp_s + t -p_t) \times \vec n_e\|_2 + \sum_M \| (Rp_s + t - p_t)^T * \vec n \|_2 \tag{18}
$$
根据1和2小节的求导，可以得到每一个误差的导数，组成一个大的雅克比矩阵(也有可能是向量)，有了雅克比矩阵之后带入高斯牛顿或者LM算法即可以求解最优的R和t。该过程是属于最优化理论相关的内容，是比较成熟的理论，不是本篇博客要探索的，不在此做细致介绍。

**是否有更好的代价函数形式？**

我在复现loam的过程中发现，其中点到线的误差项，也就是$ \sum d_e$，它实际优化过程中对于R和t求解的贡献比较小，其主要原因是点到面的误差项$\sum d_s$中M的数量比较庞大，在16线激光雷达中，经过我的验证M几乎是N的20倍以上，所以实际过程中，如果我们省略掉点到线的误差项，对于最终的精度并未产生明显影响。也许在插满了细长柱子的环境中点到线的误差项才会有明显作用。否则我认为多数真实环境下，点到面的误差项实际上已经涵盖住了点到线的误差。所以在后来一些开源项目中，例如r3live、fast-lio都只计算点到面的误差。

另外，我也尝试对loam的代价函数做了一些改变，如下式，构建成两个最小二乘项的求和：
$$
d = \sum_N (d_e)^2 + \sum_M (d_s)^2 \\
= \sum_N  \|(Rp_s + t -p_t) \times \vec n_e\|_2^2 + \sum_M \| (Rp_s + t - p_t)^T * \vec n \|_2^2 \tag{19}
$$
也就是说，对代价函数计算平方二范数误差。在我的意识中，平方二范数会有更好的收敛性，上式应该会比loam的二范数收敛的更好。但是后来的实验中证明我的想法是错误的，上式的求解得到的R和t与真值差距较大。大家可以去思考一下是什么原因导致的？如果有想法的话，可以在评论区留下你的看法。

**算法实现过程中的一些小细节**

对于式（16）和（17），它们均包含下式：
$$
\frac{(Rp_s + t - p_t)^T * \vec n}{\| (Rp_s + t - p_t)^T * \vec n \|_2} \tag{20}
$$
观察可以发现，它的值是-1或者1，如果你没看出来，可以花些时间想一想，并不难！

### 4.补充内容

### 直线向量

$$
A =
\left[
\begin{matrix}
x_0 & x_1 & x_2 & x_3 & x_4 \\
y_0 & y_1 & y_2 & y_3 & y_4 \\
z_0 & z_1 & z_2 & z_3 & z_4
\end{matrix}
\right]
$$

> 搜索最近的5个点，使用SVD分解得到三个维度上的特征向量。



### 平面方程计算

$$
ax+by+cz+1 = 0
$$

目标点集中最近5个点：$[x_0, y_0, z_0]^T$, $[x_1, y_1, z_1]^T$,…,$[x_4, y_4, z_4]^T$
$$
ax_0 + by_0 +cz_0 = -1 \\
ax_1 + by_1 +cz_1 = -1 \\
...\\
ax_4 + by_4 +cz_4 = -1
$$
写成矩阵的形式：
$$
\left[
\begin{matrix}
x_0 & y_0 & z_0 \\
x_1 & y_1 & z_1 \\
x_2 & y_2 & z_2 \\
x_3 & y_3 & z_3 \\
x_4 & y_4 & z_4
\end{matrix}
\right]

\left[
\begin{matrix}
a\\ b\\ c
\end{matrix}
\right]
=
\left[
\begin{matrix}
-1\\ -1\\ -1\\ -1\\ -1
\end{matrix}
\right]
$$
