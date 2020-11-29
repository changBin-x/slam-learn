# ROS小车的rviz仿真包

# 1. 两轮差速移动机器人运动分析、建模和控制

参考[博客](https://blog.csdn.net/iProphet/article/details/83661753?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param)

## 1.1 运动学分析建模

运动特性为两轮差速驱动，其底部后方两个同构驱动轮的转动为其提供动力，前方的随动轮起支撑作用并不推动其运动，如图1两轮差速驱动示意图所示。

<div align="center">
<img src="img/two-wheels-car-model.png">
</div>

定义其左右驱动轮的中心分别为<a href="https://www.codecogs.com/eqnedit.php?latex=W_l" target="_blank"><img src="https://latex.codecogs.com/gif.latex?W_l" title="W_l" /></a>和<a href="https://www.codecogs.com/eqnedit.php?latex=W_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?W_r" title="W_r" /></a>，且车体坐标系中这两点在惯性坐标系下移动的线速度为<a href="https://www.codecogs.com/eqnedit.php?latex=v_l" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_l" title="v_l" /></a>和<a href="https://www.codecogs.com/eqnedit.php?latex=v_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_r" title="v_r" /></a>，理想情况下即为左右轮转动时做圆周运动的线速度。该值可以通过电机驱动接口输出的角转速<a href="https://www.codecogs.com/eqnedit.php?latex=\phi_l" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\phi_l" title="\phi_l" /></a>，<a href="https://www.codecogs.com/eqnedit.php?latex=\phi_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\phi_r" title="\phi_r" /></a>和驱动轮半径<a href="https://www.codecogs.com/eqnedit.php?latex=r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?r" title="r" /></a>求得，即：

<div align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=v_l=r*\phi_l" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_l=r*\phi_l" title="v_l=r*\phi_l" /></a>
</div>
<div align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=v_r=r*\phi_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_r=r*\phi_r" title="v_r=r*\phi_r" /></a>
</div>

令两驱动轮中心连线的中点为机器的基点<a href="https://www.codecogs.com/eqnedit.php?latex=C" target="_blank"><img src="https://latex.codecogs.com/gif.latex?C" title="C" /></a>,<a href="https://www.codecogs.com/eqnedit.php?latex=C" target="_blank"><img src="https://latex.codecogs.com/gif.latex?C" title="C" /></a>点在大地坐标系<a href="https://www.codecogs.com/eqnedit.php?latex=XOY" target="_blank"><img src="https://latex.codecogs.com/gif.latex?XOY" title="XOY" /></a>下坐标为<a href="https://www.codecogs.com/eqnedit.php?latex=(x,y)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(x,y)" title="(x,y)" /></a>,机器的瞬时线速度为<a href="https://www.codecogs.com/eqnedit.php?latex=v_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_c" title="v_c" /></a>,瞬时角速度为<a href="https://www.codecogs.com/eqnedit.php?latex=\omega_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega_c" title="\omega_c" /></a>，姿态角<a href="https://www.codecogs.com/eqnedit.php?latex=\theta" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta" title="\theta" /></a>即为<a href="https://www.codecogs.com/eqnedit.php?latex=v_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_c" title="v_c" /></a>与<a href="https://www.codecogs.com/eqnedit.php?latex=X" target="_blank"><img src="https://latex.codecogs.com/gif.latex?X" title="X" /></a>轴夹角。此时，机器的位姿信息可用矢量<a href="https://www.codecogs.com/eqnedit.php?latex=\boldsymbol{P}=\begin{bmatrix}x,y,\theta\end{bmatrix}^T" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\boldsymbol{P}=\begin{bmatrix}x,y,\theta\end{bmatrix}^T" title="\boldsymbol{P}=\begin{bmatrix}x,y,\theta\end{bmatrix}^T" /></a>表示。机器人瞬时线速度<a href="https://www.codecogs.com/eqnedit.php?latex=v_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_c" title="v_c" /></a>为：
<div align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=v_c=\frac{v_r&plus;v_l}{2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_c=\frac{v_r&plus;v_l}{2}" title="v_c=\frac{v_r+v_l}{2}" /></a>
</div>

令左右轮间距为<a href="https://www.codecogs.com/eqnedit.php?latex=l" target="_blank"><img src="https://latex.codecogs.com/gif.latex?l" title="l" /></a>,且机器瞬时旋转中心(ICR)为<a href="https://www.codecogs.com/eqnedit.php?latex=O_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?O_c" title="O_c" /></a>,转动半径即为<a href="https://www.codecogs.com/eqnedit.php?latex=C" target="_blank"><img src="https://latex.codecogs.com/gif.latex?C" title="C" /></a>到<a href="https://www.codecogs.com/eqnedit.php?latex=O_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?O_c" title="O_c" /></a>的距离<a href="https://www.codecogs.com/eqnedit.php?latex=R" target="_blank"><img src="https://latex.codecogs.com/gif.latex?R" title="R" /></a>。机器在做同轴（轴为左右轮到ICR连线）圆周（圆心为ICR）运动时，左右轮及基点所处位置在该圆周运动中的角速度相同<a href="https://www.codecogs.com/eqnedit.php?latex=\omega_l=\omega_r=\omega_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega_l=\omega_r=\omega_c" title="\omega_l=\omega_r=\omega_c" /></a>，到旋转中心的半径不同，有<a href="https://www.codecogs.com/eqnedit.php?latex=l=\frac{v_r}{\omega_r}-\frac{v_l}{\omega_l}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?l=\frac{v_r}{\omega_r}-\frac{v_l}{\omega_l}" title="l=\frac{v_r}{\omega_r}-\frac{v_l}{\omega_l}" /></a>。则机器的瞬时角速度<a href="https://www.codecogs.com/eqnedit.php?latex=\omega_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega_c" title="\omega_c" /></a>可以表示为:

<div align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=\omega_c=\frac{v_r-v_l}{l}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega_c=\frac{v_r-v_l}{l}" title="\omega_c=\frac{v_r-v_l}{l}" /></a>
</div>

联立两式求出转动半径：

<div align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=R=\frac{v_c}{\omega_c}=\frac{l}{2}\frac{v_r&plus;v_l}{v_r&plus;v_l}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?R=\frac{v_c}{\omega_c}=\frac{l}{2}\frac{v_r&plus;v_l}{v_r&plus;v_l}" title="R=\frac{v_c}{\omega_c}=\frac{l}{2}\frac{v_r+v_l}{v_r+v_l}" /></a>
</div>

### 1.1.1 三种运动状态分析

差速驱动方式，即V1和V2间存在的速度差关系决定了其具备不同的三种运动状态，如图所示：

<div align="center">
<img src="img/diff-vel-car-status.png">
</div>

* 当<a href="https://www.codecogs.com/eqnedit.php?latex=v_l>v_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_l>v_r" title="v_l>v_r" /></a>时，机器做圆弧运动；
* 当<a href="https://www.codecogs.com/eqnedit.php?latex=v_l=v_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_l=v_r" title="v_l=v_r" /></a>时，机器做直线运动；
* 当<a href="https://www.codecogs.com/eqnedit.php?latex=v_l=-v_r" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_l=-v_r" title="v_l=-v_r" /></a>时，机器以左右轮中心点做原地旋转。

## 1.2 函数模型

在驱动轮与地面接触运动为纯滚动无滑动情况下，机器的运动学模型可以表示为：

<div align="center">
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;\dot{x}\\&space;\dot{y}\\&space;\dot{\theta}&space;\end{bmatrix}=\begin{bmatrix}&space;cos\theta&space;&&space;0\\&space;cos\theta&space;&&space;0&space;\\&space;0&&space;1&space;\end{bmatrix}\begin{bmatrix}&space;\frac{1}{2}&space;&&space;\frac{1}{2}\\&space;\frac{1}{l}&space;&&space;-\frac{1}{2}&space;\end{bmatrix}\begin{bmatrix}&space;v_r\\&space;v_l&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;\dot{x}\\&space;\dot{y}\\&space;\dot{\theta}&space;\end{bmatrix}=\begin{bmatrix}&space;cos\theta&space;&&space;0\\&space;cos\theta&space;&&space;0&space;\\&space;0&&space;1&space;\end{bmatrix}\begin{bmatrix}&space;\frac{1}{2}&space;&&space;\frac{1}{2}\\&space;\frac{1}{l}&space;&&space;-\frac{1}{2}&space;\end{bmatrix}\begin{bmatrix}&space;v_r\\&space;v_l&space;\end{bmatrix}" title="\begin{bmatrix} \dot{x}\\ \dot{y}\\ \dot{\theta} \end{bmatrix}=\begin{bmatrix} cos\theta & 0\\ cos\theta & 0 \\ 0& 1 \end{bmatrix}\begin{bmatrix} \frac{1}{2} & \frac{1}{2}\\ \frac{1}{l} & -\frac{1}{2} \end{bmatrix}\begin{bmatrix} v_r\\ v_l \end{bmatrix}" /></a>
</div>

机器人坐标系服从右手定则，其顺时旋转针姿态角减少，逆时针旋转姿态角增加

## 1.3. 运动控制

运动控制的被控对象是所分析的两轮差速移动机器人；直观的控制量是上述建模中所述的左右轮转速，为了更一般的描述车体的运动，控制量选车体线速度<a href="https://www.codecogs.com/eqnedit.php?latex=v" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v" title="v" /></a>与角速度<a href="https://www.codecogs.com/eqnedit.php?latex=\omega" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega" title="\omega" /></a>，左右轮转速可由模型反求取。

### 1.3.1 点到点

控制机器人由当前点移动到指定目标点，其核心是令机器人在控制器作用下（以一定合适的方式）持续地朝向目标点运动。其控制系统框图为：

<div align="center">
<img src="img/p2p-control.png">
</div>

### 1.3.2 任意姿态到达目标点

设机器人实时位姿为<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;x_t&space;&&space;y_t&space;&&space;\theta_t&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;x_t&space;&&space;y_t&space;&&space;\theta_t&space;\end{bmatrix}" title="\begin{bmatrix} x_t & y_t & \theta_t \end{bmatrix}" /></a>，目标位置为<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;x_G&space;&&space;y_G&space;&&space;\theta_G&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;x_G&space;&&space;y_G&space;&&space;\theta_G&space;\end{bmatrix}" title="\begin{bmatrix} x_G & y_G & \theta_G \end{bmatrix}" /></a>，易求出机器与目标点间实时的距离差<a href="https://www.codecogs.com/eqnedit.php?latex=d_{err}=\sqrt{(x_G-x_t)^2&plus;(y_G-y_t)^2}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?d_{err}=\sqrt{(x_G-x_t)^2&plus;(y_G-y_t)^2}" title="d_{err}=\sqrt{(x_G-x_t)^2+(y_G-y_t)^2}" /></a>与角度差<a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{err}=tan(\frac{y_G-y_t}{x_G-x_t})-\theta_t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{err}=tan(\frac{y_G-y_t}{x_G-x_t})-\theta_t" title="\theta_{err}=tan(\frac{y_G-y_t}{x_G-x_t})-\theta_t" /></a>(即图中<a href="https://www.codecogs.com/eqnedit.php?latex=\delta=\varphi&space;-\theta" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\delta=\varphi&space;-\theta" title="\delta=\varphi -\theta" /></a>)，如下图所示

<div align="center">
<img src="img/p2p-car-model.png">
</div>

一种简单通用的控制器，PID控制器可以构成运动控制器。其设计方法是：简单地，运动控制器由两个并联的PID控制器组成：

* 一个PID控制器，输入为距离差<a href="https://www.codecogs.com/eqnedit.php?latex=d_{err}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?d_{err}" title="d_{err}" /></a>输入为线速度<a href="https://www.codecogs.com/eqnedit.php?latex=v" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v" title="v" /></a>，即距离决定速度。距离远速度大，距离近速度小。【多说一句，这里要注意的是速度的连续性，（如起步时距离远，线速度由0跳变为较大值，且距离无穷远速度无穷大）所以需要将**PID输出先限幅再平滑**(即限幅<a href="https://www.codecogs.com/eqnedit.php?latex=\left&space;\|&space;v_{pidout}&space;\right&space;\|\leq&space;v_{max}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\left&space;\|&space;v_{pidout}&space;\right&space;\|\leq&space;v_{max}" title="\left \| v_{pidout} \right \|\leq v_{max}" /></a>,平滑<a href="https://www.codecogs.com/eqnedit.php?latex=\left&space;\|v_{t-1}-&space;v_{pidout}&space;\right&space;\|\leq&space;\sigma" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\left&space;\|v_{t-1}-&space;v_{pidout}&space;\right&space;\|\leq&space;\sigma" title="\left \|v_{t-1}- v_{pidout} \right \|\leq \sigma" /></a>,最后输出<a href="https://www.codecogs.com/eqnedit.php?latex=v_t=v_{pidout}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_t=v_{pidout}" title="v_t=v_{pidout}" /></a>)

* 另一个PID控制器，输入为距离差<a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{err}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{err}" title="\theta_{err}" /></a>，输出为角速度<a href="https://www.codecogs.com/eqnedit.php?latex=\omega" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega" title="\omega" /></a>即偏角误差决定转速。正偏左转，负偏右转；偏多转快，偏少转慢。【多说一句，这里要注意的是角度表示方式带来的过界问题，即当求出的<a href="https://www.codecogs.com/eqnedit.php?latex=\theta_{err}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta_{err}" title="\theta_{err}" /></a>大于或小于<a href="https://www.codecogs.com/eqnedit.php?latex=\pi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\pi" title="\pi" /></a>(如<a href="https://www.codecogs.com/eqnedit.php?latex=\varphi&space;=\frac{3\pi}{4}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\varphi&space;=\frac{3\pi}{4}" title="\varphi =\frac{3\pi}{4}" /></a>，<a href="https://www.codecogs.com/eqnedit.php?latex=\theta&space;=-\frac{3\pi}{4}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta&space;=-\frac{3\pi}{4}" title="\theta =-\frac{3\pi}{4}" /></a>)时，要将其归一化到<a href="https://www.codecogs.com/eqnedit.php?latex=(-\pi,\pi]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?(-\pi,\pi]" title="(-\pi,\pi]" /></a>,即<a href="https://www.codecogs.com/eqnedit.php?latex={\color{Yellow}&space;\pi-2\pi\leq&space;\theta_{err}\leq&space;-\pi&plus;2\pi}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?{\color{Red}&space;\pi-2\pi\leq&space;\theta_{err}\leq&space;-\pi&plus;2\pi}" title="{\color{Yellow} \pi-2\pi\leq \theta_{err}\leq -\pi+2\pi}" /></a>】

使用上述公式求出线速度<a href="https://www.codecogs.com/eqnedit.php?latex=v_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_c" title="v_c" /></a>、角速度<a href="https://www.codecogs.com/eqnedit.php?latex=\omega_c" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\omega_c" title="\omega_c" /></a>后，再解算为左右轮转速交给机器执行。控制器会控制机器人持续朝向目标移动，当其距离目标小于一定值，即移动到以目标为圆心，<a href="https://www.codecogs.com/eqnedit.php?latex=\tau" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\tau" title="\tau" /></a>为半径（目标半径）的圆内<a href="https://www.codecogs.com/eqnedit.php?latex=d_{err}\leq&space;\tau" target="_blank"><img src="https://latex.codecogs.com/gif.latex?d_{err}\leq&space;\tau" title="d_{err}\leq \tau" /></a>时，判定机器人到达目标位置，完成运动控制过程。

上述点到点的运动控制只要求到达目标点，并没有其他约束条件，虽然在实际应用中往往会有许多约束，但是其确是一切运动控制的基石。

在某些具体应用中，如泊车parking，对接docking等往往需要机器人以某个固定的姿态到达某一具体位置。


# 2. 两轮差速底盘的运动模型分析：运动控制与里程计解算

参考[博客](https://blog.csdn.net/xingdou520/article/details/83691951)

<div align="center">
<img src="img/diff-vel-car1.jpg">
<img src="img/diff-vel-car2.jpg">
<img src="img/diff-vel-car3.jpg">
<img src="img/diff-vel-car4.jpg">
<img src="img/diff-vel-car5.jpg">
<div align="center">
