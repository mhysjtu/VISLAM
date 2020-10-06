## 5.1在inertial系统中为什么要用ESKF
- EKSF(Error state kalman filter)，也叫indirect kalman filter
- 对IMU做积分即为dead-reckoning位置系统，会随着时间漂移;为了避免漂移，就要用绝对位置信息(GPS或视觉)来融合信息
- ESKF就是一种融合绝对位置信息来解决漂移问题的方法
- ESKF相比KF有以下良好的性质：
  - IMU相关应用里, 常用四元数表示旋转. 四元数的error-state是旋转向量, 是3维的, 避免了自由度的冗余(over-parametrization)以及由此带来的协方差矩阵的奇异性,也避免了在滤波或优化中引入额外的约束(例如四元书模长为1, 旋转矩阵为正交阵)
  - error-state总是工作在接近于0的附近,也就远离了可能的参数奇异问题、万向锁问题等，保证了线性化是一直有效的
  - error-state很小, 所以二阶项可以忽略,给求雅克比带来了便利
  - error的变化很缓慢, 不像nominal state会有大幅度的变化(这个大幅变化已经体现在nominal state积分过程中了)。所以在实际应用中KF correction的频率可以远低于KF prediction. 例如IMU的频率高于相机, 只有在有图像输入,即观测到视觉误差时才进行 correction, 而更多没有视觉输入的时刻仅仅进行prediction(即IMU propagation)

## 5.2 ESKF框架简介
- ESKF中有true-state, nominal-state和error-state；true-state是nominal-state和error-state的合适组合。这样做的思想是，把nominal-state看作大的信号(以非线性形式进行积分)，把error-state看作小的信号(因此可以进行线性积分，适合于线性高斯滤波器)
1. 在滤波的prediction部分, 高频的IMU观测信号um被积分到nominal-state x中, 这个过程不考虑噪声项w, 也不考虑系统的不准确性。与此同时, 对error-state及其协方差进行积分, 这个部分考虑了所有的噪声与系统误差。到这里仅仅是predict, 因为还没有IMU之外的信息来矫正这个error。 
2. 直到GPS或者Vision信号到来，进行correction(update),这些信号使得误差可观且频率低于积分。correction部分提供了对error-state的后验高斯估计。
3. 然后error-state的均值会被叠加到nominal-state, 得到新的估计值。之后将error-state置0,同时更新误差状态的协防差矩阵。
4. 重复1和2(1的频率高于2)。