# Introduction

This repository contains a C++ implementation for Arduino of the DCM (Direction Cosine Matrix) algorithm for estimating the Euler Angles (pitch, roll and yaw) of a body in a three dimensional space. Note that you will need to feed this algorithm with data from three different sensors: accelerometer, gyroscope and magnetometer. Each sensor has to measure the given magnitude for each one of the three axis (x, y and z), so 9 values are needed to use this library. Note that this algorithm is quite sensitive to calibration, so you must calibrate your measurements before feeding the algorithm with them, specially the magnetometer ones.

Note that this library is totally independant from the sensors (both software and hardware) that is used. Select your favourite sensors, read data from them and call the update function provided in this repository.

In the following secton some information about DCM algorithm is presented, but without going to much in detail. If you want to know more about the existing maths under the hood, feel free to check the references in the last section, specially [1] which contains a C implementation of the algorithm and has been used as a reference for this implementation.

# Example
<p align="center">
  <img width="360" height="200" src="https://github.com/alrevuelta/dcm-algorithm-cpp/blob/master/img/demo.gif">
</p>

# DCM Algorithm

The DCM algothim can estimate the pitch, roll and yaw using a DCM (Direction Cosine Matrix), taking into account the drift. A PI control system is used to correct the drift, where P corresponds to the proportional term and I to the integrative one. This drift correction makes this algothim quite suitable for many applications. 


TODO Figure DCM
<p align="center">
  <img width="200" height="200" src="xxx">
</p>


The algorithm can be divided in the following steps, assuming that the input data is calibrated:
* Initialize DCM: This part is only for the first iteration. It will initialize the DCM matrix with the first available data.
* Calculate heading: Calculate the heading using the magnetometer.
* Update matrix: Update the DCM matrix.
* Normalize: Keep the vectors orthogonal and normalize them.
* Correct drift: Correct the drift of the gyroscope using a PI control.
* Calculate Euler: Calculate Euler angles from DCM matrix.

## Init DCM

The first step is to initialize the DCM matrix. This is done by using the following expression:

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?R=&space;\begin{pmatrix}&space;c(\phi)c(\theta)&space;&&space;c(\phi)s(\theta)s(\psi)&space;-&space;s(\phi)c(\psi)&space;&&space;c(\phi)s(\theta)c(\psi)&space;&plus;&space;s(\phi)s(\psi)\\&space;s(\phi)c(\theta)&space;&&space;s(\phi)s(\theta)s(\psi)&space;&plus;&space;c(\phi)c(\psi)&space;&&space;s(\phi)s(\theta)c(\psi)&space;-&space;c(\phi)s(\psi)\\&space;-s(\theta)&space;&&space;c(\theta)s(\psi)&space;&&space;c(\theta)c(\psi)\\&space;\end{pmatrix}">
</p>

Remember that this rotation matrix assumes the ZYX convention explained before. Note that vec is the unitary vector in x direction, A is a vector that contains the accelerometer calibrated values and time is the cross product.


<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?aux&space;=&space;\vec{u}_x&space;\times&space;(A_{cal}&space;\times&space;\vec{u}_x)">
</p>

Using that auxiliary vector, the roll can be calculated as follows. Note that aux_y and aux_z are the second and third components of the vector.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?roll&space;=&space;\arctan&space;\left(&space;\dfrac{aux_y}{aux_z}&space;\right)">
</p>

Last step to feed the matrix, is calculating the yaw.
 

## Calculate heading

In each iteration of the algorithm, the heading value (also referred as yaw) is calculated. Calibrated values of the magnetometer are used. Note that c(), s() is equivalent to \cos(), \sin().

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?yaw&space;=&space;\arctan&space;\left(&space;\dfrac{-M_Yc(roll)&space;&plus;&space;M_Zs(roll)}{M_Xc(pitch)&space;&plus;&space;M_Ys(roll)s(pitch)&space;&plus;&space;M_Zc(roll)s(pitch)}&space;\right)">
</p>


## Update matrix

Next step is to update the DCM matrix with the values of Omega_I, Omega_P calculated in the last iteration. These values will be zero in the first iteration, because the are calculated in next steps.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\Omega&space;=&space;G&space;&plus;&space;\Omega_I&space;&plus;&space;\Omega_P">
</p>

To understand the following steps, is necessary to explain some facts of the rotation matrix. Having calibrated gyroscope data G that after drift correction using, the rotation matrix becomes.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?R(t&plus;dt)&space;=&space;R(t)\begin{pmatrix}&space;1&space;&&space;-\Omega_z&space;dt&space;&&space;\Omega_y&space;dt\\&space;\Omega_z&space;dt&space;&&space;1&space;&&space;-\Omega_x&space;dt\\&space;-\Omega_y&space;dt&space;&&space;\Omega_x&space;dt&space;&&space;1\\&space;\end{pmatrix}">
</p>

So in each step, the rotation matrix will be updated using the read gyroscope values and corrected according to the PI controller that will be explained then.

## Normalize

This step is very important for the performance of the algorithm. By definition a rotation matrix is orthogonal, which means that R^tR = RR^t = I. The problem is that numerical errors can keep accumulating and break the orthogonality condition, which will result in a bad performance because the matrix will no longer represent a rotation.

In order to avoid this, in each iteration two steps are done: first make the vectors orthogonal again, second normalize them. The Gram-Schmidt method for orthogonalization is a very well known process, but for this purpose a different approach is taken. The used method does not fix one of the vector and calculated the others. Both vectors are changed, applying to each one half of the error.

First step is to compute the X and Y orthogonal vectors of the rotation matrix. This can be done by using the following:
<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?X_{orthogonal}&space;=&space;X&space;-&space;\dfrac{error}{2}Y">
</p>

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?Y_{orthogonal}&space;=&space;Y&space;-&space;\dfrac{error}{2}X">
</p>

The error is calculated as the dot product of X dot Y. Remember that the dot product of a dot b is calculated as |a||b| \cos(\theta), where $\theta$ is the angle between both vectors. If they were orthogonal, cos(theta) would be zero and the error would be also zero.

Next step is to calculate the Z orthogonal vector to X and Y. This is done by calculating the cross product between X and Y X times Y. Remember the geometric representation of the cross product.

TODO Figure DCM
<p align="center">
  <img width="200" height="200" src="xxx">
</p>

The last step is to re normalize the calculated vectors $X,Y,Z$. This step is trivial and can be done by dividing the vector by the Euclidean norm. However, a different approach presented in [5] is used. This way assumes that the norm is not much greater than one, so Taylor's expansion can be used. The norm of a vector $\vec{u}$ using this approach is shown in here:

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\vec{u}_{norm}&space;=&space;\dfrac{1}{2}&space;(3&space;-&space;\vec{u}&space;\cdot&space;\vec{u})&space;\vec{u}">
</p>


## Correct drift

Next step is to correct the drift of the gyroscope. The accelerometer will be used to correct the pitch and roll values, and the magnetometer to correct the yaw or heading. For correcting the drift, a PI controller will be used where P stands for proportional and I for integrative. A PI controller is similar to a PID controller, which is a very well known control system.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?y(t)&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int&space;e(t)&space;dt&space;&plus;&space;K_d&space;\dfrac{de}{dt}">
</p>

A PI control will only have the proportional and integrative term. In the case of study of this project there will be two different controllers: one for the pitch/roll and other for the yaw. The constants corresponding to the integral and proportional term are:
* KP_pr: Proportional constant for pitch and roll
* KI_pr: Integral constant for pitch and roll
* KP_y: Proportional constant for yaw.
* KI_y: Integral constant for yaw.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?y(t)&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int&space;e(t)&space;dt">
</p>

First step is to calculate the error in pitch and roll. This can be calculated using the cross product between the acceleration calibrated vector $A_{cal}$ and the last row of the DCM matrix DCM_{31}, DCM_{32}, DCM_{33}.
<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?error_{pitch/roll}&space;=&space;A_{cal}&space;\times&space;\begin{pmatrix}&space;DCM_{31}&space;DCM_{32}&space;DCM_{33}\\&space;\end{pmatrix}">
</p>

Next step is to calculate the correction terms Omega_P, Omega_I that will be used to correct the gyroscope measurements. Note that |\vec{A_{cal}}| is the Euclidean norm of the acceleration vector. It is recommended to constrain this value between 0 and 1. This will reduce the error when the sensor is under big accelerations, i.e. moving it with the hand.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\Omega_P&space;=&space;error_{pitch/roll}&space;K^P_{pitch/roll}&space;|\vec{A_{cal}}|">
</p>

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\Omega_I&space;=&space;\Omega_I&space;&plus;&space;error_{pitch/roll}&space;K^I_{pitch/roll}&space;|\vec{A_{cal}}|">
</p>

Now it is time to correct the yaw drift using the yaw obtained by the magnetometer.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?error_{yaw}&space;=&space;\begin{pmatrix}&space;DCM_{31}&space;DCM_{32}&space;DCM_{33}\\&space;\end{pmatrix}&space;\left(&space;DCM_{11}\sin(yaw)&space;-&space;DCM_{21}&space;\cos(yaw)&space;\right)">
</p>

And now it is time to add the contribution of the yaw to Omega_I, Omega_P.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\Omega_P&space;=&space;\Omega_P&space;&plus;&space;error_{yaw}&space;K^P_{yaw}">
</p>

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\Omega_I&space;=&space;\Omega_I&space;&plus;&space;error_{yaw}&space;K^I_{yaw}">
</p>

In the next iteration this values will be used to correct the drift of the gyroscope values.

## Calculate Euler Angles

Once this step has reached, there is nothing much left. Just calculate the Euler angles from the updated rotation matrix. This can be calculated using the following expression. After this section the algorithm will run from the beginning again.

<p align="center">
  <img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}&space;pitch&space;\\&space;roll&space;\\&space;yaw&space;\\&space;\end{pmatrix}=&space;\begin{pmatrix}&space;-\arcsin&space;\left(&space;R_{31}&space;\right)&space;\\&space;\arctan&space;\left(&space;\frac{R_{31}}{R_{33}}\right)&space;\\&space;\arctan&space;\left(&space;\frac{R_{21}}{R_{11}}\right)&space;\\&space;\end{pmatrix}">
</p>

# References
* [1] https://github.com/Razor-AHRS/razor-9dof-ahrs/
* [2] http://www.mdpi.com/1424-8220/15/3/7016/pdf
* [3] http://www.ti.com/lit/an/slaa518a/slaa518a.pdf
* [4] http://www.starlino.com/dcm_tutorial.html
* [5] http://www.academia.edu/11778144/Direction_Cosine_Matrix_IMU_Theory
