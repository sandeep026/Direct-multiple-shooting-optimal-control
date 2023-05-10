# Numerical optimal control- Multiple-shooting

An optimal control problem for a cart system is solved using multiple shooting method. The optimal control problem is discretized and transformed to a nonlinear optimization problem and solved using the IPOPT solver.

# Requirements
1. MATLAB/[OCTAVE](https://octave.org/)
2. [Casadi](https://web.casadi.org/)

# Problem description

The optimal control problem for the cart system [^1] is provided below.The states are z<sub>1</sub> and z<sub>2</sub>, which are the the position and velocity of the cart, respectively. f is the force applied and there is a drag force which is proportional to the velocity of the cart. The system starts from rest and an additional boundary condition is placed at the end of the trajectory. Along the trajectory the control effort is minimized from time 0 to 2.

![image](https://user-images.githubusercontent.com/16457676/236567436-9d87b891-e74f-4299-802c-a394693c1f60.png)

# Analytical solution

The system admits the following analytical solution, which can be later used to verify the numerical solution and its accuracy.

![image](https://user-images.githubusercontent.com/16457676/236629178-b6da4837-b1d8-454d-9ec4-2d67fb1abeba.png)

# Discretization

The time domain is discretized and a piecewise constant control is assumed over each discretization interval. Similarly, the states are dicretized at the node points and over each interval, the dynamics are integrated forward in time using a suitable numerical method like Euler foward, ruge kutta 45 etc. To achieve continuity of the states at the junction of each interval, equality constraints are placed. Similarly the integral in the objective function is discretized using a riemann sum or a sutable method. The nonlinear constraints are only evaluated at the grid points. Collectively, this results in a nonlinear optimization problem which can be solved using a off-the-shelf solver. Here, we haveused IPOPT for finding the optimal solution. The following figure [^2] illustrates the method, where q/s represents the discrete control inputs and x the state. The discrete state and contro inputs form the decision variables for the optimization.

![image](https://github.com/sandeep026/Numerical-optimal-control--Multiple-shooting/assets/16457676/59597e12-634f-4109-b0d4-8c7bf85d1a11)

# Results

The results are plotted in phase space for a grid size of 200 and they are in close agreement with the analytical solution.

![image](https://github.com/sandeep026/Numerical-optimal-control--Multiple-shooting/assets/16457676/00d8680b-3f4f-422e-bca6-dab1da1e73ed)

![image](https://github.com/sandeep026/Numerical-optimal-control--Multiple-shooting/assets/16457676/074208a9-480e-4f6d-ab04-e684db9e88c3)

|Method|Optimal objective value|
|---|---|
|Analytical|0.577678|
|Numerical|0.5777|

# Advantages

1. Unlike single shooting method, the states are also treated as decision variables. In addition there are continuity constraints which leads a large and sparse optimization problem. By exploiting suitable linear algebra libraries, sparsity can be exploited and can be efficiently solved. 
2. In addition, the nonlinearities are evenly distributed among variables which also improves the convergence of the solver. This can be also viewed as a lifted version of the single shooting method.

# Known issues

1. TBA 

# References

[^1]: Conway, B. A. and K. Larson (1998). Collocation versus differential inclusion in direct optimization. Journal of Guidance, Control, and Dynamics, 21(5), 780–785

[^2]: Diehl, M., & Gros, S. (2011). Numerical optimal control. Optimization in Engineering Center (OPTEC).

[^3]: Albersmeyer, J., & Diehl, M. (2010). The lifted Newton method and its application in optimization. SIAM Journal on Optimization, 20(3), 1655-1684.
