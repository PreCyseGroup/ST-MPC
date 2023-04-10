# Dual-model Set-Theoretic MPC (ST-MPC) in a polyhedral framework.

This code shows how to implement, in a polyhedral framework, the dual-mode receding horizon controller developed in [[Angeli et al., Automatica, 2008]](https://www.sciencedirect.com/science/article/abs/pii/S0005109808003014).  Note that the mentioned article developed the control framework assuming an ellipsoidal modelling for disturbances and constraints.  On the other hand, the proposed implementation is for a polyhedral framework (i.e., constraints and disturbance sets are polyhedral sets).  Nevertheless, the underlying set-theoretic MPC concepts and the used MPC philosophy are the same.  The implemented algorithm is described in, e.g., [[Lucia et al., CONES, 2022], Section II.A.](https://ieeexplore.ieee.org/document/9795085)

## Considered plant model and polyhedral framework 
It is assumed that the plant is described by a LTI model subject to state and input constraints, as well as bounded state disturbances.  All the constraints and disturbances sets are modelled as convex polyhedral sets.

## What does ST-MPC?  And how does it work? 
ST-MPC solves a regulation problem for linear systems subject to state constraints, input constriants and bounded disturbances. 

Most of the required computations are moved into an offline phase, leaving online a simple and convex optimization problem. 

**OFFLINE**, the controller is built resorting to three main ingredients:
- A stabilizing state-feedback controller.  In the proposed implementation, such a controller is a standard LQR (u=Kx).
- The smallest Robust Positively Invariant (RPI) region associated with the closed-loop system and complying with the prescribed constraints and disturbances.
- A family of Robust One-step Controllable Sets (ROSC) (also known as Robust Backward Reachable Sets) recursively built starting from the smallest RPI region.  The recursive computation of ROSC sets can be terminated when the desired state-space region of initial conditions is covered, or the set's growth saturates (for the presence of constraints and disturbances).  Note that the RPI region is associated with an index = 0, while the ROSC sets have an index (>=1) increasing with the number of iterations.

**ONLINE**, the controller takes the following steps:
- Find the set with the smallest index containing the current state. 
- If the set index is zero, then the terminal controller (u=Kx) is used. 
- Else, the control input u is obtained solving a convex constrained optimization problem which imposes that the robust one-step evolution of the system goes into a set whose index which associated index is one unit smaller than the current one. Such an optimization defines a standard MPC optimization problem where the prediction horizon is 1.

### ST-MPC properties
- The strategy is recursively feasible. In other words, if the strategy admits a solution at t=0, then an admissible solution will be found for any successive iteration of the algorithm.
- For any admissible initial state condition, the state trajectory reaches the RPI region in a finite number of steps where it remains confined.

## For which kind of control problem ST-MPC is useful ?
In short, ST-MPC is suitable for applications requiring a computationally low demanding controller capable of dealing with hard constraints and disturbances.

In the literature, the ST-MPC idea (or extentions of it) has been used to solve (or partially solve) different control problems (partial list):
- Control of autonomous vehicles: [[AV 1]](https://ieeexplore.ieee.org/abstract/document/9992658), [[AV 2]](https://link.springer.com/article/10.1007/s10626-020-00337-7), [[AV 3]](https://ieeexplore.ieee.org/abstract/document/9468311), [[AV 4]](https://ieeexplore.ieee.org/abstract/document/9206381), [[AV 5]](https://ieeexplore.ieee.org/abstract/document/7987748), [[AV 6]](https://www.sciencedirect.com/science/article/pii/S0167691114002758?casa_token=QuUdNUynXZYAAAAA:2PwL_cFVxIHRhRKJzqMcmmri6Glp7KNkFJGly8VRCFkC8VtO0zR9YAAA70M1oLSyEpt1AAMn)
- Data-driven MPC control: [[Data-Driven 1]](https://arxiv.org/abs/2303.04749) ([github](https://github.com/PreCyseGroup/Data-Driven-ST-MPC))
- Cyber-physical systems security and privacy: [[CPS 1]](https://www.sciencedirect.com/science/article/abs/pii/S0005109823001097), [[CPS 2]](https://ieeexplore.ieee.org/abstract/document/9887874), [[CPS 3]](https://ieeexplore.ieee.org/abstract/document/9838573), [[CPS 4]](https://ieeexplore.ieee.org/abstract/document/9795085), [[CPS 5]](https://ieeexplore.ieee.org/abstract/document/9794322), [[CPS 6]](https://ieeexplore.ieee.org/abstract/document/9670717), [[CPS 7]](https://ieeexplore.ieee.org/abstract/document/9442797), [[CPS 8]](https://ieeexplore.ieee.org/abstract/document/9235508)
- Fault-detection and mitigation: [[FD 1]](https://ieeexplore.ieee.org/document/8080208)
- Control of swithcing and switched systems: [SW 1](https://onlinelibrary.wiley.com/doi/abs/10.1002/acs.2804), 

## Matlab implementation and required packages
This code has been developed and tested for Matlab R2020b.  It assumes that the [MPT3 toolbox](https://www.mpt3.org/) (including the Yalmip package) is correctly installed in Matlab.

## Example in the code
The example is developed by assuming a toy mass-spring-damper model (2 states, 1 input) subject to state and input constraints and bounded disturbances. 

## Run the simulation
- Run the file "main.m"
- When requested, press any button to go from the offline phase to the online phase.  The request to press a button will appear (at the end of the offline phase) in both the title of the current Figure and in the Command Window

### Expected simulation outcome (figures)

![ROSC_SETSandTrajectory](https://user-images.githubusercontent.com/127126601/230968736-fd7bcdc2-f44a-4d66-bdd6-61a7fe6a83b6.png)

![Input_constraints](https://user-images.githubusercontent.com/127126601/230968747-572bc2b4-603f-4752-b33c-9dc7a23f3af3.png)

![SetMembership_Index](https://user-images.githubusercontent.com/127126601/230968894-56d87ce3-846e-4281-b3ed-d91aac4819ad.png)




