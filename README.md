# Uncertainty aware Local Navigation for Multiagent Systems
Motion planning for multiagent system is a challenging problem with a wide range of applications from warehouse robotics to social iteraction modelling. In this project we will explore three different algorithms that can used to achieve local navigation for multiagent systems under uncertain environments

## Robot setup
The agents/robots are modelled as disks of mass $m$ and radius $r$ and can move freely in the x-y plane accoding to $m\ddot{x}=a$. This characterisitc is similar to omnidirectional robots. The robots have a local sensing radius $d_h$ to detect obstacles and neighbors. Further, each robot has a max force $f_{max}$ and a max velocity $v_{max}$ constraint. The goal location $x_g$ is assumed to be known. 

## Navigation problem
The goal for each robot is to navigate to the goal while avoiding other robots in the environment. Hence the motion planning problem can be decomposed into following a global goal velocity $v_g$ while locally avoiding obstacles. The equations of motion for the robot $i$ is given as follows

$\dot{x}_i = v_i$

$\dot{v}_i = F_i/m_i$

## Algorithm 1: Social Force Model (SFM)$^{[1]}$
![SFM_3agents](https://github.com/sriram-2502/uncertainty_aware_navigation/blob/master/gif/SFM_3.gif)

SFM is a reactive force based approach to local navigation. The control input to the robot is a reactive force where the robot follows an attractive force to reach the goal while locally reacts to the its neighbors/obstalces within its sensing radius. 

The control input $F_i$ for agent $i$ is computed as follows:

$F_i = F_{attractive} + F_{repulsive}$

$F_{attractive} = m (v_g - v)/\xi$ $\rightarrow$ handles global attraction towards the goal

$F_{repulsive} = F_{normal} + F_{tangential}$ $\rightarrow$ handles local repulsion to neighbors/obstalces

The normal force $F_{normal}$ is a sum of a repulsive force and a body forces which counteracts compression between two agents (inspired by crowd behaviour)

$F_{normal} = \{A_i\exp[(r_{ij}-d_{ij})/B_i] + kg(r_{ij}-d_{ij})\}\mathbf{n}_{ij}$

The tangential force $F_{tangential}$ models the force due to sliding friction experienced by the robot

$F_{tangential} = \kappa g(r_{ij}-d_{ij})\Delta_{ji}^t \mathbf{t}_{ij}$

$r_{ij} = r_i + r_j$ denotes the sum of disk radius of robot $i$ and $j$, $d_{ij} = ||\mathbf{r}_i -\mathbf{r}_j||$ is the distance between the robot's center of mass $\mathbf{r}_i, \mathbf{r}_j$, $\mathbf{n}_{ij}$ is the normal vector pointing from robot $i$ to $j$, $\Delta_{ji}^t = (\mathbf{v}_j - \mathbf{v}_i) \mathbf{t}_{ij}$ is the tangential velocity difference, 
$A_i,B_i, k$ and $\kappa$ are parameters 

### Simulation setup
parameters used in this simulation
* $m = 80$
* $r_i = 2$
* $d_h = 10$
* $f_{max} = 800$
* $\xi = 5$
* $A_i = 2000$
* $B_i = 0.08$
* $k = 1.2\times 10^5$
* $\kappa = 2.4 \times 10^5$



### Advantages
* The SFM approach is relatively fast and simple to implement when simulating large number robots/agents such as a crowd behaviour
* The control law accounts for various physcal phenomenon experienced by the robot such as body forces and friction
* It is physically intuitive and easy to modify to generate desired behaviour

### Disadvantages
* It is reactive in nature and hence the robots tend to generate a reactive force when there is a neighbor in its sensing raidus even when there is no collision!
* The robot mimics a particle like behavior leading to non smooth control inputs
* Cannot guarantee collision-free motion

## Algorithm 2: Time to Collision (TTC)$^{[2]}$
TTC is a predcitive force based approach to local navigation. The control to the robot is a reactive force. However, unline SFM, the repulsive forces are calculated by predciting future collisions

The control input $F_i$ for agent $i$ is computed as follows:

$F_i = F_{attractive} + F_{repulsive}$

$F_{attractive} = m (v_g - v)/\xi$ $\rightarrow$ handles global attraction towards the goal

$F_{repulsive} = \frac{\max(\tau_H-\tau,0)}{\tau} \mathbf{n}$ $\rightarrow$ handles local repulsion when based on predcited time to collision $\tau$ within a time horizon $\tau_H$

$\mathbf{n} = \frac{x+v\tau}{||x+v\tau||}$ is the unit vector that pushes two agents apart with relative position $x$ and relative velocity $v$ at the moment of impact $\tau$. 

Note: $\tau$ is calcualted assuming agents move in a linear fashion leading upto collision

### Simulation setup
parameters used in this simulation
* $m = 1$
* $r_i = 2$
* $d_h = 10$
* $f_{max} = 10$
* $\xi = 0.5$
* $\tau_H = 4$

### Advantages
* The TTC approach is also relavitely fast and can very easy to implement.
* The control law is predictive in nature and leads to a smooth avoidance motion 
* The robots behave in a natural way while avoidance neighbours/obstalces

### Disadvantages
* The prediction model assumes that its neibhours/obstacles move linearly 
* Very small simulation timestep $\Delta t$ for numerical stability
* Cannot guarantee collision-free motion

## Algorithm 3: Velocity Obstalces (VO)$^{[3]}$
VO is a predcitive velocity based approach where the 

## References
[1] Helbing, Dirk, Illés Farkas, and Tamas Vicsek. "Simulating dynamical features of escape panic." Nature 407.6803 (2000): 487-490.

[2] Davis, Bobby, Ioannis Karamouzas, and Stephen J. Guy. "NH-TTC: A gradient-based framework for generalized anticipatory collision avoidance."
