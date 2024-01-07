## \[Proporsal-3\]: Obstacle Displacement Motion Planning ##
_\[Real-Time Navigation for Autonomous Surface Vehicles In Ice-Covered Waters\]_  

#minimizing-collision #obstacle-displacement #motion-planning #motion-primitives #moving-horizon #lattice-based-planner

   In crowded environments where the obstacles also dynamic, an effective planning can be made with pushing the objects to the sides. A preliminary example of this setting is movement of the water vehicles in ice-covered waters, where the 2D ice blocks are the dynamic obstacles. The complexity of the problem is minimizing the displacement of the dynamic obstacles, the prior work focuses on the collision-free path planning for the vehicles. The related paper proposes a method to minimize the displacement of the ice blocks while planning the path for the vehicle. 
   
   The methodology includes a familiar A* path planner over a graph of motion primitives, which is a lattice-based planner. For the interactions with the dynamic obstacles and the vehicle an existing collision model is adopted which uses a cost function minimizing the _kinetic energy_. Lastly the objective is defined such that the goal is a forward progress over the channel.

  The problem is formulated over a 2D environment with a simplified ship footprint. The simulations took part in the Python environment with the 2D physics library "Pymunk" is used for the collisions.

   The paper is a good example of the motion planning in a setting featuring a significantly high concentrations of obstacles, where the obstacles are also dynamic. This environment can adopt to many other applications for example a human crowd in a public space or a person moving around in a room with chairs scattered around. The challenges are mostly the dynamic simulations and the collision model. The 2D nature of the problem can be extended to 3D with a similar approach.

\[1\] [Real-Time Navigation for Autonomous Surface Vehicles In Ice-Covered Waters](https://arxiv.org/pdf/2302.11601.pdf)  
\[2\] [Computational Tradeoff in Minimum Obstacle Displacement Planning for Robot Navigation](https://arxiv.org/pdf/2302.07114.pdf)  

## ##
<h7>
<div dir="rtl">Ozgur Gulsuna, 2307668</div>
<div dir="rtl">2023-11-19</div>
</h7>


