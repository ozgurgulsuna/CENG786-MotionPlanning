## motion planning paper implementation ##

### conferences / journals  ###
- [T-RO](https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=8860) (IEEE Transactions on Robotics) [2021-2023]
- [ICRA](https://ieeexplore.ieee.org/xpl/conhome/1000639/all-proceedings) (IEEE International Conference on Robotics and Automation) [2021-2023]
- [IROS](https://ieeexplore.ieee.org/xpl/conhome/9981026/proceeding0640/all-proceedings) (IEEE/RSJ International Conference on Intelligent Robots and Systems) [2021-2022]
- [RSS](http://www.roboticsconference.org/) (Robotics: Science and Systems) [not skimmed yet]
- [RA-L](https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=7083369) (IEEE Robotics and Automation Letters) [2022-2023]

--------------------
## \[Proporsal-1\]: Truss Robot Motion Planning ##
#### \[P-1\] [Polygon-Based Random Tree Search Algorithm for a Size-Changing Robot](https://ieeexplore.ieee.org/document/10287401) ####

#computational-geometry #cellular-robots #modular-robots #shape-changing #polygonal-linkages #motion-planning #RRT #VGT #truss-robots #voronoi-diagram

  Variable geometry truss systems are a class of modular robots that can change their shape by changing the length of their links and passively rotating their joints. These type of robots can be constructed in a variety of shapes and complex structures, and can be used to perform a variety of tasks, including locomotion, manipulation, and self-reconfiguration. This very nature, being capable of various tasks makes them well-suited for applications in space exploration, where they can be used to traverse rough terrain and to adapt to changing environments.

  The motion planning problem for these robots can be defined as a search a sequence of link length changes that transforms the robot from an initial configuration to a goal configuration. This problem is challenging since the robot's configuration space is high-dimensional and non-convex. The related paper proposes a modified version of the RRT algorithm to solve this problem. The algorithm uses a varying-size polygon-based representation of the robot's configuration space to efficiently explore the tight spaces between obstacles. This state changing concept of the motion planning is the main interest  of the work. Roadmap method is included in the algorithm to find the characteristic points from the initial and goal configuration.

\[1\] [Polygon-Based Random Tree Search Algorithm for a Size-Changing Robot](https://ieeexplore.ieee.org/document/10287401)
\[2\] [Motion Planning for Variable Topology Trusses: Reconfiguration and Locomotion](https://ieeexplore.ieee.org/document/10287401)

includes kinematics, dynamics, and motion planning of the robot.





can we 




### [proporsal-1]: extras ###
- **continuum robots** (soft robots)
    - the problem is similar to above, the kinematics is different and simulation environment should be written, which presumed to be more complicated than the above.
    - [paper](/references/A_Cross-Entropy_Motion_Planning_Framework_for_Hybrid_Continuum_Robots.pdf)


--------------------
