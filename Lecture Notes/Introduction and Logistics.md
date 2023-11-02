## Introduction
### What is a robot?
A [[robot]] is a machine that can sense, think, and act.
 - Should be able to [[sense]] the environment.
 - Should be programmable, "intelligence" and autonomy are desired.
 - Should be able to "[[move]]" (i.e. do physical work).

### Motivation
Programming [[motion]] is tricky problem to define, and even trickier to solve. We need to understand the problem well, and we need to know the tools to solve it.

## Motion Planning and Control
Motion planning is to decide on what motions to go through to achieve a goal. It is not only a [[search]] problem, it is also about [[probabilistic]] reasoning, how to deal with [[uncertainty]].

Motion control is about how to execute the planned motions, yet it is not only about [[control theory]] or [[optimization]], it is also another search problem.

These two problems are tightly coupled, not always we need to first plan then act.

## Trends in Robotics/Motion Planning
```mermaid
%%{:root {--mermaid-theme:dark;}}%%
	flowchart
	A["<u><b>Classical Robotics (mid-70's)</u></b>
	- exact models
	- no sensing necessary"] -->
	  B["<u><b>Reactive Paradigm (mid-80's)</u></b>
	- no models
	- relies on good sensing"]
	  B --> C["<u><b>Hybrids (since 90's)</u></b>
	- model based at higher levels
	- reactive at lower levels"]
      B --> D["<u><b>Probabilistic Robotics (since mid-90's)</u></b>
    - seamless integration of sensing and model
    - inaccurate models, noisy sensing"]

```

## Mathematical Rigor

$$\begin{align*}
\begin{array}{cccc}
\text{Meaning} && \text{Symbol} && \text{Meaning} && \text{Symbol}  \\
\hline
\textrm{for all} && \forall && \textrm{there exists} && \exists \\
\textrm{infinity} && \infty && \textrm{such that} && \textrm{s.t.} \\
\textrm{element of} && \in && \textrm{not in} && \notin \\
\textrm{real numbers} && \mathbb{R} && \textrm{n dimensional real space} && \mathbb{R}^n \\
\textrm{union} && \cup && \textrm{intersection} && \cap \\
\textrm{subset} && \subseteq && \textrm{proper subset} && \subset \\
\textrm{empty set} && \emptyset && \textrm{set of integers} && \mathbb{Z} \\
\textrm{set difference} && \setminus && \textrm{set of natural numbers} && \mathbb{N} \\
\textrm{implies} && \Rightarrow && \textrm{if and only if} && \Leftrightarrow \\
\textrm{a circle} && S^1 && \textrm{a sphere} && S^2 \\
\textrm{gradient} && \nabla && \textrm{differential} && D \\
\textrm{distance to i$^{th}$ obstacle} && d_i && \textrm{distance between} && d(x,y) \\
\textrm{null space} && \textrm{Null} && \textrm{Jacobian} && J \\
\textrm{Christoffel symbols} && \Gamma && \textrm{Hessian} && H \\
\textrm{configuration space} && \mathcal{Q} && \textrm{workspace} && \mathcal{W} \\
\textrm{free space} && \mathcal{Q}_{free} && \textrm{obstacle space} && \mathcal{Q}_{obs} \\
\textrm{state at time k} && x(k) && \textrm{norm of vector} && ||x|| \\
\textrm{closure of set} && \overline{S} && \textrm{interior of set} && \textrm{int}(S) \\
\textrm{n-dimensonal torus space} && T^n && \textrm{unit n-ball} && B^n \\
\textrm{n-dimensional sphere in $\mathbb{R}^{n+1}$} && S^n && \textrm{unit n-cube} && C^n \\
\textrm{special orthogonal group} && SO(n) && \textrm{special Euclidean group} && SE(n) \\
\textrm{open ball of radius $\epsilon$} && B_{\epsilon}(x) && \textrm{closed ball of radius $\epsilon$} && \overline{B}_{\epsilon}(x) \\
\textrm{differential of $f$} && Df && \textrm{gradient of $f$} && \nabla f \\
\textrm{affine connection} && \nabla && \textrm{covariant derivative} && \nabla_X Y \\
\textrm{continuous function} && C^0 && \textrm{n times differentiable} && C^n \\
\textrm{inner product of $x$ and $y$} && \langle x, y \rangle && \textrm{identity matrix} && I \\
\textrm{tangent space of $M$ at $x$} && T_x M && \textrm{tangent bundle of $M$} && TM \\
\textrm{Lie bracket of fields $f$ and $g$} && [f,g] && \textrm{Lie algebra of set of vector fields $\mathcal{G}$} && \overline{Lie}(\mathcal{G}) \\
\textrm{involute closure of distribution $\mathcal{D}$} && \overline{\mathcal{D}} && \textrm{symmetric product of vector fields} && \langle Y_1:Y_2 \rangle \\
\textrm{}
\end{array}
\end{align*}$$

## Motion Planning Statement
Given a robot and its environment, find a path for the robot to move from a start configuration to a goal configuration, while avoiding obstacles.

In more formal setting, we can define the problem as follows:

If $W$ denotes the robot's workspace,
and $WO_i$ denotes the $i^{th}$ obstacle in the workspace,
then the robot's free space, $W_{free}$, is defined as:

$$
\begin{align*}
W_{free} := W \setminus \bigcup_{i=1}^n WO_i
\end{align*}
$$

and a path $ c \in C^0$ is $c: [0,1] \rightarrow W_{free}$ where $c(0) = q_{start}$ and $c(1) = q_{goal}$.

The goal of a motion planning [[algorithm]] is to find a feasible path in the free space that satisfies any given constraints or objectives, such as minimizing distance or avoiding certain regions of the [[workspace]].


-----
#CENG786 - [Robot Motion Control and Planning](Robot%20Motion%20Control%20and%20Planning.md) at [METU](METU.md)