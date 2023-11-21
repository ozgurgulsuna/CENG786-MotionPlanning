## Potential Functions ##

**Definition**: A [[potential function]] is a differential valued function $U: \mathbb{R}^n \rightarrow \mathbb{R}$ such that $U(x) \geq 0$ and $U(x) = 0$ if and only if $x = x^*$. 

The [[gradient]] is defined as $\nabla U(x) = [\frac{\partial U(x)}{\partial x_1}, \frac{\partial U(x)}{\partial x_2}, \dots, \frac{\partial U(x)}{\partial x_n}]^T$. Now use the gradient to define a [[vector field]] $W(x) = -\nabla U(x)$.

The systems we are interested in are first order systems of the form $\dot{x} = f(x)$, where $f: \mathbb{R}^n \rightarrow \mathbb{R}^n$ is a [[vector field]]. The gradient of a potential function is a vector field. The _gradient descent_ algorithm is a method for finding the minimum of a function. The gradient descent algorithm is defined as $\dot{x} = -\nabla U(x)$.

The motion is terminated when $U(x) \leq \epsilon$ for some $\epsilon > 0$.

The second derivative of this function is called the [[Hessian]] matrix, and is denoted by $H(x)$. The Hessian matrix is a square matrix of second-order partial derivatives of the function. The Hessian matrix is defined as $H(x) = \begin{bmatrix} \frac{\partial^2 U(x)}{\partial x_1^2} & \frac{\partial^2 U(x)}{\partial x_1 \partial x_2} & \dots & \frac{\partial^2 U(x)}{\partial x_1 \partial x_n} \\ \frac{\partial^2 U(x)}{\partial x_2 \partial x_1} & \frac{\partial^2 U(x)}{\partial x_2^2} & \dots & \frac{\partial^2 U(x)}{\partial x_2 \partial x_n} \\ \vdots & \vdots & \ddots & \vdots \\ \frac{\partial^2 U(x)}{\partial x_n \partial x_1} & \frac{\partial^2 U(x)}{\partial x_n \partial x_2} & \dots & \frac{\partial^2 U(x)}{\partial x_n^2} \end{bmatrix}$

When the Hessian is nonsingular at the critical point, the critical point is non-degenerate, implying that the critical point is isolated. When the Hessian is positive-definite, the critical point is a local minimum; when the Hessian is negative-definite, then the critical point is a local maximum. Generally, we consider potential functions whose Hessians are nonsingular, i.e., those which only have isolated critical points. This also means that the potential function is never flat.

###### Potential Functions ######
## Additive Attractive/Repulsive Potential ##

The simplest [[potential function]] is the additive attractive/repulsive potential. The intuition behind is the goal attracts the robot while the obstacles repel it. The sum of these effects is the total potential function.
This potential function is defined as,
$\begin{equation}
U(x) = U_{att}(x) + U_{rep}(x)
\end{equation}$

#### Attactive Potential ####


------------------------------------------------------------------------------------------
#CENG786 - [[Robot Motion Planning and Control]] at [[METU]]