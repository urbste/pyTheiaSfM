# Math {#documentation-math}

At the root of computer vision is a heavy amount of math and probability. Theia implements a broad C++ numerics layer; pyTheia exposes **only a small subset** to Python—see below.

## Python API (`pytheia.math`) {#math-python-api}

Bindings live in [`math.cc`](https://github.com/urbste/pyTheiaSfM/blob/master/src/pytheia/math/math.cc).

### Free functions

| Symbol | Role |
|--------|------|
| `FindQuadraticPolynomialRoots(polynomial, real, imaginary)` | Quadratic roots in Theia coefficient order $p_0 x^2 + p_1 x + p_2$ (see [`polynomial.h`](https://github.com/urbste/pyTheiaSfM/blob/master/src/theia/math/polynomial.h)). Writes real and imaginary parts into the output vectors (same pattern as the C++ `Eigen::VectorXd*` out-parameters). |
| `AlignRotations(rotations, gt_rotations)` | In-place: solves for a shared rotation $R$ so stored angle-axis rotations align with ground truth (see [Transformations](transformations.md#transformations-align-rotations)). |
| `AlignOrientations(rotations_map, gt_map)` | Same idea using `dict[int, array(3)]` view-id → angle-axis; returns aligned map. |
| `MultiplyRotations(r1, r2)` | Composition $R_1 R_2$ in angle-axis form. |
| `RelativeRotationFromTwoRotations(r1, r2)` | $R_{12} = R_2 R_1^\top$. |
| `ApplyRelativeRotation(r12, r1)` | $R_2 = R_{12} R_1$. |
| `RelativeTranslationFromTwoPositions(R1, p1, p2)` | $t_{12} = R_1 (p_2 - p_1)$. |
| `Sim3FromRotationTranslationScale(R, t, s)` | Build `Sim3d` from matrix rotation, translation, scale. |
| `SE3FromRotationTranslation(R, t)` | Build `SE3d`. |

### Types and submodule

- **`SE3d`**, **`Sim3d`**: Sophus rigid and similarity transforms (matrix, translation, `exp` / `log`, `inverse`, products, helpers such as `rot_x` / `trans`, etc.).
- **`pt.math.Sophus`**: re-exports `SE3d` and `Sim3d` for namespaced access.

Rotation alignment behavior and usage examples are covered on [Transformations](transformations.md).

## C++ library layout (`src/theia/math`) {#math-cpp-layout}

These headers power Theia’s solvers and pipelines; **most are not bound to Python**.

| Area | Headers (representative) | Purpose |
|------|---------------------------|---------|
| Polynomials | `polynomial.h`, `closed_form_polynomial_solver.h`, `find_polynomial_roots_*.h` | Closed-form and generic root finding, Jenkins–Traub, companion matrix, Newton/Laguerre steps |
| Rotation / geometry | `rotation.h` | Angle-axis alignment, relative rotation utilities (partially in Python) |
| $L_1$ / optimization | `l1_solver.h`, `constrained_l1_solver.h`, `qp_solver.h` | $L_1$ minimization, QP interfaces |
| Linear algebra | `matrix/linear_operator.h`, `matrix/sparse_*.h`, `matrix/rq_decomposition.h`, `matrix/gauss_jordan.h`, `matrix/matrix_square_root.h`, `nullspace.h` | Operators, sparse Cholesky, RQ, Gauss–Jordan, square root |
| Graph | `graph/connected_components.h`, `graph/minimum_spanning_tree.h`, `graph/normalized_graph_cut.h`, `graph/triplet_extractor.h` | Graph algorithms used in SfM |
| Probability / sampling | `probability/sequential_probability_ratio.h`, `reservoir_sampler.h`, `distribution.h`, `histogram.h` | SPRT (RANSAC), sampling, distributions |
| SDP / rank restrictions | `sdp_solver.h`, `bcm_sdp_solver.h`, `rbr_sdp_solver.h`, `rank_restricted_sdp_solver.h`, `riemannian_staircase.h` | Convex relaxations for rotation estimation |
| Utilities | `util.h`, `solver_options.h`, `solver_summary.h` | Shared helpers |

For **SPRT**, **Gauss–Jordan**, **$L_1$ solver**, and **generic polynomial** APIs, refer to the sections below (C++ oriented).

---

## Closed Form Polynomial Solver {#section-closed_form_poly}

Many problems in vision rely on solving a polynomial quickly. For small degrees (n \<= 4) this can be done in closed form, making them exceptionally fast. We have implemented solvers for these cases.

**`int SolveQuadraticReals(double a, double b, double c, double\* roots)`**

**`int SolveQuadratic(double a, double b, double c, std::complex\<double\>\* roots)`**

Provides solutions to the equation $a*x^2 + b*x + c = 0$

!!! note "Note"

    The closed form solutions for cubic and quartic solvers are known to be numerically unstable. We recommend using the generic polynomial solvers (below) instead. This will sacrifice efficiency a small amount for a significant improvement in solution quality.

**`int SolveCubicReals(double a, double b, double c, double d, double\* roots)`**

**`int SolveCubic(double a, double b, double c, double d, std::complex\<double\>\* roots)`**

Provides solutions to the equation $a*x^3 + b*x^2 + c*x + d = 0$ using [Cardano's](http://en.wikipedia.org/wiki/Cubic_function#Cardano.27s_method) method.

**`int SolveQuarticReals(double a, double b, double c, double d, double e, double\* roots)`**

**`int SolveQuartic(double a, double b, double c, double d, double e, std::complex\<double\>\* roots)`**

Provides solutions to the equation $a*x^4 + b*x^3 + c*x^2 + d*x + e = 0$ using [Ferrari's method](http://en.wikipedia.org/wiki/Quartic_function#Ferrari.27s_solution) to reduce to problem to a depressed cubic.

## Generic Polynomial Solver {#section-generic_poly}

For polynomials of degree \> 4 there are no closed-form solutions, making the problem of finding roots much more difficult. However, we have implemented several functions that will solve for polynomial roots. For all polynomials we require that the largest degree appears first and the smallest degree appears last in the input VectorXd such that:

$$\sum_{i=0}^N p(i) x^{N-i}$$

Where $p(i)$ is the input VectorXd.

**`bool FindPolynomialRoots(const Eigen::VectorXd& polynomial, Eigen::VectorXd\* real, Eigen::VectorXd\* imaginary)`**

This function finds the roots of the input polynomial using one of the methods below. All methods in Theia that require finding polynomial roots use this method. This is so that we can easily change the default root-finding method of choice (i.e. Companion Matrix to Jenkins-Traub, etc.) by modifying this function once instead of modify every instance where we want to find polynomial roots. This allows us to easily swap in new polynomial root-solvers (that may be more efficient or numerically stable) as they are implemented.

**`bool FindPolynomialRootsJenkinsTraub(const Eigen::VectorXd& polynomial, Eigen::VectorXd\* real, Eigen::VectorXd\* imaginary)`**

The [Jenkins Traub algorithm](https://en.wikipedia.org/wiki/Companion_matrix) is a three-stage algorithm for finding roots of polynomials with real coefficients as outlined in [JenkinsTraub](bibliography.md#JenkinsTraub). Please note that this variant is different than the complex-coefficient version, and is estimated to be up to 4 times faster.

The algorithm works by computing shifts in so-called "K-polynomials" that deflate the polynomial to reveal the roots. Once a root is found (or in the real-polynomial case, a pair of roots) then it is divided from the polynomial and the process is repeated. This method is consider to be "pratically a standard in black-box polynomial root finder" (Numerical Recipes 2007) and is based on the [Rpoly++](http://github.com/sweeneychris/RpolyPlusPlus) implementation.

**`bool FindPolynomialRootsCompanionMatrix(const Eigen::VectorXd& polynomial, Eigen::VectorXd\* real, Eigen::VectorXd\* imaginary)`**

Roots are computed using the [Companion Matrix](https://en.wikipedia.org/wiki/Companion_matrix) with balancing to help improve the condition of the matrix system we solve. This is a reliable, stable method for computing roots but is most often the slowest method.

**`double FindRootIterativeLaguere(const Eigen::VectorXd& polynomial, const double x0, const double epsilon, const int max_iter)`**

Finds a single polynomials root iteratively based on the starting position $x_0$ and guaranteed precision of epsilon using [Laguerre's Method](https://en.wikipedia.org/wiki/Laguerre%27s_method).

**`double FindRootIterativeNewton(const Eigen::VectorXd& polynomial, const double x0, const double epsilon, const int max_iter)`**

Finds a single polynomials root iteratively based on the starting position $x_0$ and guaranteed precision of epsilon using [Newton's Method](https://en.wikipedia.org/wiki/Newton%27s_method).

## Matrix Methods {#section-matrix_methods}

Theia implements many useful linear algebra methods including optimizations, factorizations, and utility methods.

We implement a robust $L_1$ solver that minimizes $||Ax - b||_1$ under $L_1$ norm. This problem may be cast as a simple and efficient linear program and solved with interior point methods. The interface is fairly generic and may be used with sparse or dense matrices. An initial guess is needed for $x$ to perform the minimization.

- double L1Solver::Options::max_num_iterations

DEFAULT: `100`

The maximum number of iterations to perform before stopping.

``` c++
Eigen::MatrixXd A;
Eigen::VectorXd b, x;
// Fill A and b with known values.

L1Solver::Options option;
L1Solver<Eigen::MatrixXd> l1_solver(options, A);
l1_solver.Solve(b, &x);
// x now contains the solution that minimizes ||Ax - b|| under L1 norm.
```

This class finds the dominant eigenvalue/eigenvector pair of a given matrix using [power iterations](https://en.wikipedia.org/wiki/Power_iteration). We use a generic interface that utilizes the `LinearOperator` class so that the user may determine who the dominant eigenvalues are computed. For instance, by passing the `SparseInveseLULinearOperator` the `DominantEigensolver` performs inverse power iterations and thus the smallest eigenvalue/eigenvector pair may be computed. This is useful for recovering a null space vector.

**`void GaussJordan(Eigen::MatrixBase\<Derived\>\* input, int max_rows = 99999)`**

Perform traditional Gauss-Jordan elimination on an Eigen3 matrix. If `max_rows` is specified, it will on perform Gauss-Jordan on the first `max_rows` number of rows. This is useful for problems where your system is extremely overdetermined and you do not need all rows to be solved.

## Sequential Probability Ratio Test {#section-sprt}

Modified version of Wald's [SPRT](http://en.wikipedia.org/wiki/Sequential_probability_ratio_test) as [Matas](bibliography.md#Matas) et. al. implement it in "Randomized RANSAC with Sequential Probability Ratio Test"

**`double CalculateSPRTDecisionThreshold(double sigma, double epsilon, double time_compute_model_ratio = 200.0, int num_models_verified = 1)`**

`sigma`: Probability of rejecting a good model (Bernoulli parameter).

`epsilon`: Inlier ratio.

`time_compute_model_ratio`: Computing the model parameters from a sample takes the same time as verification of time_compute_model_ratio data points. Matas et. al. use 200.

`num_model_verified`: Number of models that are verified per sample.

`Returns`: The SPRT decision threshold based on the input parameters.

**`bool SequentialProbabilityRatioTest(const std::vector\<double\>& residuals, double error_thresh, double sigma, double epsilon, double decision_threshold, int\* num_tested_points, double\* observed_inlier_ratio)`**

Modified version of Wald's SPRT as [Matas](bibliography.md#Matas) et. al. implement it in "Randomized RANSAC with Sequential Probability Ratio Test". See the paper for more details.

`residuals`: Error residuals to use for SPRT analysis.

`error_thresh`: Error threshold for determining when Datum fits the model.

`sigma`: Probability of rejecting a good model.

`epsilon`: Inlier ratio.

`decision_threshold`: The decision threshold at which to terminate.

`observed_inlier_ratio`: Output parameter of inlier ratio tested.

## Global rotation alignment {#math-align-rotations}

For **`AlignRotations`**, **`AlignOrientations`**, and related **angle-axis** helpers on **`pytheia.math`**, see [Transformations](transformations.md#transformations-align-rotations).
