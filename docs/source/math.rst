.. highlight:: c++

.. default-domain:: cpp

.. _documentation-math:

====
Math
====

At the root of computer vision is a heavy amount of math and probability. Theia contains various math functions implemented with a generic interface for ease of use.

.. _section-closed_form_poly:

Closed Form Polynomial Solver
=============================

Many problems in vision rely on solving a polynomial quickly. For small degrees
(n <= 4) this can be done in closed form, making them exceptionally fast. We
have implemented solvers for these cases.

.. function:: int SolveQuadraticReals(double a, double b, double c, double* roots)

.. function:: int SolveQuadratic(double a, double b, double c, std::complex<double>* roots)

  Provides solutions to the equation :math:`a*x^2 + b*x + c = 0`

.. note:: The closed form solutions for cubic and quartic solvers are known to be numerically unstable. We recommend using the generic polynomial solvers (below) instead. This will sacrifice efficiency a small amount for a significant improvement in solution quality.

.. function::  int SolveCubicReals(double a, double b, double c, double d, double* roots)

.. function::  int SolveCubic(double a, double b, double c, double d, std::complex<double>* roots)

   Provides solutions to the equation :math:`a*x^3 + b*x^2 + c*x + d = 0` using `Cardano's <http://en.wikipedia.org/wiki/Cubic_function#Cardano.27s_method>`_ method.


.. function::  int SolveQuarticReals(double a, double b, double c, double d, double e, double* roots)

.. function::  int SolveQuartic(double a, double b, double c, double d, double e, std::complex<double>* roots)

  Provides solutions to the equation :math:`a*x^4 + b*x^3 + c*x^2 + d*x + e = 0` using `Ferrari's method <http://en.wikipedia.org/wiki/Quartic_function#Ferrari.27s_solution>`_ to reduce to problem to a depressed cubic.


.. _section-generic_poly:

Generic Polynomial Solver
=========================

For polynomials of degree > 4 there are no closed-form solutions, making the
problem of finding roots much more difficult. However, we have implemented
several functions that will solve for polynomial roots. For all polynomials we
require that the largest degree appears first and the smallest degree appears
last in the input VectorXd such that:

.. math:: \sum_{i=0}^N p(i) x^{N-i}

Where :math:`p(i)` is the input VectorXd.

.. function:: bool FindPolynomialRoots(const Eigen::VectorXd& polynomial, Eigen::VectorXd* real, Eigen::VectorXd* imaginary)

  This function finds the roots of the input polynomial using one of the methods
  below. All methods in Theia that require finding polynomial roots use this
  method. This is so that we can easily change the default root-finding method
  of choice (i.e. Companion Matrix to Jenkins-Traub, etc.) by modifying this
  function once instead of modify every instance where we want to find
  polynomial roots. This allows us to easily swap in new polynomial root-solvers
  (that may be more efficient or numerically stable) as they are implemented.

.. function:: bool FindPolynomialRootsJenkinsTraub(const Eigen::VectorXd& polynomial, Eigen::VectorXd* real, Eigen::VectorXd* imaginary)

  The `Jenkins Traub algorithm <https://en.wikipedia.org/wiki/Companion_matrix>`_
  is a three-stage algorithm for finding roots of polynomials with real
  coefficients as outlined in [JenkinsTraub]_. Please note that
  this variant is different than the complex-coefficient version, and is
  estimated to be up to 4 times faster.

  The algorithm works by computing shifts in so-called "K-polynomials" that
  deflate the polynomial to reveal the roots. Once a root is found (or in the
  real-polynomial case, a pair of roots) then it is divided from the polynomial
  and the process is repeated. This method is consider to be "pratically a
  standard in black-box polynomial root finder" (Numerical Recipes 2007) and is
  based on the `Rpoly++ <http://github.com/sweeneychris/RpolyPlusPlus>`_ implementation.

.. function:: bool FindPolynomialRootsCompanionMatrix(const Eigen::VectorXd& polynomial, Eigen::VectorXd* real, Eigen::VectorXd* imaginary)

  Roots are computed using the `Companion Matrix <https://en.wikipedia.org/wiki/Companion_matrix>`_ with balancing to help improve
  the condition of the matrix system we solve. This is a reliable, stable method
  for computing roots but is most often the slowest method.

.. function:: double FindRootIterativeLaguere(const Eigen::VectorXd& polynomial, const double x0, const double epsilon, const int max_iter)

  Finds a single polynomials root iteratively based on the starting position :math:`x_0` and
  guaranteed precision of epsilon using `Laguerre's Method <https://en.wikipedia.org/wiki/Laguerre%27s_method>`_.

.. function:: double FindRootIterativeNewton(const Eigen::VectorXd& polynomial, const double x0, const double epsilon, const int max_iter)

  Finds a single polynomials root iteratively based on the starting position :math:`x_0` and
  guaranteed precision of epsilon using `Newton's Method <https://en.wikipedia.org/wiki/Newton%27s_method>`_.

.. _section-matrix_methods:

Matrix Methods
==============

Theia implements many useful linear algebra methods including optimizations, factorizations, and utility methods.

.. class:: L1Solver

  We implement a robust :math:`L_1` solver that minimizes :math:`||Ax - b||_1`
  using the Alternating Direction Method of Multipliers (ADMM). This solver is particularly 
  efficient as it only needs to factorize the system matrix once using Cholesky decomposition.
  The solver converges quickly to good solutions in early iterations while subsequent iterations
  refine the solution to achieve global optimality.

.. member:: double L1Solver::Options::max_num_iterations

  DEFAULT: ``100``

  The maximum number of iterations to perform before stopping.

.. member:: double L1Solver::Options::rho

  DEFAULT: ``1.0``
  
  The augmented Lagrangian parameter used in ADMM.

.. member:: double L1Solver::Options::alpha

  DEFAULT: ``1.0``
  
  Over-relaxation parameter (typically between 1.0 and 1.8).

.. class:: QPSolver

  Implements an efficient quadratic program solver that minimizes:
  
  .. math:: \frac{1}{2} x^T P x + q^T x + r
  
  subject to bound constraints :math:`lb \leq x \leq ub`. The solver uses ADMM which provides
  fast convergence to good solutions while maintaining numerical stability through Cholesky
  factorization of the system matrix.

.. member:: QPSolver::Options::max_num_iterations

  DEFAULT: ``1000``
  
  Maximum number of ADMM iterations.

.. member:: QPSolver::Options::rho 

  DEFAULT: ``1.0``
  
  Augmented Lagrangian parameter.

.. member:: QPSolver::Options::alpha

  DEFAULT: ``1.0``
  
  Over-relaxation parameter (typically between 1.0 and 1.8).

Both solvers handle sparse matrices efficiently through specialized Cholesky factorization
implementations. The matrix operations are built on top of the Eigen library providing
both dense and sparse matrix support.

.. code:: c++

  // Example L1 solver usage
  L1Solver::Options options;
  Eigen::MatrixXd A;
  Eigen::VectorXd b, x;
  // Fill A and b with known values
  
  L1Solver<Eigen::MatrixXd> l1_solver(options, A);
  l1_solver.Solve(b, &x);
  // x now contains the solution that minimizes ||Ax - b|| under L1 norm

  // Example QP solver usage  
  QPSolver::Options qp_options;
  Eigen::SparseMatrix<double> P;
  Eigen::VectorXd q;
  double r;
  // Set up P, q, r
  
  QPSolver qp_solver(qp_options, P, q, r);
  Eigen::VectorXd solution;
  qp_solver.Solve(&solution);

.. _section-sprt:

Sequential Probability Ratio Test
=================================

Modified version of Wald's `SPRT <http://en.wikipedia.org/wiki/Sequential_probability_ratio_test>`_ as [Matas]_ et. al. implement it in "Randomized
RANSAC with Sequential Probability Ratio Test"

.. function:: double CalculateSPRTDecisionThreshold(double sigma, double epsilon, double time_compute_model_ratio = 200.0, int num_models_verified = 1)

 ``sigma``: Probability of rejecting a good model (Bernoulli parameter).

 ``epsilon``: Inlier ratio.

 ``time_compute_model_ratio``: Computing the model parameters from a sample takes the same time as verification of time_compute_model_ratio data points. Matas et. al. use 200.

 ``num_model_verified``: Number of models that are verified per sample.

 ``Returns``:  The SPRT decision threshold based on the input parameters.


.. function:: bool SequentialProbabilityRatioTest(const std::vector<double>& residuals, double error_thresh, double sigma, double epsilon, double decision_threshold, int* num_tested_points, double* observed_inlier_ratio)

 Modified version of Wald's SPRT as [Matas]_ et. al. implement it in "Randomized
 RANSAC with Sequential Probability Ratio Test". See the paper for more
 details.

 ``residuals``: Error residuals to use for SPRT analysis.

 ``error_thresh``: Error threshold for determining when Datum fits the model.

 ``sigma``: Probability of rejecting a good model.

 ``epsilon``: Inlier ratio.

 ``decision_threshold``: The decision threshold at which to terminate.

 ``observed_inlier_ratio``: Output parameter of inlier ratio tested.
