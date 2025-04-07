"""Tests for qp_solver functionality."""
import tests
import pytheia as pt
import numpy as np
import scipy.sparse as sp


def test_unbounded():
    """
    Python implementation of C++ test: QPSolver.Unbounded
    
    Tests an unbounded QP minimization problem with a known output.
    """
    tolerance = 1e-4
    
    # Define the quadratic programming problem:
    # 1/2 * x' * P * x + q' * x + r
    #     [  5  -2  -1 ]
    # P = [ -2   4   3 ]
    #     [ -1   3   5 ]
    #
    # q = [ 2  -35  -47 ]^T
    # r = 5
    P = np.array([[5, -2, -1], 
                  [-2, 4, 3], 
                  [-1, 3, 5]])
    q = np.array([2, -35, -47])
    r = 5
    
    # Convert to sparse matrix
    P_sparse = sp.csr_matrix(P)
    
    # Create solver options
    options = pt.math.QPSolverOptions()
    options.max_num_iterations = 100
    
    # Create QP solver
    qp_solver = pt.math.QPSolver(options, P_sparse, q, r)
    
    # Solve the problem
    solution = np.zeros(3)
    result = qp_solver.Solve(solution)
    
    assert result is True
    
    # Verify the solution is near (3, 5, 7)
    gt_solution = np.array([3, 5, 7])
    for i in range(3):
        assert abs(solution[i] - gt_solution[i]) < tolerance
    
    # Check that the residual is near optimal
    residual = 0.5 * solution.dot(P.dot(solution)) + solution.dot(q) + r
    gt_residual = 0.5 * gt_solution.dot(P.dot(gt_solution)) + gt_solution.dot(q) + r
    assert abs(residual - gt_residual) < tolerance


def test_loosebounds():
    """
    Python implementation of C++ test: QPSolver.LooseBounds
    
    Tests a QP minimization with loose bounds that shouldn't affect the solution.
    """
    tolerance = 1e-4
    
    # Define the quadratic programming problem (same as unbounded)
    P = np.array([[5, -2, -1], 
                  [-2, 4, 3], 
                  [-1, 3, 5]])
    q = np.array([2, -35, -47])
    r = 5
    
    # Convert to sparse matrix
    P_sparse = sp.csr_matrix(P)
    
    # Create solver options
    options = pt.math.QPSolverOptions()
    options.max_num_iterations = 100
    
    # Create QP solver
    qp_solver = pt.math.QPSolver(options, P_sparse, q, r)
    
    # Set bounds that shouldn't affect the output
    lower_bound = np.array([0, 0, 0])
    upper_bound = np.array([10, 10, 10])
    qp_solver.SetLowerBound(lower_bound)
    qp_solver.SetUpperBound(upper_bound)
    
    # Solve the problem
    solution = np.zeros(3)
    result = qp_solver.Solve(solution)
    
    assert result is True
    
    # Verify the solution is near (3, 5, 7)
    gt_solution = np.array([3, 5, 7])
    for i in range(3):
        assert abs(solution[i] - gt_solution[i]) < tolerance
    
    # Check that the residual is near optimal
    residual = 0.5 * solution.dot(P.dot(solution)) + solution.dot(q) + r
    gt_residual = 0.5 * gt_solution.dot(P.dot(gt_solution)) + gt_solution.dot(q) + r
    assert abs(residual - gt_residual) < tolerance


def test_tightbounds():
    """
    Python implementation of C++ test: QPSolver.TightBounds
    
    Tests a QP minimization with tight bounds that constrain the solution.
    """
    tolerance = 1e-4
    
    # Define the quadratic programming problem (same as unbounded)
    P = np.array([[5, -2, -1], 
                  [-2, 4, 3], 
                  [-1, 3, 5]])
    q = np.array([2, -35, -47])
    r = 5
    
    # Convert to sparse matrix
    P_sparse = sp.csr_matrix(P)
    
    # Create solver options
    options = pt.math.QPSolverOptions()
    options.absolute_tolerance = 1e-8
    options.relative_tolerance = 1e-8
    
    # Create QP solver
    qp_solver = pt.math.QPSolver(options, P_sparse, q, r)
    
    # Set bounds that constrain the output
    lower_bound = np.array([5, 7, 9])
    upper_bound = np.array([10, 12, 14])
    qp_solver.SetLowerBound(lower_bound)
    qp_solver.SetUpperBound(upper_bound)
    
    # Solve the problem
    solution = np.zeros(3)
    result = qp_solver.Solve(solution)
    
    assert result is True
    
    # Verify the solution is near (5, 7, 9)
    gt_solution = np.array([5, 7, 9])
    for i in range(3):
        assert abs(solution[i] - gt_solution[i]) < tolerance
    
    # Check that the residual is near optimal
    residual = 0.5 * solution.dot(P.dot(solution)) + solution.dot(q) + r
    gt_residual = 0.5 * gt_solution.dot(P.dot(gt_solution)) + gt_solution.dot(q) + r
    assert abs(residual - gt_residual) < tolerance


def test_invalidbounds():
    """
    Python implementation of C++ test: QPSolver.InvalidBounds
    
    Tests a QP minimization with invalid bounds where lower_bound > upper_bound.
    """
    # Define the quadratic programming problem (same as unbounded)
    P = np.array([[5, -2, -1], 
                  [-2, 4, 3], 
                  [-1, 3, 5]])
    q = np.array([2, -35, -47])
    r = 5
    
    # Convert to sparse matrix
    P_sparse = sp.csr_matrix(P)
    
    # Create solver options
    options = pt.math.QPSolverOptions()
    
    # Create QP solver
    qp_solver = pt.math.QPSolver(options, P_sparse, q, r)
    
    # Set the upper bound as the lower bound
    lower_bound = np.array([5, 7, 9])
    qp_solver.SetUpperBound(lower_bound)
    
    # Set the lower bound as the upper bound, making the valid solution space non-existent
    upper_bound = np.array([10, 12, 14])
    qp_solver.SetLowerBound(upper_bound)
    
    # Solve the problem
    solution = np.zeros(3)
    result = qp_solver.Solve(solution)
    
    # With invalid bounds, the solver should return False
    assert result is False
