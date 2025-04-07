"""Tests for l1_solver functionality."""
import tests
import pytheia as pt
import numpy as np


def test_smallproblem():
    """
    Python implementation of C++ test: L1Solver.SmallProblem
    
    Tests a small L1 minimization problem with a known output.
    """
    tolerance = 1e-8
    
    # A rigged L1 minimization problem with a known output.
    # [  1  2  3 ]         [ 1 ]
    # [  4  5  5 ] * [x] = [ 2 ]
    # [  7  8  9 ]         [ 3 ]
    # [ 10 11 23 ]         [ 4 ]
    #
    # For this problem, the L1 minimization should result in
    # x = [-0.75, 1.0, 0.0]^t.
    
    # Set up the LHS matrix
    lhs = np.zeros((4, 3))
    for i in range(4):
        for j in range(3):
            lhs[i, j] = i * 4 + j
    lhs[1, 2] = 5
    
    # Set up the RHS vector
    rhs = np.zeros(4)
    for i in range(4):
        rhs[i] = i + 1
    
    # Create solution vector
    solution = np.zeros(3)
    
    # Create solver options
    options = pt.math.L1SolverOptions()
    options.max_num_iterations = 100
    
    # Create and run the L1 solver
    l1_solver = pt.math.L1Solver(options, lhs)
    l1_solver.Solve(rhs, solution)
    
    # Verify the solution is near (-0.75, 1.0, 0.0)
    gt_solution = np.array([-0.75, 1.0, 0.0])
    for i in range(3):
        assert abs(solution[i] - gt_solution[i]) < tolerance
    
    # Check that the residual is near zero
    residual = lhs.dot(solution) - rhs
    for i in range(residual.size):
        assert abs(residual[i]) < tolerance


def test_decoding():
    """
    Python implementation of C++ test: L1Solver.Decoding
    
    Tests a more complex L1 minimization problem formulated as a codeword recovery problem.
    This example is taken from the L1-magic library.
    """
    tolerance = 1e-8
    
    # Set a specific seed for deterministic testing
    np.random.seed(94)
    
    # Define problem dimensions
    source_length = 256
    codeword_length = 4 * source_length
    num_perturbations = int(0.2 * codeword_length)
    
    # Create random matrix and source word
    mat = np.random.random((codeword_length, source_length))
    source_word = np.random.random(source_length)
    code_word = mat.dot(source_word)
    
    # Apply random perturbations to create observation
    observation = code_word.copy()
    perturbations = 0.5 * np.random.random(num_perturbations)
    for i in range(num_perturbations):
        rand_entry = np.random.randint(0, observation.size)
        observation[rand_entry] = perturbations[i]
    
    # Create solver options
    options = pt.math.L1SolverOptions()
    options.absolute_tolerance = 1e-8
    options.relative_tolerance = 1e-8
    
    # Create and run the L1 solver
    l1_solver = pt.math.L1Solver(options, mat)
    
    # Set the initial noisy guess
    mat_t_mat_inv = np.linalg.inv(mat.T.dot(mat))
    solution = mat_t_mat_inv.dot(mat.T).dot(observation)
    
    # Solve the L1 minimization problem
    l1_solver.Solve(observation, solution)
    
    # Verify the solution
    residual = mat.dot(solution) - code_word
    for i in range(residual.size):
        assert abs(residual[i]) < tolerance
