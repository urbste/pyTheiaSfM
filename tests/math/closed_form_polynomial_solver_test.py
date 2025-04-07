"""Tests for closed_form_polynomial_solver functionality."""
import tests
import pytheia as pt
import numpy as np


def test_degeneratesolution():
    """
    Python implementation of C++ test: SolveQuadraticPolynomial.DegenerateSolution
    
    Tests solving a degenerate quadratic equation (where a = 0).
    """
    # - 2x + 1 = 0
    a = 0.0
    b = -2.0
    c = 1.0
    tolerance = 1e-12
    roots = np.zeros(2)
    num_roots = pt.math.SolveQuadraticReals(a, b, c, tolerance, roots)
    assert num_roots == 1
    assert abs(roots[0] - 0.5) < tolerance


def test_solvequadraticreals():
    """
    Python implementation of C++ test: SolveQuadraticPolynomial.SolveReals
    
    Tests solving quadratic equations with real roots.
    """
    # x^2 - 2x + 1 = 0
    a = 1.0
    b = -2.0
    c = 1.0
    tolerance = 1e-12
    roots = np.zeros(2)
    num_roots = pt.math.SolveQuadraticReals(a, b, c, tolerance, roots)
    assert num_roots == 2
    assert abs(roots[0] - 1.0) < tolerance
    assert abs(roots[1] - 1.0) < tolerance

    # x^2 - 11x + 30 = 0
    a = 1.0
    b = -11.0
    c = 30.0
    num_roots = pt.math.SolveQuadraticReals(a, b, c, tolerance, roots)
    assert num_roots == 2
    
    # Sort the roots for comparison
    sorted_roots = np.sort(roots[:num_roots])
    soln_roots = np.array([5.0, 6.0])
    
    for i in range(num_roots):
        assert abs(sorted_roots[i] - soln_roots[i]) < tolerance


def test_solvequadraticcomplex():
    """
    Python implementation of C++ test: SolveQuadraticPolynomial.SolveComplex
    
    Tests solving quadratic equations with complex roots.
    """
    # x^2 - 2x + 5 = 0 should yield 1 + 2i, 1 - 2i
    a = 1.0
    b = -2.0
    c = 5.0
    roots = np.zeros(2, dtype=np.complex128)
    num_roots = pt.math.SolveQuadratic(a, b, c, roots)
    
    assert num_roots == 2
    assert abs(roots[0].real - 1.0) < 1e-12
    assert abs(roots[1].real - 1.0) < 1e-12
    assert abs(abs(roots[0].imag) - 2.0) < 1e-12
    assert abs(abs(roots[1].imag) - 2.0) < 1e-12


def test_solvecubicreals():
    """
    Python implementation of C++ test: SolveCubicPolynomial.SolveReals
    
    Tests solving cubic equations with real roots.
    """
    # x^3 - 6x^2 + 11x - 6 = 0
    a = 1.0
    b = -6.0
    c = 11.0
    d = -6.0
    tolerance = 1e-12
    roots = np.zeros(3)
    
    num_roots = pt.math.SolveCubicReals(a, b, c, d, tolerance, roots)
    assert num_roots == 3
    
    # Sort the roots for comparison
    sorted_roots = np.sort(roots[:num_roots])
    soln_roots = np.array([1.0, 2.0, 3.0])
    
    for i in range(num_roots):
        assert abs(sorted_roots[i] - soln_roots[i]) < tolerance


def test_solvequarticreals():
    """
    Python implementation of C++ test: SolveQuarticPolynomial.SolveReals
    
    Tests solving quartic equations with real roots.
    """
    # y = 3x^4 + 6x^3 - 123x^2 - 126x + 1080 = 0
    a = 3.0
    b = 6.0
    c = -123.0
    d = -126.0
    e = 1080.0
    tolerance = 1e-12
    roots = np.zeros(4, dtype=np.float64)
    
    num_roots = pt.math.SolveQuarticReals(a, b, c, d, e, tolerance, roots)
    assert num_roots == 4
    
    # Sort the roots for comparison
    sorted_roots = np.sort(roots[:num_roots])
    soln_roots = np.array([-6.0, -4.0, 3.0, 5.0])
    
    for i in range(num_roots):
        assert abs(sorted_roots[i] - soln_roots[i]) < tolerance
