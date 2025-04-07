"""Tests for polynomial functionality."""
import tests
import pytheia as pt
import numpy as np


def test_findrootiterativelaguerretest():
    """
    Python implementation of C++ test: Polynomial.FindRootIterativeLaguerreTest
    
    Tests the Laguerre method for finding roots of a polynomial.
    """
    # (x - 3) * (x + 4) * (x + 5) * (x - 6) * (x + 7)
    polynomial = np.array([1.0, 7.0, -43.0, -319.0, 234.0, 2520.0])
    
    tolerance = 1e-10
    epsilon = 1e-8
    max_iter = 10
    
    # Test finding different roots with different initial guesses
    assert abs(pt.math.FindRootIterativeLaguerre(polynomial, 3.1, epsilon, max_iter) - 3.0) < tolerance
    assert abs(pt.math.FindRootIterativeLaguerre(polynomial, -4.1, epsilon, max_iter) - (-4.0)) < tolerance
    assert abs(pt.math.FindRootIterativeLaguerre(polynomial, -5.1, epsilon, max_iter) - (-5.0)) < tolerance
    assert abs(pt.math.FindRootIterativeLaguerre(polynomial, 6.1, epsilon, max_iter) - 6.0) < tolerance
    assert abs(pt.math.FindRootIterativeLaguerre(polynomial, -7.1, epsilon, max_iter) - (-7.0)) < tolerance


def test_findrootiterativenewtontest():
    """
    Python implementation of C++ test: Polynomial.FindRootIterativeNewtonTest
    
    Tests the Newton method for finding roots of a polynomial.
    """
    # (x - 3) * (x + 4) * (x + 5) * (x - 6) * (x + 7)
    polynomial = np.array([1.0, 7.0, -43.0, -319.0, 234.0, 2520.0])
    
    tolerance = 1e-10
    epsilon = 1e-8
    max_iter = 20
    
    # Test finding different roots with different initial guesses
    assert abs(pt.math.FindRootIterativeNewton(polynomial, 3.1, epsilon, max_iter) - 3.0) < tolerance
    assert abs(pt.math.FindRootIterativeNewton(polynomial, -4.1, epsilon, max_iter) - (-4.0)) < tolerance
    assert abs(pt.math.FindRootIterativeNewton(polynomial, -5.1, epsilon, max_iter) - (-5.0)) < tolerance
    assert abs(pt.math.FindRootIterativeNewton(polynomial, 6.1, epsilon, max_iter) - 6.0) < tolerance
    assert abs(pt.math.FindRootIterativeNewton(polynomial, -7.1, epsilon, max_iter) - (-7.0)) < tolerance


def test_differentiateconstantpolynomial():
    """
    Python implementation of C++ test: Polynomial.DifferentiateConstantPolynomial
    
    Tests differentiating a constant polynomial.
    """
    # p(x) = 1
    polynomial = np.array([1.0])
    
    derivative = pt.math.DifferentiatePolynomial(polynomial)
    assert derivative.shape[0] == 1
    assert derivative[0] == 0


def test_differentiatequadraticpolynomial():
    """
    Python implementation of C++ test: Polynomial.DifferentiateQuadraticPolynomial
    
    Tests differentiating a quadratic polynomial.
    """
    # p(x) = x^2 + 2x + 3
    polynomial = np.array([1.0, 2.0, 3.0])
    
    derivative = pt.math.DifferentiatePolynomial(polynomial)
    assert derivative.shape[0] == 2
    assert derivative[0] == 2.0
    assert derivative[1] == 2.0


def test_multiplypolynomials():
    """
    Python implementation of C++ test: Polynomial.MultiplyPolynomials
    
    Tests multiplying two polynomials.
    """
    # p1(x) = 2x^2 + x + 1
    poly1 = np.array([2.0, 1.0, 1.0])
    
    # p2(x) = 1
    poly2 = np.zeros(3)
    poly2[0] = 1.0
    
    multiplied_poly = pt.math.MultiplyPolynomials(poly1, poly2)
    
    # Expected result: 2x^2 + x + 1
    expected_poly = np.zeros(5)
    expected_poly[0] = 2.0
    expected_poly[1] = 1.0
    expected_poly[2] = 1.0
    
    tolerance = 1e-12
    assert np.all(np.abs(expected_poly - multiplied_poly) < tolerance)


def test_dividepolynomialsamedegree():
    """
    Python implementation of C++ test: Polynomial.DividePolynomialSameDegree
    
    Tests dividing two polynomials of the same degree.
    """
    # p1(x) = 2x^2 + x + 1
    poly1 = np.array([2.0, 1.0, 1.0])
    
    # p2(x) = 1
    poly2 = np.zeros(3)
    poly2[0] = 1.0
    
    quotient = np.zeros(3)
    remainder = np.zeros(2)
    
    pt.math.DividePolynomial(poly1, poly2, quotient, remainder)
    
    # Verify by reconstruction: poly1 = quotient * poly2 + remainder
    reconstructed_poly = pt.math.MultiplyPolynomials(quotient, poly2)
    reconstructed_poly[-remainder.shape[0]:] += remainder
    
    tolerance = 1e-12
    assert np.all(np.abs(poly1 - reconstructed_poly) < tolerance)


def test_dividepolynomiallowerdegree():
    """
    Python implementation of C++ test: Polynomial.DividePolynomialLowerDegree
    
    Tests dividing a polynomial by a lower degree polynomial.
    """
    # p1(x) = 2x^2 + x + 1
    poly1 = np.array([2.0, 1.0, 1.0])
    
    # p2(x) = 1
    poly2 = np.zeros(2)
    poly2[0] = 1.0
    
    quotient = np.zeros(2)
    remainder = np.zeros(1)
    
    pt.math.DividePolynomial(poly1, poly2, quotient, remainder)
    
    # Verify by reconstruction: poly1 = quotient * poly2 + remainder
    reconstructed_poly = pt.math.MultiplyPolynomials(quotient, poly2)
    reconstructed_poly[-remainder.shape[0]:] += remainder
    
    tolerance = 1e-12
    assert np.all(np.abs(poly1 - reconstructed_poly) < tolerance)


def test_dividepolynomialhigherdegree():
    """
    Python implementation of C++ test: Polynomial.DividePolynomialHigherDegree
    
    Tests dividing a polynomial by a higher degree polynomial.
    """
    # p1(x) = 2x^2 + x + 1
    poly1 = np.array([2.0, 1.0, 1.0])
    
    # p2(x) = 1
    poly2 = np.zeros(3)
    poly2[0] = 1.0
    
    quotient = np.zeros(1)
    remainder = np.zeros(2)
    
    pt.math.DividePolynomial(poly1, poly2, quotient, remainder)
    
    # Verify by reconstruction: poly1 = quotient * poly2 + remainder
    reconstructed_poly = pt.math.MultiplyPolynomials(quotient, poly2)
    reconstructed_poly[-remainder.shape[0]:] += remainder
    
    tolerance = 1e-12
    assert np.all(np.abs(poly1 - reconstructed_poly) < tolerance)


def test_dividepolynomial():
    """
    Python implementation of C++ test: Polynomial.DividePolynomial
    
    General test for dividing polynomials.
    """
    # p1(x) = 2x^2 + x + 1
    poly1 = np.array([2.0, 1.0, 1.0])
    
    # p2(x) = 1
    poly2 = np.zeros(3)
    poly2[0] = 1.0
    
    quotient = np.zeros(3)
    remainder = np.zeros(2)
    
    pt.math.DividePolynomial(poly1, poly2, quotient, remainder)
    
    # Verify by reconstruction: poly1 = quotient * poly2 + remainder
    reconstructed_poly = pt.math.MultiplyPolynomials(quotient, poly2)
    reconstructed_poly[-remainder.shape[0]:] += remainder
    
    tolerance = 1e-12
    assert np.all(np.abs(poly1 - reconstructed_poly) < tolerance)


def test_minimizeconstantpolynomial():
    """
    Python implementation of C++ test: Polynomial.MinimizeConstantPolynomial
    
    Tests minimizing a constant polynomial.
    """
    # p(x) = 1
    polynomial = np.array([1.0])
    
    min_x = 0.0
    max_x = 1.0
    
    optimal_x = np.zeros(1)
    optimal_value = np.zeros(1)
    
    pt.math.MinimizePolynomial(polynomial, min_x, max_x, optimal_x, optimal_value)
    
    assert optimal_value[0] == 1.0
    assert optimal_x[0] <= max_x
    assert optimal_x[0] >= min_x


def test_minimizelinearpolynomial():
    """
    Python implementation of C++ test: Polynomial.MinimizeLinearPolynomial
    
    Tests minimizing a linear polynomial.
    """
    # p(x) = x - 2
    polynomial = np.array([1.0, 2.0])
    
    min_x = 0.0
    max_x = 1.0
    
    optimal_x = np.zeros(1)
    optimal_value = np.zeros(1)
    
    pt.math.MinimizePolynomial(polynomial, min_x, max_x, optimal_x, optimal_value)
    
    assert optimal_x[0] == 0.0
    assert optimal_value[0] == 2.0


def test_minimizequadraticpolynomial():
    """
    Python implementation of C++ test: Polynomial.MinimizeQuadraticPolynomial
    
    Tests minimizing a quadratic polynomial with different bounds.
    """
    # p(x) = x^2 - 3x + 2
    # min_x = 3/2
    # min_value = -1/4
    polynomial = np.array([1.0, -3.0, 2.0])
    
    # Test case 1: Minimum is within bounds
    min_x = -2.0
    max_x = 2.0
    
    optimal_x = np.zeros(1)
    optimal_value = np.zeros(1)
    
    pt.math.MinimizePolynomial(polynomial, min_x, max_x, optimal_x, optimal_value)
    
    assert abs(optimal_x[0] - 3.0/2.0) < 1e-12
    assert abs(optimal_value[0] - (-1.0/4.0)) < 1e-12
    
    # Test case 2: Minimum is outside bounds (maximum x bound is smaller)
    min_x = -2.0
    max_x = 1.0
    
    pt.math.MinimizePolynomial(polynomial, min_x, max_x, optimal_x, optimal_value)
    
    assert abs(optimal_x[0] - 1.0) < 1e-12
    assert abs(optimal_value[0] - 0.0) < 1e-12
    
    # Test case 3: Minimum is outside bounds (minimum x bound is larger)
    min_x = 2.0
    max_x = 3.0
    
    pt.math.MinimizePolynomial(polynomial, min_x, max_x, optimal_x, optimal_value)
    
    assert abs(optimal_x[0] - 2.0) < 1e-12
    assert abs(optimal_value[0] - 0.0) < 1e-12
