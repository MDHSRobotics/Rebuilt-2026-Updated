package frc.robot.util;

import java.util.Arrays;

/**
 * Polynomial interpolation using Lagrange's method.
 *
 * <p>Given a set of (x, y) points, this class finds the unique polynomial of degree n-1 that passes
 * through all n points, and evaluates it at any x.
 */
public class PolynomialInterpolation {

  private double[] xs;
  private double[] ys;

  /**
   * Constructs the interpolator with an initial set of points.
   *
   * @param xs the x-coordinates (must be distinct)
   * @param ys the y-coordinates (must match xs in length)
   */
  public PolynomialInterpolation(double[] xs, double[] ys) {
    setPoints(xs, ys);
  }

  /**
   * Replaces the current point set with a new one and resets the interpolant.
   *
   * @param xs the new x-coordinates (must be distinct)
   * @param ys the new y-coordinates (must match xs in length)
   */
  public void setPoints(double[] xs, double[] ys) {
    if (xs == null || ys == null)
      throw new IllegalArgumentException("Point arrays must not be null.");
    if (xs.length != ys.length)
      throw new IllegalArgumentException(
          "xs and ys must have the same length (got " + xs.length + " vs " + ys.length + ").");
    if (xs.length < 2)
      throw new IllegalArgumentException("At least 2 points are required for interpolation.");

    validateDistinct(xs);

    this.xs = Arrays.copyOf(xs, xs.length);
    this.ys = Arrays.copyOf(ys, ys.length);
  }

  /**
   * Adds a single new point and updates the interpolant.
   *
   * @param x the new x-coordinate (must not duplicate an existing one)
   * @param y the new y-coordinate
   */
  public void addPoint(double x, double y) {
    for (double existing : xs)
      if (Double.compare(existing, x) == 0)
        throw new IllegalArgumentException("Duplicate x-value: " + x);

    xs = Arrays.copyOf(xs, xs.length + 1);
    ys = Arrays.copyOf(ys, ys.length + 1);
    xs[xs.length - 1] = x;
    ys[ys.length - 1] = y;
  }

  /**
   * Updates the y-value of an existing point identified by its x-coordinate.
   *
   * @param x the x-coordinate of the point to update
   * @param newY the replacement y-value
   */
  public void updatePoint(double x, double newY) {
    for (int i = 0; i < xs.length; i++) {
      if (Double.compare(xs[i], x) == 0) {
        ys[i] = newY;
        return;
      }
    }
    throw new IllegalArgumentException("No point found with x = " + x);
  }

  /**
   * Evaluates the interpolating polynomial at the given x using Lagrange's formula:
   *
   * <p>P(x) = Σ_i y_i · Π_{j≠i} (x - x_j) / (x_i - x_j)
   *
   * @param x the value at which to evaluate the polynomial
   * @return the interpolated y value
   */
  public double evaluate(double x) {
    int n = xs.length;
    double result = 0.0;

    for (int i = 0; i < n; i++) {
      double term = ys[i];
      for (int j = 0; j < n; j++) {
        if (j != i) {
          term *= (x - xs[j]) / (xs[i] - xs[j]);
        }
      }
      result += term;
    }

    return result;
  }

  /** Returns the degree of the interpolating polynomial (n - 1). */
  public int degree() {
    return xs.length - 1;
  }

  /** Returns a read-only view of the current x-coordinates. */
  public double[] getXs() {
    return Arrays.copyOf(xs, xs.length);
  }

  /** Returns a read-only view of the current y-coordinates. */
  public double[] getYs() {
    return Arrays.copyOf(ys, ys.length);
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  private void validateDistinct(double[] values) {
    for (int i = 0; i < values.length; i++)
      for (int j = i + 1; j < values.length; j++)
        if (Double.compare(values[i], values[j]) == 0)
          throw new IllegalArgumentException(
              "x-coordinates must be distinct; duplicate found: " + values[i]);
  }
}
