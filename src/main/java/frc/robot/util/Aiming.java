package frc.robot.util;

import static java.lang.Math.*;

import org.wpilib.math.controller.PIDController;

public class Aiming {
  private static PIDController yawPID = new PIDController(0.01, 0, 0);

  /**
   * @param lensHeight Distance from the floor to the Limelight's lens in inches
   * @param goalHeight The height of the goal in inches
   * @param mountAngle The mounting angle of the limelight in degrees
   * @param offsetAngle The additional angle to the target in degrees
   * @return The horizontal distance from the Limelight to the apriltag
   */
  public static double calculateTagDistance(
      double lensHeight, double goalHeight, double mountAngle, double offsetAngle) {
    double angleToGoalRadians = toRadians(mountAngle + offsetAngle);
    return (goalHeight - lensHeight) / tan(angleToGoalRadians);
  }

  /**
   * @param tx The current tx given by the limelight
   * @return The turning power for the steering motors
   */
  public static double getYawTxAdjustment(double tx) {
    double value = yawPID.calculate(tx, 0);
    return value;
  }

  /**
   * Calculate the Shooter RPM based on a slope and intercept, and ty
   *
   * @param slope The Slope value taken from ShooterConstants file
   * @param intercept The shooter's base RPM
   * @param distance The distance from the robot to the target
   * @return The target RPM calculate based on the distance of the robot to the hub
   */
  public static double calculateShooterRPM(
      double slope, double intercept, double distance) {
    double targetRPM = slope * distance + intercept;
    return targetRPM;
  }

  /**
   * Calculate the Shooter RPM based on polynomial interpolation
   *
   * @param polynomial The polnomial to be used
   * @param distance The limelight's distance value when looking at a tag
   * @return The target RPM calculate based on the distance of the robot to the hub
   */
  public static double calculateShooterRPM(
      PolynomialInterpolation polynomial, double distance) {
    double targetRPM = polynomial.evaluate(distance);
    return targetRPM;
  }
}
