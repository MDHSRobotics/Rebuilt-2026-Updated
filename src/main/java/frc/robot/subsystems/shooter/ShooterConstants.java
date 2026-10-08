package frc.robot.subsystems.shooter;

import org.wpilib.command2.SubsystemBase;

public class ShooterConstants extends SubsystemBase {
  public static final int SHOOTER_LEFT_MOTOR_ID = 20;
  public static final int SHOOTER_RIGHT_MOTOR_ID = 19;
  public static final int KICKER_MOTOR_ID = 21;

  /**
   * The current limit for the shooter and kicker in amps. This is currently set to the value
   * suggested by <a
   * href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV
   * for the NEO Vortex.</a>
   */
  public static final int CURRENT_LIMIT = 80;

  /* Shooter Motors PID Constants */
  public static final double K_P_SHOOTER = 0.004;
  public static final double K_I_SHOOTER = 0.00000;
  public static final double K_D_SHOOTER = 0.00001;

  /* Power of kicker when shooting */
  public static final double KICKER_POWER = 0.3;

  /** Testing Variables */
  public static final double TEST_POWER = 0.5;

  public static final double TEST_RPM = 500.0;

  public static final double TEST_RPM_2 = 1000.0;

  public static final double TEST_TIMEOUT = 1.5;

  /* Constants for calculating rpm */
  public static final double[] DISTANCES = {
    0, 40, 64, 92
  }; // Fill out with more values to make more accurate
  public static final double[] RPMS = {2400, 2500, 2650, 3000};

  public static final double SLOPE = 36. / 7.;
  // This is the rpm of the closest distance that the shooter can shoot the ball into the hub
  public static final double INTERCEPT = 2156;
}
