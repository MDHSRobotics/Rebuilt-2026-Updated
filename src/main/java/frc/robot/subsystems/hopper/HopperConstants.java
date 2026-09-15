package frc.robot.subsystems.hopper;

import org.wpilib.command2.SubsystemBase;

public class HopperConstants extends SubsystemBase {

  private HopperConstants() {}

  public static final int HOPPER_MOTOR_ID = 22;

  /**
   * The current limit for the hopper in amps. This is currently set to the value suggested by <a
   * href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV
   * for the NEO Vortex.</a>
   */
  public static final int CURRENT_LIMIT = 80;

  /** The proportional gain for the Intake */
  public static final double K_P = 0;

  /** Testing Variables */
  public static final double TEST_POWER = 0.5;

  public static final double TEST_RPM = 500.0;

  public static final double TEST_TIMEOUT = 1.5;

  public static final double SHOOT_POWER = .5;

  public enum HopperPowers {
    SHOOT(0.5),
    INTAKE(0.4),
    INTAKE_REVERSE(-0.4);

    public final double power;

    HopperPowers(double power) {
      this.power = power;
    }
  }
}
