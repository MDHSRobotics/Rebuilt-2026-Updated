package frc.robot.util;

import org.wpilib.command2.Command;

public interface Testable {
  Command test();

  void resetTestIndicators();
}
