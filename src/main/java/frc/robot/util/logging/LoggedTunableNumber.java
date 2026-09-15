// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.logging;

import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
@SuppressWarnings("unused")
public class LoggedTunableNumber {
  private static final String tableKey = "/Tuning";

  private final String m_key;
  private double m_defaultValue;
  private double m_lastValue;

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    m_key = "Tuning/" + dashboardKey;
    this.m_defaultValue = defaultValue;
    m_lastValue = defaultValue;
    if (Constants.TUNING_MODE) {
      SmartDashboard.setDefaultNumber(m_key, defaultValue);
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (Constants.TUNING_MODE) {
      return SmartDashboard.getNumber(m_key, m_defaultValue);
    }
    return m_defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != m_lastValue) {
      m_lastValue = currentValue;
      return true;
    }

    return false;
  }
}
