package frc.robot.util.logging;

import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.telemetry.Telemetry;
import frc.robot.Constants.BusConstants;

public class EnergyMonitor {

  private final PowerDistribution m_pdh;

  // SmartDashboard keys
  private static final String KEY_TOTAL_CURRENT = "Energy/TotalCurrent";
  private static final String KEY_VOLTAGE = "Energy/Voltage";
  private static final String KEY_TOTAL_POWER = "Energy/TotalPower";
  private static final String KEY_TEMPERATURE = "Energy/Temperature";
  private static final String KEY_TOTAL_ENERGY = "Energy/TotalEnergy_Joules";

  public EnergyMonitor() {
    m_pdh = new PowerDistribution(BusConstants.ENERGY_MONITOR_BUS, 1, PowerDistribution.ModuleType.REV);
    // Clear accumulated energy on boot
    m_pdh.clearStickyFaults();
  }

  /** Call this from robotPeriodic(). */
  public void update() {
    Telemetry.log(KEY_TOTAL_CURRENT, m_pdh.getTotalCurrent());
    Telemetry.log(KEY_VOLTAGE, m_pdh.getVoltage());
    Telemetry.log(KEY_TOTAL_POWER, m_pdh.getTotalPower());
    Telemetry.log(KEY_TEMPERATURE, m_pdh.getTemperature());
    Telemetry.log(KEY_TOTAL_ENERGY, m_pdh.getTotalEnergy());
  }

  /** Optionally expose per-channel current for motor debugging. */
  public void updateChannels(int... channels) {
    for (int ch : channels) {
      Telemetry.log("Energy/Channel_" + ch, m_pdh.getCurrent(ch));
    }
  }
}
