package org.dovershockwave.subsystems.led;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("led");
  private final StringPublisher ledPanelPatternPub = table.getStringTopic("ledPanelPattern").publish();

  public void setLedPanelPattern(String pattern) {
    ledPanelPatternPub.set(pattern);
  }
}