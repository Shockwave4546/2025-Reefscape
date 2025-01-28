package org.dovershockwave.subsystems.elevator.lidar;

import org.littletonrobotics.junction.AutoLog;

public interface LidarIO {
  @AutoLog class LidarIOInputs {
    public boolean hasValidMeasurement = false;
    public double distanceMeters = 0.0;
  }

  default void updateInputs(LidarIOInputs inputs) {}
}