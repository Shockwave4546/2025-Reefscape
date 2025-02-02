package org.dovershockwave.subsystems.coralrollers;

import org.littletonrobotics.junction.AutoLog;

public interface CoralRollersIO {
  @AutoLog class CoralRollersIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(CoralRollersIOInputs inputs) {}

  default void setVoltage(double volts) {}
}