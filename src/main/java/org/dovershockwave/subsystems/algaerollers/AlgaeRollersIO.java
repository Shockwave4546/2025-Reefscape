package org.dovershockwave.subsystems.algaerollers;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollersIO {
  @AutoLog class AlgaeRollersIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(AlgaeRollersIOInputs inputs) {}

  default void setVoltage(double volts) {}
}