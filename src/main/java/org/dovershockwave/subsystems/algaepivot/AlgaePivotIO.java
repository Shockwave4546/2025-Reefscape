package org.dovershockwave.subsystems.algaepivot;

import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
  @AutoLog class AlgaePivotIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(AlgaePivotIOInputs inputs) {}

  default void setPivotPosition(double rad) {}

  default void setPivotPIDF(PIDFGains gains) {}
}