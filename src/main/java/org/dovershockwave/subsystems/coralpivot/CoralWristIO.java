package org.dovershockwave.subsystems.coralpivot;

import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface CoralWristIO {
  @AutoLog class CoralWristIOInputs {
    public boolean wristConnected = false;
    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  default void updateInputs(CoralWristIOInputs inputs) {}

  default void setWristPosition(double rad, double ff) {}

  default void setWristPIDF(PIDFGains gains) {}

  default void setWristAbsPosOffset(double offset) {}
}