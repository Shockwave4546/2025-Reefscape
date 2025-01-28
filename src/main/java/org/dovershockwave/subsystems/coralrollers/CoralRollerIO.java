package org.dovershockwave.subsystems.coralrollers;

import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface CoralRollerIO {
  @AutoLog class CoralRollerIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(CoralRollerIOInputs inputs) {}

  default void setVelocity(double velocityRadPerSec) {}

  default void setPIDF(PIDFGains gains) {}
}