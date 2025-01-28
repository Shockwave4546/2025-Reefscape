package org.dovershockwave.subsystems.algaerollers;

import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRollerIO {
  @AutoLog class AlgaeRollerIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(AlgaeRollerIOInputs inputs) {}

  default void setVelocity(double velocityRadPerSec) {}

  default void setPIDF(PIDFGains gains) {}
}