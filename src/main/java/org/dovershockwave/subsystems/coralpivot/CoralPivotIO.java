package org.dovershockwave.subsystems.coralpivot;

import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface CoralPivotIO {
  @AutoLog class CoralPivotIOInputs {
    public boolean wristConnected = false;
    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;

    public boolean biggerPivotLeftConnected = false;
    public double biggerPivotLeftPositionRad = 0.0;
    public double biggerPivotLeftVelocityRadPerSec = 0.0;
    public double biggerPivotLeftAppliedVolts = 0.0;
    public double biggerPivotLeftCurrentAmps = 0.0;

    public boolean biggerPivotRightConnected = false;
    public double biggerPivotRightAppliedVolts = 0.0;
    public double biggerPivotRightCurrentAmps = 0.0;
  }

  default void updateInputs(CoralPivotIOInputs inputs) {}

  default void setWristPosition(double rad) {}

  default void setBiggerPivotPosition(double rad, double ff) {}

  default void setWristPIDF(PIDFGains gains) {}

  default void setBiggerPivotPIDF(PIDFGains gains) {}
}