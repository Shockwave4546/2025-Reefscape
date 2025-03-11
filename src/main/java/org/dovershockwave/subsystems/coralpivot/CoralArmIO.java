package org.dovershockwave.subsystems.coralpivot;

import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface CoralArmIO {
  @AutoLog class CoralArmIOInputs {
    /**
     * Follower motor
     */
    public boolean armLeftConnected = false;
    public double armLeftAppliedVolts = 0.0;
    public double armLeftCurrentAmps = 0.0;

    /**
     * Master motor
     */
    public boolean armRightConnected = false;
    public double armRightPositionRad = 0.0;
    public double armRightVelocityRadPerSec = 0.0;
    public double armRightAppliedVolts = 0.0;
    public double armRightCurrentAmps = 0.0;
  }

  default void updateInputs(CoralArmIOInputs inputs) {}

  default void setArmPosition(double rad, double ff) {}

  default void setArmPIDF(PIDFGains gains) {}
}