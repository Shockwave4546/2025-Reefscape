package org.dovershockwave.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableProfiledPIDFController {
  private final ProfiledPIDController pid;
  private final TunableProfiledPIDF tunableGains;
  private final TunableNumber posTolerance;

  public TunableProfiledPIDFController(String prefix, PIDFGains pidGains, double posTolerance, TrapezoidProfile.Constraints constraints) {
    pid = new ProfiledPIDController(pidGains.p(), pidGains.i(), pidGains.d(), constraints);
    tunableGains = new TunableProfiledPIDF(prefix, pidGains, constraints);
    this.posTolerance = new TunableNumber(prefix + "(9) PosTolerance", posTolerance);
  }

  public void periodic() {
    tunableGains.periodic(gains -> {
      pid.setP(gains.p());
      pid.setI(gains.i());
      pid.setD(gains.d());
    }, pid::setConstraints, pid::setGoal);

    TunableNumber.ifChanged(hashCode() + 3, values -> pid.setTolerance(values[0]), posTolerance);
  }

  public void enableContinuousInput(double min, double max) {
    pid.enableContinuousInput(min, max);
  }

  public double calculate(double currentPos, double goalPos) {
    return pid.calculate(currentPos, goalPos);
  }

  public double getError() {
    return pid.getPositionError();
  }

  public void reset(double currentPos) {
    pid.reset(currentPos);
  }

  public void setGoal(double goalPos) {
    pid.setGoal(goalPos);
  }

  public boolean atGoal() {
    return pid.atGoal();
  }
}