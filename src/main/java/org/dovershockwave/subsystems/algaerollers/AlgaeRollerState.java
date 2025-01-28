package org.dovershockwave.subsystems.algaerollers;

public enum AlgaeRollerState {
  STOPPED(0.0),
  INTAKE(0.0),
  OUTTAKE(0.0);

  public final double velocityRadPerSec;

  AlgaeRollerState(double velocityRadPerSec) {
    this.velocityRadPerSec = velocityRadPerSec;
  }
}
