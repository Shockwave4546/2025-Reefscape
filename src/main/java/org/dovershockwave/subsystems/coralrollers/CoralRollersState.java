package org.dovershockwave.subsystems.coralrollers;

public enum CoralRollersState {
  STOPPED(0.0),
  INTAKE(0.0),
  OUTTAKE(0.0);

  public final double velocityRadPerSec;

  CoralRollersState(double velocityRadPerSec) {
    this.velocityRadPerSec = velocityRadPerSec;
  }
}