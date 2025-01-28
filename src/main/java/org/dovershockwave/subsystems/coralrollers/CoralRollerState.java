package org.dovershockwave.subsystems.coralrollers;

public enum CoralRollerState {
  STOPPED(0.0),
  INTAKE(0.0),
  OUTTAKE(0.0);

  public final double velocityRadPerSec;

  CoralRollerState(double velocityRadPerSec) {
    this.velocityRadPerSec = velocityRadPerSec;
  }
}