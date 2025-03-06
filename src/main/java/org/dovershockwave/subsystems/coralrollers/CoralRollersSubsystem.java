package org.dovershockwave.subsystems.coralrollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralRollersSubsystem extends SubsystemBase {
  private final CoralRollersIO coralIO;
  private final CoralRollersIOInputsAutoLogged coralInputs = new CoralRollersIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Disconnected coral roller motor (" + CoralRollersConstants.SPARK_ID + ")", Alert.AlertType.kError);

  private CoralRollersState desiredState = CoralRollersState.STOPPED;

  public CoralRollersSubsystem(CoralRollersIO coralIO) {
    this.coralIO = coralIO;
  }

  @Override public void periodic() {
    coralIO.updateInputs(coralInputs);
    Logger.processInputs("CoralRollers", coralInputs);

    disconnectedAlert.set(!coralInputs.connected);
  }

  public double getCurrentAmps() {
    return coralInputs.currentAmps;
  }

  public void setDesiredState(CoralRollersState desiredState) {
    coralIO.setVoltage(desiredState.volts());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "CoralRollers/State")
  public CoralRollersState getState() {
    return new CoralRollersState(coralInputs.appliedVolts);
  }

  @AutoLogOutput(key = "CoralRollers/DesiredState")
  public CoralRollersState getDesiredState() {
    return this.desiredState;
  }
}