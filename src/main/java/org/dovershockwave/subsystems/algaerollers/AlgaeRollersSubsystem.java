package org.dovershockwave.subsystems.algaerollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollersSubsystem extends SubsystemBase {
  private final AlgaeRollersIO algaeIO;
  private final AlgaeRollersIOInputsAutoLogged algaeInputs = new AlgaeRollersIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Disconnected algae rollers motor (" + AlgaeRollersConstants.SPARK_ID + ")", Alert.AlertType.kError);

  private AlgaeRollersState desiredState = AlgaeRollersState.STOPPED;

  public AlgaeRollersSubsystem(AlgaeRollersIO algaeIO) {
    this.algaeIO = algaeIO;
  }

  @Override public void periodic() {
    algaeIO.updateInputs(algaeInputs);
    Logger.processInputs("AlgaeRollers", algaeInputs);

    disconnectedAlert.set(!algaeInputs.connected);
  }

  public void setDesiredState(AlgaeRollersState desiredState) {
    algaeIO.setVoltage(desiredState.volts());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "AlgaeRollers/State")
  public AlgaeRollersState getState() {
    return new AlgaeRollersState(algaeInputs.velocityRadPerSec);
  }

  @AutoLogOutput(key = "AlgaeRollers/DesiredState")
  public AlgaeRollersState getDesiredState() {
    return this.desiredState;
  }
}