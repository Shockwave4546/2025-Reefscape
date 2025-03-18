package org.dovershockwave.subsystems.algaerollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.algaerollers.commands.IdleAlgaeRollersCommand;
import org.dovershockwave.utils.tunable.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollersSubsystem extends SubsystemBase {
  private final AlgaeRollersIO algaeIO;
  private final AlgaeRollersIOInputsAutoLogged algaeInputs = new AlgaeRollersIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Disconnected algae rollers motor (" + AlgaeRollersConstants.SPARK_ID + ")", Alert.AlertType.kError);

  private final TunableNumber voltIn = new TunableNumber("AlgaeRollers/VoltIn", 0.0);
  private AlgaeRollersState desiredState = AlgaeRollersState.STOPPED;

  public AlgaeRollersSubsystem(AlgaeRollersIO algaeIO) {
    this.algaeIO = algaeIO;
  }

  @Override public void periodic() {
    algaeIO.updateInputs(algaeInputs);
    Logger.processInputs("AlgaeRollers", algaeInputs);

    disconnectedAlert.set(!algaeInputs.connected);
    setDesiredState(desiredState);
//    algaeIO.setVoltage(voltIn.get());
  }

  public double getCurrentAmps() {
    return algaeInputs.currentAmps;
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