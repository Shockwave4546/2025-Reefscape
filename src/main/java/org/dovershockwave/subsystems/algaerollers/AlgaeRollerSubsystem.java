package org.dovershockwave.subsystems.algaerollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollerSubsystem extends SubsystemBase {
  private final AlgaeRollersIO algaeIO;
  private final AlgaeRollersIOInputsAutoLogged algaeInputs = new AlgaeRollersIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("AlgaeRoller/PID/", AlgaeRollersConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected motor on the algae roller.", Alert.AlertType.kError);

  private AlgaeRollersState desiredState = AlgaeRollersState.STOPPED;

  public AlgaeRollerSubsystem(AlgaeRollersIO algaeIO) {
    this.algaeIO = algaeIO;
  }

  @Override public void periodic() {
    algaeIO.updateInputs(algaeInputs);
    Logger.processInputs("AlgaeRollers", algaeInputs);

    tunablePIDF.periodic(algaeIO::setPIDF, velocityRadPerSec -> {
      algaeIO.setVelocity(velocityRadPerSec);
      setDesiredState(new AlgaeRollersState("PID Tuning", velocityRadPerSec));
    });

    disconnectedAlert.set(!algaeInputs.connected);
  }

  public void setDesiredState(AlgaeRollersState desiredState) {
    algaeIO.setVelocity(desiredState.velocityRadPerSec());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "AlgaeRollers/State")
  public AlgaeRollersState getState() {
    return new AlgaeRollersState("Current State", algaeInputs.velocityRadPerSec);
  }

  @AutoLogOutput(key = "AlgaeRollers/DesiredState")
  public AlgaeRollersState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "AlgaeRollers/AtDesiredState")
  public boolean atDesiredState() {
    return MathUtil.isNear(algaeInputs.velocityRadPerSec, desiredState.velocityRadPerSec(), AlgaeRollersConstants.VELOCITY_TOLERANCE);
  }
}