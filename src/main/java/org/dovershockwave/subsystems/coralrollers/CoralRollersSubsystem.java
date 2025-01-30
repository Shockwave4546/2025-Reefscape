package org.dovershockwave.subsystems.coralrollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralRollersSubsystem extends SubsystemBase {
  private final CoralRollersIO coralIO;
  private final CoralRollersIOInputsAutoLogged coralInputs = new CoralRollersIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("CoralRoller/PID/", CoralRollersConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected motor on the coral roller.", Alert.AlertType.kError);

  private CoralRollersState desiredState = CoralRollersState.STOPPED;

  public CoralRollersSubsystem(CoralRollersIO coralIO) {
    this.coralIO = coralIO;
  }

  @Override public void periodic() {
    coralIO.updateInputs(coralInputs);
    Logger.processInputs("CoralRollers", coralInputs);

    tunablePIDF.periodic(coralIO::setPIDF, velocityRadPerSec -> {
      coralIO.setVelocity(velocityRadPerSec);
      setDesiredState(new CoralRollersState("PID Tuning", velocityRadPerSec));
    });

    disconnectedAlert.set(!coralInputs.connected);
  }

  public void setDesiredState(CoralRollersState desiredState) {
    coralIO.setVelocity(desiredState.velocityRadPerSec());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "CoralRollers/State")
  public CoralRollersState getState() {
    return new CoralRollersState("Current State", coralInputs.velocityRadPerSec);
  }

  @AutoLogOutput(key = "CoralRollers/DesiredState")
  public CoralRollersState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "CoralRollers/AtDesiredState")
  public boolean atDesiredState() {
    return MathUtil.isNear(coralInputs.velocityRadPerSec, desiredState.velocityRadPerSec(), CoralRollersConstants.VELOCITY_TOLERANCE);
  }
}