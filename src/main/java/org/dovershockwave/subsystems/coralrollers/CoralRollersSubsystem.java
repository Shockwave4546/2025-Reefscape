package org.dovershockwave.subsystems.coralrollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class CoralRollersSubsystem extends SubsystemBase {
  private final CoralRollersIO coralIO;
  private final CoralRollersIOInputsAutoLogged coralInputs = new CoralRollersIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("CoralRoller/PID/", CoralRollersConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected motor on the coral roller.", Alert.AlertType.kError);

  public CoralRollersSubsystem(CoralRollersIO coralIO) {
    this.coralIO = coralIO;
  }

  @Override public void periodic() {
    coralIO.updateInputs(coralInputs);
    Logger.processInputs("CoralRollers", coralInputs);

    tunablePIDF.periodic(coralIO::setPIDF, coralIO::setVelocity);

    disconnectedAlert.set(!coralInputs.connected);
  }

  public void setState(CoralRollersState state) {
    coralIO.setVelocity(state.velocityRadPerSec);
  }
}