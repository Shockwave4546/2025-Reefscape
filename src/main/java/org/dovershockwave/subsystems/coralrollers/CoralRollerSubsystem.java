package org.dovershockwave.subsystems.coralrollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class CoralRollerSubsystem extends SubsystemBase {
  private final CoralRollerIO coralIO;
  private final CoralRollerIOInputsAutoLogged coralInputs = new CoralRollerIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("CoralRoller/PID/", CoralRollersConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected motor on the coral roller.", Alert.AlertType.kError);

  public CoralRollerSubsystem(CoralRollerIO coralIO) {
    this.coralIO = coralIO;
  }

  @Override public void periodic() {
    coralIO.updateInputs(coralInputs);
    Logger.processInputs("CoralRoller", coralInputs);

    tunablePIDF.periodic(coralIO::setPIDF, coralIO::setVelocity);

    disconnectedAlert.set(!coralInputs.connected);
  }

  public void setState(CoralRollerState state) {
    coralIO.setVelocity(state.velocityRadPerSec);
  }
}