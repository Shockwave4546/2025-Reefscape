package org.dovershockwave.subsystems.algaerollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollerSubsystem extends SubsystemBase {
  private final AlgaeRollerIO algaeIO;
  private final AlgaeRollerIOInputsAutoLogged algaeInputs = new AlgaeRollerIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("AlgaeRoller/PID/", AlgaeRollersConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected motor on the algae roller.", Alert.AlertType.kError);

  public AlgaeRollerSubsystem(AlgaeRollerIO algaeIO) {
    this.algaeIO = algaeIO;
  }

  @Override public void periodic() {
    algaeIO.updateInputs(algaeInputs);
    Logger.processInputs("AlgaeRoller", algaeInputs);

    tunablePIDF.periodic(algaeIO::setPIDF, algaeIO::setVelocity);

    disconnectedAlert.set(!algaeInputs.connected);
  }

  public void setState(AlgaeRollerState state) {
    algaeIO.setVelocity(state.velocityRadPerSec);
  }
}