package org.dovershockwave.subsystems.coralpivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class CoralPivotSubsystem extends SubsystemBase {
  private final CoralPivotIO coralPivotIO;
  private final CoralPivotIOInputsAutoLogged coralPivotInputs = new CoralPivotIOInputsAutoLogged();

  private final TunablePIDF wristTunablePIDF = new TunablePIDF("CoralPivot/Wrist/PID/", CoralPivotConstants.WRIST_GAINS);
  private final TunablePIDF biggerPivotTunablePIDF = new TunablePIDF("CoralPivot/BiggerPivot/PID/", CoralPivotConstants.BIGGER_PIVOT_GAINS);

  private final Alert wristDisconnectedAlert = new Alert("Disconnected motor on the coral pivot wrist motor.", Alert.AlertType.kError);
  private final Alert biggerPivotLeftDisconnectedAlert = new Alert("Disconnected motor on the coral bigger pivot left motor.", Alert.AlertType.kError);
  private final Alert biggerPivotRightDisconnectedAlert = new Alert("Disconnected motor on the coral bigger pivot right motor.", Alert.AlertType.kError);

  public CoralPivotSubsystem(CoralPivotIO coralPivotIO) {
    this.coralPivotIO = coralPivotIO;
  }

  @Override public void periodic() {
    coralPivotIO.updateInputs(coralPivotInputs);
    Logger.processInputs("CoralPivot", coralPivotInputs);

    wristTunablePIDF.periodic(coralPivotIO::setWristPIDF, coralPivotIO::setWristPosition);
    biggerPivotTunablePIDF.periodic(coralPivotIO::setBiggerPivotPIDF, coralPivotIO::setBiggerPivotPosition);

    wristDisconnectedAlert.set(!coralPivotInputs.wristConnected);
    biggerPivotLeftDisconnectedAlert.set(!coralPivotInputs.biggerPivotLeftConnected);
    biggerPivotRightDisconnectedAlert.set(!coralPivotInputs.biggerPivotRightConnected);
  }

  public void setState(CoralPivotState state) {
    coralPivotIO.setWristPosition(state.wristPositionRad);
    coralPivotIO.setBiggerPivotPosition(state.biggerPivotPositionRad);
  }
}