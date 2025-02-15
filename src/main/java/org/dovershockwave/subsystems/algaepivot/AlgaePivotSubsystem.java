package org.dovershockwave.subsystems.algaepivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaePivotSubsystem extends SubsystemBase {
  private final AlgaePivotIO algaePivotIO;
  private final AlgaePivotIOInputsAutoLogged algaePivotInputs = new AlgaePivotIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("AlgaePivot/PID/", AlgaePivotConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected motor on the algae pivot motor.", Alert.AlertType.kError);

  private AlgaePivotState desiredState = AlgaePivotState.STARTING;

  public AlgaePivotSubsystem(AlgaePivotIO algaePivotIO) {
    this.algaePivotIO = algaePivotIO;
  }

  @Override public void periodic() {
    algaePivotIO.updateInputs(algaePivotInputs);
    Logger.processInputs("AlgaePivot", algaePivotInputs);

    tunablePIDF.periodic(algaePivotIO::setPivotPIDF, positionRad -> {
      algaePivotIO.setPivotPosition(positionRad);
      setDesiredState(new AlgaePivotState(positionRad));
    });

    disconnectedAlert.set(!algaePivotInputs.connected);
  }

  public void setDesiredState(AlgaePivotState desiredState) {
    algaePivotIO.setPivotPosition(desiredState.positionRad());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "AlgaePivot/State")
  public AlgaePivotState getState() {
    return new AlgaePivotState(algaePivotInputs.positionRad);
  }

  @AutoLogOutput(key = "AlgaePivot/DesiredState")
  public AlgaePivotState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "AlgaePivot/AtDesiredState")
  public boolean atDesiredState() {
    return MathUtil.isNear(algaePivotInputs.positionRad, desiredState.positionRad(), AlgaePivotConstants.POSITION_TOLERANCE);
  }
}