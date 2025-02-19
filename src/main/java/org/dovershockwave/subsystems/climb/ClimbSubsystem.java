package org.dovershockwave.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("Climb/PID/", ClimbConstants.GAINS);

  private final Alert disconnectedAlert = new Alert("Disconnected climb motor (" + ClimbConstants.SPARK_ID + ")", Alert.AlertType.kError);

  private ClimbState desiredState = ClimbState.STARTING;

  public ClimbSubsystem(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override public void periodic() {
    climbIO.updateInputs(climbInputs);
    Logger.processInputs("Climb", climbInputs);

    tunablePIDF.periodic(climbIO::setPivotPIDF, positionRad -> {
      climbIO.setPivotPosition(positionRad);
      setDesiredState(new ClimbState(positionRad));
    });

    disconnectedAlert.set(!climbInputs.connected);
  }

  public void setDesiredState(ClimbState desiredState) {
    climbIO.setPivotPosition(desiredState.positionRad());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "Climb/State")
  public ClimbState getState() {
    return new ClimbState(climbInputs.positionRad);
  }

  @AutoLogOutput(key = "Climb/DesiredState")
  public ClimbState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "Climb/AtDesiredState")
  public boolean atDesiredState() {
    return MathUtil.isNear(climbInputs.positionRad, desiredState.positionRad(), ClimbConstants.POSITION_TOLERANCE);
  }
}