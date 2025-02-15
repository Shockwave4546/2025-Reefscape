package org.dovershockwave.subsystems.coralpivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.utils.TunableArmFeedforward;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralPivotSubsystem extends SubsystemBase {
  private final CoralPivotIO coralPivotIO;
  private final CoralPivotIOInputsAutoLogged coralPivotInputs = new CoralPivotIOInputsAutoLogged();

  private final TunablePIDF wristTunablePIDF = new TunablePIDF("CoralPivot/Wrist/PID/", CoralPivotConstants.WRIST_GAINS);
  private final TunablePIDF biggerPivotTunablePIDF = new TunablePIDF("CoralPivot/BiggerPivot/PID/", CoralPivotConstants.BIGGER_PIVOT_GAINS);
  private final TunableArmFeedforward tunableArmFeedforward = new TunableArmFeedforward("CoralPivot/BiggerPivot/ArmFeedforward/", CoralPivotConstants.ARM_FEEDFORWARD_CONSTANTS);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(
          CoralPivotConstants.ARM_FEEDFORWARD_CONSTANTS.kS(),
          CoralPivotConstants.ARM_FEEDFORWARD_CONSTANTS.kG(),
          CoralPivotConstants.ARM_FEEDFORWARD_CONSTANTS.kV(),
          CoralPivotConstants.ARM_FEEDFORWARD_CONSTANTS.kA()
  );

  private final Alert wristDisconnectedAlert = new Alert("Disconnected motor on the coral pivot wrist motor.", Alert.AlertType.kError);
  private final Alert biggerPivotLeftDisconnectedAlert = new Alert("Disconnected motor on the coral bigger pivot left motor.", Alert.AlertType.kError);
  private final Alert biggerPivotRightDisconnectedAlert = new Alert("Disconnected motor on the coral bigger pivot right motor.", Alert.AlertType.kError);

  private CoralPivotState desiredState = CoralPivotState.STARTING;

  public CoralPivotSubsystem(CoralPivotIO coralPivotIO) {
    this.coralPivotIO = coralPivotIO;
  }

  @Override public void periodic() {
    coralPivotIO.updateInputs(coralPivotInputs);
    Logger.processInputs("CoralPivot", coralPivotInputs);

    wristTunablePIDF.periodic(coralPivotIO::setWristPIDF, positionRad -> {
      coralPivotIO.setWristPosition(positionRad);
      setDesiredState(new CoralPivotState(positionRad, coralPivotInputs.biggerPivotLeftPositionRad));
    });

    biggerPivotTunablePIDF.periodic(coralPivotIO::setBiggerPivotPIDF, positionRad -> {
      setDesiredState(new CoralPivotState(coralPivotInputs.wristPositionRad, positionRad));
    });

    tunableArmFeedforward.periodic((armFeedforwardConstants) -> {
      armFeedforward.setKs(armFeedforwardConstants.kS());
      armFeedforward.setKg(armFeedforwardConstants.kG());
      armFeedforward.setKv(armFeedforwardConstants.kV());
      armFeedforward.setKa(armFeedforwardConstants.kA());
    }, positionRad -> {
      setDesiredState(new CoralPivotState(coralPivotInputs.wristPositionRad, positionRad));
    });

    wristDisconnectedAlert.set(!coralPivotInputs.wristConnected);
    biggerPivotLeftDisconnectedAlert.set(!coralPivotInputs.biggerPivotLeftConnected);
    biggerPivotRightDisconnectedAlert.set(!coralPivotInputs.biggerPivotRightConnected);
  }

  public void setDesiredState(CoralPivotState desiredState) {
    coralPivotIO.setWristPosition(desiredState.wristPositionRad());

    final var currentState = new TrapezoidProfile.State(coralPivotInputs.biggerPivotLeftPositionRad, coralPivotInputs.biggerPivotLeftVelocityRadPerSec);
    final var goalState = new TrapezoidProfile.State(desiredState.biggerPivotPositionRad(), 0.0);
    final var nextState = CoralPivotConstants.ARM_TRAPEZOID_PROFILE.calculate(0.02, currentState, goalState);
    coralPivotIO.setBiggerPivotPosition(desiredState.biggerPivotPositionRad(), armFeedforward.calculate(nextState.position, nextState.velocity));
    this.desiredState = desiredState;
  }

  public void setDesiredState(ReefScoringPosition.ReefLevel level) {
    switch (level) {
      case L1 -> setDesiredState(CoralPivotState.L1);
      case L2 -> setDesiredState(CoralPivotState.L2);
      case L3 -> setDesiredState(CoralPivotState.L3);
      case L4 -> setDesiredState(CoralPivotState.L4);
    }
  }

  @AutoLogOutput(key = "CoralPivot/State")
  public CoralPivotState getState() {
    return new CoralPivotState(coralPivotInputs.wristPositionRad, coralPivotInputs.biggerPivotLeftPositionRad);
  }

  @AutoLogOutput(key = "CoralPivot/DesiredState")
  public CoralPivotState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "CoralPivot/AtDesiredWristState")
  public boolean atDesiredWristState() {
    return MathUtil.isNear(coralPivotInputs.wristPositionRad, desiredState.wristPositionRad(), CoralPivotConstants.WRIST_POSITION_TOLERANCE);
  }

  @AutoLogOutput(key = "CoralPivot/AtDesiredBiggerPivotState")
  public boolean atDesiredBiggerPivotState() {
    return MathUtil.isNear(coralPivotInputs.biggerPivotLeftPositionRad, desiredState.biggerPivotPositionRad(), CoralPivotConstants.BIGGER_PIVOT_POSITION_TOLERANCE);
  }

  @AutoLogOutput(key = "CoralPivot/AtDesiredState")
  public boolean atDesiredState() {
    return atDesiredWristState() && atDesiredBiggerPivotState();
  }
}