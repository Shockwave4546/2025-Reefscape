package org.dovershockwave.subsystems.coralpivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.utils.TunableArmGains;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralPivotSubsystem extends SubsystemBase {
  private final CoralArmIO coralArmIO;
  private final CoralArmIOInputsAutoLogged coralArmInputs = new CoralArmIOInputsAutoLogged();

  private final CoralWristIO coralWristIO;
  private final CoralWristIOInputsAutoLogged coralWristInputs = new CoralWristIOInputsAutoLogged();

  private final TunablePIDF wristTunablePIDF = new TunablePIDF("CoralPivot/Wrist/PID/", CoralPivotConstants.WRIST_GAINS);
  private final TunableArmGains armTunableGains = new TunableArmGains(
          "CoralPivot/ArmGains/",
          CoralPivotConstants.ARM_GAINS,
          CoralPivotConstants.ARM_FEEDFORWARD_GAINS,
          CoralPivotConstants.ARM_CONSTRAINTS,
          CoralPivotConstants.ARM_MIN_POS,
          CoralPivotConstants.ARM_MAX_POS
  );
  private final ArmFeedforward armFeedforward = new ArmFeedforward(
          CoralPivotConstants.ARM_FEEDFORWARD_GAINS.kS(),
          CoralPivotConstants.ARM_FEEDFORWARD_GAINS.kG(),
          CoralPivotConstants.ARM_FEEDFORWARD_GAINS.kV(),
          CoralPivotConstants.ARM_FEEDFORWARD_GAINS.kA()
  );
  private TrapezoidProfile armProfile = new TrapezoidProfile(CoralPivotConstants.ARM_CONSTRAINTS);

  private final Alert wristDisconnectedAlert = new Alert("Disconnected motor on the coral pivot wrist motor.", Alert.AlertType.kError);
  private final Alert armLeftDisconnectedAlert = new Alert("Disconnected motor on the coral pivot left arm motor.", Alert.AlertType.kError);
  private final Alert armRightDisconnectedAlert = new Alert("Disconnected motor on the coral pivot right arm motor.", Alert.AlertType.kError);

  private CoralPivotState desiredState = CoralPivotState.STARTING;

  public CoralPivotSubsystem(CoralArmIO coralArmIO, CoralWristIO coralWristIO) {
    this.coralArmIO = coralArmIO;
    this.coralWristIO = coralWristIO;
  }

  @Override public void periodic() {
    coralArmIO.updateInputs(coralArmInputs);
    coralWristIO.updateInputs(coralWristInputs);
    Logger.processInputs("CoralPivot/Arm", coralArmInputs);
    Logger.processInputs("CoralPivot/Wrist", coralWristInputs);

    wristTunablePIDF.periodic(coralWristIO::setWristPIDF, positionRad -> {
      setDesiredState(new CoralPivotState(positionRad, desiredState.armPositionRad()));
    });

    armTunableGains.periodic(coralArmIO::setArmPIDF, armFFConstants -> {
      armFeedforward.setKs(armFFConstants.kS());
      armFeedforward.setKg(armFFConstants.kG());
      armFeedforward.setKv(armFFConstants.kV());
      armFeedforward.setKa(armFFConstants.kA());
    }, constraints -> {
      this.armProfile = new TrapezoidProfile(constraints);
    }, positionRad -> {
      setDesiredState(new CoralPivotState(desiredState.wristPositionRad(), positionRad));
    });

    wristDisconnectedAlert.set(!coralWristInputs.wristConnected);
    armLeftDisconnectedAlert.set(!coralArmInputs.armLeftConnected);
    armRightDisconnectedAlert.set(!coralArmInputs.armRightConnected);

    // Update FF values but this feels wrong
    setDesiredState(desiredState);
  }

  public void setDesiredState(CoralPivotState desiredState) {
    this.desiredState = desiredState;

    coralWristIO.setWristPosition(desiredState.wristPositionRad());

    final var currentState = new TrapezoidProfile.State(coralArmInputs.armRightPositionRad, coralArmInputs.armRightVelocityRadPerSec);
    final var goalState = new TrapezoidProfile.State(desiredState.armPositionRad(), 0.0);
    final var nextState = armProfile.calculate(0.02, currentState, goalState);
    coralArmIO.setArmPosition(desiredState.armPositionRad(), armFeedforward.calculate(nextState.position, nextState.velocity));
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
    return new CoralPivotState(coralWristInputs.wristPositionRad, coralArmInputs.armRightPositionRad);
  }

  @AutoLogOutput(key = "CoralPivot/DesiredState")
  public CoralPivotState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "CoralPivot/AtDesiredWristState")
  public boolean atDesiredWristState() {
    return MathUtil.isNear(coralWristInputs.wristPositionRad, desiredState.wristPositionRad(), CoralPivotConstants.WRIST_POSITION_TOLERANCE);
  }

  @AutoLogOutput(key = "CoralPivot/AtDesiredArmState")
  public boolean atDesiredArmState() {
    return MathUtil.isNear(coralArmInputs.armRightPositionRad, desiredState.armPositionRad(), CoralPivotConstants.ARM_POSITION_TOLERANCE);
  }

  @AutoLogOutput(key = "CoralPivot/AtDesiredState")
  public boolean atDesiredState() {
    return atDesiredWristState() && atDesiredArmState();
  }
}