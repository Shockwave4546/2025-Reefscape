package org.dovershockwave.subsystems.coralpivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.utils.tunable.TunableArmGains;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralPivotSubsystem extends SubsystemBase {
  private final CoralArmIO coralArmIO;
  private final CoralArmIOInputsAutoLogged coralArmInputs = new CoralArmIOInputsAutoLogged();

  private final CoralWristIO coralWristIO;
  private final CoralWristIOInputsAutoLogged coralWristInputs = new CoralWristIOInputsAutoLogged();

  private final TunableArmGains wristTunableGains = new TunableArmGains(
          "CoralPivot/Wrist/Gains/",
          CoralPivotConstants.WRIST_GAINS,
          CoralPivotConstants.WRIST_FEEDFORWARD_GAINS,
          CoralPivotConstants.WRIST_CONSTRAINTS,
          CoralPivotConstants.WRIST_MIN_POS,
          CoralPivotConstants.WRIST_MAX_POS
  );
  private final ArmFeedforward wristFeedforward = new ArmFeedforward(
          CoralPivotConstants.WRIST_FEEDFORWARD_GAINS.kS(),
          CoralPivotConstants.WRIST_FEEDFORWARD_GAINS.kG(),
          CoralPivotConstants.WRIST_FEEDFORWARD_GAINS.kV(),
          CoralPivotConstants.WRIST_FEEDFORWARD_GAINS.kA()
  );
  private TrapezoidProfile wristProfile = new TrapezoidProfile(CoralPivotConstants.WRIST_CONSTRAINTS);

  private final TunableArmGains armTunableGains = new TunableArmGains(
          "CoralPivot/Arm/Gains/",
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

  private final Alert wristDisconnectedAlert = new Alert("Disconnected coral wrist motor (" + CoralPivotConstants.WRIST_SPARK_ID + ")", Alert.AlertType.kError);
  private final Alert armLeftDisconnectedAlert = new Alert("Disconnected coral left arm motor (" + CoralPivotConstants.ARM_LEFT_SPARK_ID, Alert.AlertType.kError);
  private final Alert armRightDisconnectedAlert = new Alert("Disconnected coral right arm motor (" + CoralPivotConstants.ARM_RIGHT_SPARK_ID + ")", Alert.AlertType.kError);

  private CoralPivotState desiredState = CoralPivotState.SAFE_POSITION_AFTER_START_ONE;

  public CoralPivotSubsystem(CoralArmIO coralArmIO, CoralWristIO coralWristIO) {
    this.coralArmIO = coralArmIO;
    this.coralWristIO = coralWristIO;
  }

  @Override public void periodic() {
    coralArmIO.updateInputs(coralArmInputs);
    coralWristIO.updateInputs(coralWristInputs);
    Logger.processInputs("CoralPivot/Arm", coralArmInputs);
    Logger.processInputs("CoralPivot/Wrist", coralWristInputs);

    wristTunableGains.periodic(coralWristIO::setWristPIDF, wristFFConstants -> {
      wristFeedforward.setKs(wristFFConstants.kS());
      wristFeedforward.setKg(wristFFConstants.kG());
      wristFeedforward.setKv(wristFFConstants.kV());
      wristFeedforward.setKa(wristFFConstants.kA());
    }, constraints -> {
      this.wristProfile = new TrapezoidProfile(constraints);
    }, positionRad -> {
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

    setDesiredState(desiredState);
  }

  @SuppressWarnings("removal")
  public void setDesiredState(CoralPivotState desiredState) {
    this.desiredState = desiredState;

    final var wristCurrentState = new TrapezoidProfile.State(coralWristInputs.wristPositionRad, coralWristInputs.wristVelocityRadPerSec);
    final var wristGoalState = new TrapezoidProfile.State(desiredState.wristPositionRad(), 0.0);
    final var wristNextState = wristProfile.calculate(0.02, wristCurrentState, wristGoalState);
    final var wristAcceleration = (wristNextState.velocity - wristCurrentState.velocity) / 0.02;
//    coralWristIO.setWristPosition(desiredState.wristPositionRad(), wristFeedforward.calculate(wristNextState.position, wristNextState.velocity, wristAcceleration));

    final var armCurrentState = new TrapezoidProfile.State(coralArmInputs.armRightPositionRad, coralArmInputs.armRightVelocityRadPerSec);
    final var armGoalState = new TrapezoidProfile.State(desiredState.armPositionRad(), 0.0);
    final var armNextState = armProfile.calculate(0.02, armCurrentState, armGoalState);
    final var armAcceleration = (armNextState.velocity - armCurrentState.velocity) /  0.02;
//    coralArmIO.setArmPosition(desiredState.armPositionRad(), armFeedforward.calculate(armNextState.position, armNextState.velocity, armAcceleration));
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