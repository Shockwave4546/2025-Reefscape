package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.elevator.lidar.LidarIO;
import org.dovershockwave.subsystems.elevator.lidar.LidarIOInputsAutoLogged;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.utils.TunableElevatorGains;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final LidarIO lidarIO;
  private final LidarIOInputsAutoLogged lidarInputs = new LidarIOInputsAutoLogged();

  private final TunableElevatorGains tunableGains = new TunableElevatorGains("Elevator/Gains/", ElevatorConstants.PID_GAINS, ElevatorConstants.FEEDFORWARD_GAINS, ElevatorConstants.CONSTRAINTS);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
          ElevatorConstants.FEEDFORWARD_GAINS.kS(),
          ElevatorConstants.FEEDFORWARD_GAINS.kG(),
          ElevatorConstants.FEEDFORWARD_GAINS.kV(),
          ElevatorConstants.FEEDFORWARD_GAINS.kA()
  );
  private TrapezoidProfile profile = new TrapezoidProfile(ElevatorConstants.CONSTRAINTS);

  private final Alert leftDisconnectedAlert = new Alert("Disconnected left motor on the elevator.", Alert.AlertType.kError);
  private final Alert rightDisconnectedAlert = new Alert("Disconnected right motor on the elevator.", Alert.AlertType.kError);

  private ElevatorState desiredState = ElevatorState.STARTING;

  public ElevatorSubsystem(ElevatorIO elevatorIO, LidarIO lidarIO) {
    this.elevatorIO = elevatorIO;
    this.lidarIO = lidarIO;
  }

  @Override public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    lidarIO.updateInputs(lidarInputs);
    Logger.processInputs("Elevator/Lidar", lidarInputs);

    tunableGains.periodic(elevatorIO::setPIDF, ffConstants -> {
      feedforward.setKs(ffConstants.kS());
      feedforward.setKg(ffConstants.kG());
      feedforward.setKv(ffConstants.kV());
      feedforward.setKa(ffConstants.kA());
    }, constraints -> {
      this.profile = new TrapezoidProfile(constraints);
    }, positionRad -> {
      setDesiredState(new ElevatorState(positionRad));
    });

    leftDisconnectedAlert.set(!elevatorInputs.leftConnected);
    rightDisconnectedAlert.set(!elevatorInputs.rightConnected);

    setDesiredState(desiredState);
  }

  public void setDesiredState(ElevatorState desiredState) {
    final var currentState = new TrapezoidProfile.State(elevatorInputs.leftPositionRad, elevatorInputs.leftVelocityRadPerSec);
    final var goalState = new TrapezoidProfile.State(desiredState.positionRad(), 0.0);
    final var nextState = profile.calculate(0.02, currentState, goalState);
    elevatorIO.setPosition(desiredState.positionRad(), feedforward.calculate(nextState.velocity));
    this.desiredState = desiredState;
  }

  public void setDesiredState(ReefScoringPosition.ReefLevel level) {
    switch (level) {
      case L1 -> setDesiredState(ElevatorState.L1);
      case L2 -> setDesiredState(ElevatorState.L2);
      case L3 -> setDesiredState(ElevatorState.L3);
      case L4 -> setDesiredState(ElevatorState.L4);
    }
  }

  @AutoLogOutput(key = "Elevator/State")
  public ElevatorState getState() {
    return new ElevatorState(elevatorInputs.leftPositionRad);
  }

  @AutoLogOutput(key = "Elevator/DesiredState")
  public ElevatorState getDesiredState() {
    return this.desiredState;
  }

  @AutoLogOutput(key = "Elevator/AtDesiredState")
  public boolean atDesiredState() {
    return MathUtil.isNear(elevatorInputs.leftPositionRad, desiredState.positionRad(), ElevatorConstants.POSITION_TOLERANCE);
  }
}