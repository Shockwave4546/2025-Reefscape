package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.elevator.lidar.LidarIO;
import org.dovershockwave.subsystems.elevator.lidar.LidarIOInputsAutoLogged;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final LidarIO lidarIO;
  private final LidarIOInputsAutoLogged lidarInputs = new LidarIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("Elevator/PID/", ElevatorConstants.GAINS);

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
    Logger.processInputs("Lidar", lidarInputs);

    tunablePIDF.periodic(elevatorIO::setPIDF, positionRad -> {
      elevatorIO.setPosition(positionRad);
      setDesiredState(new ElevatorState("PID Tuning", positionRad));
    });

    leftDisconnectedAlert.set(!elevatorInputs.leftConnected);
    rightDisconnectedAlert.set(!elevatorInputs.rightConnected);
  }

  /**
   * When the elevator is at the bottom, but the LIDAR reads a distance greater/less than the starting distance,
   * the relative encoder must be reset to ensure the robot knows the elevator is at the bottom.
   */
  public void calibratePosition() {
    final var atStartingState = desiredState == ElevatorState.STARTING && atDesiredState();
    final var trulyAtBottom = MathUtil.isNear(lidarInputs.distanceMeters, ElevatorConstants.LIDAR_STARTING_DISTANCE, ElevatorConstants.LIDAR_STARTING_DISTANCE_TOLERANCE);

    if (atStartingState && !trulyAtBottom) {
      elevatorIO.resetPosition();
    }
  }

  public void setDesiredState(ElevatorState desiredState) {
    elevatorIO.setPosition(desiredState.positionRad());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "Elevator/State")
  public ElevatorState getState() {
    return new ElevatorState("Current State", elevatorInputs.leftPositionRad);
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