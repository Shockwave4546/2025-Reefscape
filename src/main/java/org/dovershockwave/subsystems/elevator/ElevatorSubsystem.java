package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.elevator.lidar.LidarIO;
import org.dovershockwave.subsystems.elevator.lidar.LidarIOInputsAutoLogged;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final LidarIO lidarIO;
  private final LidarIOInputsAutoLogged lidarInputs = new LidarIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("Elevator/PID/", ElevatorConstants.GAINS);

  private final Alert leftDisconnectedAlert = new Alert("Disconnected left motor on the elevator.", Alert.AlertType.kError);
  private final Alert rightDisconnectedAlert = new Alert("Disconnected right motor on the elevator.", Alert.AlertType.kError);

  public ElevatorSubsystem(ElevatorIO elevatorIO, LidarIO lidarIO) {
    this.elevatorIO = elevatorIO;
    this.lidarIO = lidarIO;
  }

  @Override public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    lidarIO.updateInputs(lidarInputs);
    Logger.processInputs("Lidar", lidarInputs);

    tunablePIDF.periodic(elevatorIO::setPIDF, elevatorIO::setPosition);

    leftDisconnectedAlert.set(!elevatorInputs.leftConnected);
    rightDisconnectedAlert.set(!elevatorInputs.rightConnected);
  }

  public void setPosition(ElevatorPosition position) {
    elevatorIO.setPosition(position.positionRad);
  }
}