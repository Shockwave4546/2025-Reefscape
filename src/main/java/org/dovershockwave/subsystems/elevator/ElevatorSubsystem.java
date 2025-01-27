package org.dovershockwave.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final TunablePIDF tunablePIDF = new TunablePIDF("Elevator/PID/", ElevatorConstants.GAINS);

  private final Alert leftDisconnectedAlert = new Alert("Disconnected left motor on the elevator.", Alert.AlertType.kError);
  private final Alert rightDisconnectedAlert = new Alert("Disconnected right motor on the elevator.", Alert.AlertType.kError);

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
