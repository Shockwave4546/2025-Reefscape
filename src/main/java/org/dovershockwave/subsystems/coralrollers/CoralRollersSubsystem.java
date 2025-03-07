package org.dovershockwave.subsystems.coralrollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.subsystems.coralrollers.lidar.LidarIO;
import org.dovershockwave.subsystems.coralrollers.lidar.LidarIOInputsAutoLogged;
import org.dovershockwave.utils.tunable.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralRollersSubsystem extends SubsystemBase {
  private final CoralRollersIO coralIO;
  private final CoralRollersIOInputsAutoLogged coralInputs = new CoralRollersIOInputsAutoLogged();

  private final LidarIO lidarIO;
  private final LidarIOInputsAutoLogged lidarInputs = new LidarIOInputsAutoLogged();

  private final Alert coralRollersdisconnectedAlert = new Alert("Disconnected coral roller motor (" + CoralRollersConstants.SPARK_ID + ")", Alert.AlertType.kError);
  private final Alert lidarDisconnectedAlert = new Alert("Disconnected Lidar LaserCAN", Alert.AlertType.kError);

  /**
   * If the Lidar doesn't report a valid measurement for more than two seconds, we can assume it's disconnected.
   */
  private final Debouncer lidarDisconnectedDebouncer = new Debouncer(2);

  private final TunableNumber voltIn = new TunableNumber("CoralRollers/VoltIn", 0.0);

  private CoralRollersState desiredState = CoralRollersState.STOPPED;

  public CoralRollersSubsystem(CoralRollersIO coralIO, LidarIO lidarIO) {
    this.coralIO = coralIO;
    this.lidarIO = lidarIO;
  }

  @Override public void periodic() {
    coralIO.updateInputs(coralInputs);
    lidarIO.updateInputs(lidarInputs);
    Logger.processInputs("CoralRollers", coralInputs);
    Logger.processInputs("CoralRollers/Lidar", lidarInputs);

    coralRollersdisconnectedAlert.set(!coralInputs.connected);
    lidarDisconnectedAlert.set(lidarDisconnectedDebouncer.calculate(!lidarInputs.hasValidMeasurement));

//    coralIO.setVoltage(voltIn.get());
  }

  public double getCurrentAmps() {
    return coralInputs.currentAmps;
  }

  public double getLidarDistanceMeters() {
    return lidarInputs.distanceMeters;
  }

  public void setDesiredState(ReefScoringPosition.ReefLevel level) {
    switch (level) {
      case L1 -> coralIO.setVoltage(CoralRollersState.L1_OUTTAKE.volts());
      case L2 -> coralIO.setVoltage(CoralRollersState.L2_OUTTAKE.volts());
      case L3 -> coralIO.setVoltage(CoralRollersState.L3_OUTTAKE.volts());
      case L4 -> coralIO.setVoltage(CoralRollersState.L4_OUTTAKE.volts());
    }
  }

  public void setDesiredState(CoralRollersState desiredState) {
    coralIO.setVoltage(desiredState.volts());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "CoralRollers/State")
  public CoralRollersState getState() {
    return new CoralRollersState(coralInputs.appliedVolts);
  }

  @AutoLogOutput(key = "CoralRollers/DesiredState")
  public CoralRollersState getDesiredState() {
    return this.desiredState;
  }
}