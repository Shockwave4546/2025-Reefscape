package org.dovershockwave.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

import java.util.EnumMap;

public class Module {
  private static final EnumMap<ModuleType, Rotation2d> TURN_CHARACTERIZATION_ROT = new EnumMap<>(ModuleType.class);

  static {
    TURN_CHARACTERIZATION_ROT.put(ModuleType.FRONT_LEFT, Rotation2d.fromDegrees(-45));
    TURN_CHARACTERIZATION_ROT.put(ModuleType.FRONT_RIGHT, Rotation2d.fromDegrees(45));
    TURN_CHARACTERIZATION_ROT.put(ModuleType.BACK_LEFT, Rotation2d.fromDegrees(-135));
    TURN_CHARACTERIZATION_ROT.put(ModuleType.BACK_RIGHT, Rotation2d.fromDegrees(135));
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleType type;

  private final TunablePIDF drivePIDF;
  private final TunablePIDF turnPIDF;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, ModuleType type) {
    this.io = io;
    this.type = type;
    drivePIDF = new TunablePIDF("Drive/" + type.name + "Module/DrivePID/", SwerveConstants.DRIVE_PIDF);
    turnPIDF = new TunablePIDF("Drive/" + type.name + "Module/TurnPID/", SwerveConstants.TURN_PIDF);

    driveDisconnectedAlert = new Alert("Disconnected drive motor on " + type.name + " module.", Alert.AlertType.kError);
    turnDisconnectedAlert = new Alert("Disconnected turn motor on " + type.name + " module.", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + type.name + "Module", inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * SwerveConstants.WHEEL_RADIUS_METERS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    drivePIDF.periodic(io::setDrivePIDF, io::setDriveVelocity);
    turnPIDF.periodic(io::setTurnPIDF, value -> io.setTurnPosition(new Rotation2d(value)));

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / SwerveConstants.WHEEL_RADIUS_METERS);
    io.setTurnPosition(state.angle);
  }

  public void setDrivePIDF(PIDFGains gains) {
    io.setDrivePIDF(gains);
    drivePIDF.setGains(gains);
  }

  public PIDFGains getDrivePIDF() {
    return drivePIDF.getGains();
  }

  public void setTurnPIDF(PIDFGains gains) {
    io.setTurnPIDF(gains);
    turnPIDF.setGains(gains);
  }

  public PIDFGains getTurnPIDF() {
    return turnPIDF.getGains();
  }

  public void runDriveCharacterization(double volts) {
    io.setTurnPosition(new Rotation2d());
    io.setDriveVolts(volts);
  }

  public void runTurnCharacterization(double volts) {
    final boolean invertDriveDirection = type == ModuleType.FRONT_RIGHT || type == ModuleType.BACK_LEFT;
    io.setTurnPosition(TURN_CHARACTERIZATION_ROT.get(type));
    io.setDriveVolts(invertDriveDirection ? -1 * volts : volts);
  }

  public void setTurnPosition(Rotation2d rotation) {
    io.setTurnPosition(rotation);
  }

  public void stop() {
    io.setDriveVolts(0.0);
    io.setTurnVolts(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * SwerveConstants.WHEEL_RADIUS_METERS;
  }

  public double getVelocityRadPerSec() {
    return inputs.driveVelocityRadPerSec;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS_METERS;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}