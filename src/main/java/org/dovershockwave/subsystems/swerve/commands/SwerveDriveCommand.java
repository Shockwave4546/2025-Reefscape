package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.Constants;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;

  public SwerveDriveCommand(SwerveSubsystem swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override public void execute() {
    final var xVelocity = Constants.CURRENT_MODE == Constants.Mode.REAL ? -controller.getLeftY() : controller.getLeftY();
    final var yVelocity = Constants.CURRENT_MODE == Constants.Mode.REAL ? -controller.getLeftX() : controller.getLeftX();
    final var linearVelocity = getLinearVelocityFromJoysticks(xVelocity, yVelocity);
    final var speeds = getChassisSpeeds(linearVelocity);
    final var isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    swerve.runVelocityFieldRelative(speeds);
//    swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
//            speeds,
//            isFlipped ? swerve.getRotation().plus(new Rotation2d(Math.PI)) : swerve.getRotation()), false);
  }

  private ChassisSpeeds getChassisSpeeds(Translation2d linearVelocity) {
    var omega = MathUtil.applyDeadband(-controller.getRightX(), Constants.DRIVE_DEADBAND);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    return new ChassisSpeeds(
            linearVelocity.getX() * SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND * SwerveSubsystem.getVelocityMultiplier(),
            linearVelocity.getY() * SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND * SwerveSubsystem.getVelocityMultiplier(),
            omega * SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * SwerveSubsystem.getVelocityMultiplier()
    );
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.DRIVE_DEADBAND);
    final var linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
  }
}