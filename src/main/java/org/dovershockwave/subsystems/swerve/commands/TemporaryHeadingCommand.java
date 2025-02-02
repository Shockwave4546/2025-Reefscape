package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

/**
 * Align to an AprilTag with a temporary heading, so that driving can be more seamless.
 */
public class TemporaryHeadingCommand extends Command {
  private final SwerveSubsystem swerve;
  private Rotation2d previousHeading;

  public TemporaryHeadingCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    previousHeading = swerve.getRotation();

    // TODO: 2/2/25 idk what camera to use for getting the apriltag jropifwaklv 
  }

  @Override public void end(boolean interrupted) {
    swerve.setPose(new Pose2d(swerve.getPose().getTranslation(), previousHeading));
  }
}