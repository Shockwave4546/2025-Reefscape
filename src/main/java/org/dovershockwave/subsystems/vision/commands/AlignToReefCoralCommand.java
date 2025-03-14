package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.controllers.FullAlignController;
import org.dovershockwave.subsystems.vision.controllers.XHeadingAlignController;
import org.littletonrobotics.junction.Logger;

public class AlignToReefCoralCommand extends Command {
  private final FullAlignController fullAlignController = new FullAlignController(
          "AlignToReefCoralCommand",
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID,
          VisionConstants.ALIGNMENT_RAD_TOLERANCE,
          VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE,
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED)
  );

  private final XHeadingAlignController xHeadingAlignController;

  private final SwerveSubsystem swerve;
  private final ReefScoringSelector selector;
  private final ReefScoringPosition.ReefScoringSide side;

  public AlignToReefCoralCommand(SwerveSubsystem swerve, ReefScoringSelector selector, ReefScoringPosition.ReefScoringSide side, CommandXboxController controller) {
    this.swerve = swerve;
    this.selector = selector;
    this.side = side;
    this.xHeadingAlignController = new XHeadingAlignController(
            "AlignToReefCoralL1Command",
            VisionConstants.ALIGNMENT_OMEGA_PID,
            VisionConstants.ALIGNMENT_X_VELOCITY_PID,
            VisionConstants.ALIGNMENT_RAD_TOLERANCE,
            VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
            new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
            new TrapezoidProfile.Constraints(SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND, SwerveConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED),
            () -> -controller.getLeftX() * SwerveConstants.MAX_REAL_SPEED_METERS_PER_SECOND * SwerveSubsystem.getVelocityMultiplier()
    );
    selector.setSide(side);
    addRequirements(swerve);
  }

  @Override public void initialize() {
    fullAlignController.resetPIDErrors(swerve.getPose());
    xHeadingAlignController.resetPIDErrors(swerve.getPose());
  }

  @Override public void execute() {
    ReefScoringPosition.getCoralPositionFor(swerve.getPose(), side, selector.getLevel()).ifPresentOrElse(position -> {
      final var goalPose = new Pose2d(
              position.position().toTranslation2d(),
              position.robotHeading()
      );
      final var speeds = selector.getLevel() == ReefScoringPosition.ReefLevel.L1 ?
              xHeadingAlignController.calculate(
                      swerve.getPose(),
                      goalPose,
                      String.valueOf(position.id())
              ) :

              fullAlignController.calculate(
                      swerve.getPose(),
                      goalPose,
                      String.valueOf(position.id())
              );

      Logger.recordOutput("AlignToReefCoralCommand/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefCoralCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralCommand/Omega", speeds.omegaRadiansPerSecond);

      swerve.runVelocityFieldRelative(speeds);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return selector.getLevel() == ReefScoringPosition.ReefLevel.L1 ? xHeadingAlignController.atGoal() : fullAlignController.atGoal();
  }
}