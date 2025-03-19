package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

public class AlignToReefCoralIntermediateCommand extends Command {
  private final FullAlignController fullAlignController = new FullAlignController(
          "AlignToReefCoralCommand",
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID,
          Units.degreesToRadians(2),
          Units.inchesToMeters(0.25),
          Units.inchesToMeters(0.25),
          new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
          new TrapezoidProfile.Constraints(2, 0.5),
          new TrapezoidProfile.Constraints(2, 0.5)
  );

  private final XHeadingAlignController xHeadingAlignController;

  private final SwerveSubsystem swerve;
  private final ReefScoringSelector selector;
  private final ReefScoringPosition.ReefScoringSide side;

  public AlignToReefCoralIntermediateCommand(SwerveSubsystem swerve, ReefScoringSelector selector, ReefScoringPosition.ReefScoringSide side, CommandXboxController controller) {
    this.swerve = swerve;
    this.selector = selector;
    this.side = side;
    this.xHeadingAlignController = new XHeadingAlignController(
            "AlignToReefCoralL1Command",
            VisionConstants.ALIGNMENT_OMEGA_PID,
            VisionConstants.ALIGNMENT_X_VELOCITY_PID,
            Units.degreesToRadians(2),
            Units.inchesToMeters(0.25),
            new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC, SwerveConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
            new TrapezoidProfile.Constraints(2, 1),
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
              position.intermediatePosition().toTranslation2d(),
              position.robotHeading()
      );
      var speeds = selector.getLevel() == ReefScoringPosition.ReefLevel.L1 ?
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
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      }

      Logger.recordOutput("AlignToReefCoralIntermediateCommand/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefCoralIntermediateCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralIntermediateCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralIntermediateCommand/Omega", speeds.omegaRadiansPerSecond);

      swerve.runVelocityFieldRelative(speeds);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return fullAlignController.atGoal();
  }
}