package org.dovershockwave.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.dovershockwave.ReefScoringPosition;
import org.dovershockwave.ReefScoringSelector;
import org.dovershockwave.subsystems.coralpivot.CoralPivotState;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersState;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorState;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.VisionConstants;

import java.util.Map;

public class FullAlgaeKnockoffCommand extends ParallelCommandGroup {
  private static final Map<Integer, AlgaeHeight> BLUE_ALGAE_HEIGHTS = Map.of(
          18, AlgaeHeight.L3,
          19, AlgaeHeight.L2,
          20, AlgaeHeight.L3,
          21, AlgaeHeight.L2,
          22, AlgaeHeight.L3,
          17, AlgaeHeight.L2
  );

  private static final Map<Integer, AlgaeHeight> RED_ALGAE_HEIGHTS = Map.of(
          7, AlgaeHeight.L3,
          6, AlgaeHeight.L2,
          11, AlgaeHeight.L3,
          10, AlgaeHeight.L2,
          9, AlgaeHeight.L3,
          8, AlgaeHeight.L2
  );

  private final SwerveSubsystem swerve;
  private final ReefScoringSelector selector;

  public FullAlgaeKnockoffCommand(SwerveSubsystem swerve, CoralPivotSubsystem coralPivot, ElevatorSubsystem elevator, CoralRollersSubsystem coralRollers, ReefScoringSelector selector) {
    this.swerve = swerve;
    this.selector = selector;

    addCommands(
            new ConditionalCommand(
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.ALGAE_KNOCK_OFF_L2), coralPivot),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.ALGAE_KNOCK_OFF_L3), coralPivot),
                    () -> getClosestAlgaeHeight() == AlgaeHeight.L2
            ),
            new ConditionalCommand(
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.KNOCKOFF_ALGAE_L2), elevator),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.KNOCKOFF_ALGAE_L3)),
                    () -> getClosestAlgaeHeight() == AlgaeHeight.L2
            ),

            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.KNOCKOFF_ALGAE), coralRollers)
    );
  }

  private int getClosestAlgaeTagId() {
    return VisionConstants.APRIL_TAG_FIELD.getTags().stream()
            .filter(tag ->
                    (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ?
                            BLUE_ALGAE_HEIGHTS.containsKey(tag.ID) :
                            RED_ALGAE_HEIGHTS.containsKey(tag.ID))
            .min((tag1, tag2) -> {
              final var pose1 = VisionConstants.APRIL_TAG_FIELD.getTagPose(tag1.ID).orElse(null);
              final var pose2 = VisionConstants.APRIL_TAG_FIELD.getTagPose(tag2.ID).orElse(null);

              if (pose1 == null || pose2 == null) {
                return 0; // Treat as equally distant if either pose is missing
              }

              return Double.compare(
                      swerve.getPose().getTranslation().getDistance(pose1.toPose2d().getTranslation()),
                      swerve.getPose().getTranslation().getDistance(pose2.toPose2d().getTranslation())
              );
            })
            .map(tag -> tag.ID) // Extract the ID of the closest tag
            .orElse(-1); // Return -1 if no tag was found
  }

  private AlgaeHeight getClosestAlgaeHeight() {
    final var closestId = getClosestAlgaeTagId();
    if (closestId == -1) return selector.getLevel() == ReefScoringPosition.ReefLevel.L2 ? AlgaeHeight.L2 : AlgaeHeight.L3;
    final var alliance = DriverStation.getAlliance().get();
    if (alliance == DriverStation.Alliance.Blue) {
      return BLUE_ALGAE_HEIGHTS.get(closestId);
    } else {
      return RED_ALGAE_HEIGHTS.get(closestId);
    }
  }

  public enum AlgaeHeight {
    L2, L3
  }
}