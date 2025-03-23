package org.dovershockwave;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.auto.CenterGAuto;
import org.dovershockwave.auto.CenterHAuto;
import org.dovershockwave.auto.StartingNonProcessorJKLAuto;
import org.dovershockwave.auto.StartingProcessorEDCAuto;
import org.dovershockwave.commands.*;
import org.dovershockwave.subsystems.algaepivot.*;
import org.dovershockwave.subsystems.algaerollers.*;
import org.dovershockwave.subsystems.coralpivot.*;
import org.dovershockwave.subsystems.coralrollers.*;
import org.dovershockwave.subsystems.coralrollers.commands.IndexCoralCommand;
import org.dovershockwave.subsystems.coralrollers.commands.IntakeCoralCommand;
import org.dovershockwave.subsystems.coralrollers.lidar.LidarIO;
import org.dovershockwave.subsystems.coralrollers.lidar.LidarIOLaserCan;
import org.dovershockwave.subsystems.elevator.*;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.swerve.commands.FeedforwardCharacterizationCommand;
import org.dovershockwave.subsystems.swerve.commands.ResetFieldOrientatedDriveCommand;
import org.dovershockwave.subsystems.swerve.commands.SwerveDriveCommand;
import org.dovershockwave.subsystems.swerve.commands.WheelRadiusCharacterizationCommand;
import org.dovershockwave.subsystems.swerve.commands.sysid.SysIdDriveDynamicCommand;
import org.dovershockwave.subsystems.swerve.commands.sysid.SysIdDriveQuasistaticCommand;
import org.dovershockwave.subsystems.swerve.commands.sysid.SysIdTurnQuasistaticCommand;
import org.dovershockwave.subsystems.swerve.gyro.GyroIO;
import org.dovershockwave.subsystems.swerve.gyro.GyroIONavX;
import org.dovershockwave.subsystems.swerve.module.ModuleIO;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSim;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSpark;
import org.dovershockwave.subsystems.swerve.module.ModuleType;
import org.dovershockwave.subsystems.vision.*;
import org.dovershockwave.subsystems.vision.commands.AlignToHumanPlayerCommand;
import org.dovershockwave.subsystems.vision.commands.AlignToReefAlgaeCommand;
import org.dovershockwave.subsystems.vision.commands.AlignToReefCoralCommand;
import org.dovershockwave.subsystems.vision.commands.AlignToReefCoralIntermediateCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  protected final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final ElevatorSubsystem elevator;
  private final CoralRollersSubsystem coralRollers;
  private final AlgaeRollersSubsystem algaeRollers;
  private final AlgaePivotSubsystem algaePivot;
  private final CoralPivotSubsystem coralPivot;
//  private final ClimbSubsystem climb;
  protected final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
  private final ReefScoringSelector selector = new ReefScoringSelector();

  protected final LoggedDashboardChooser<Command> autoChooser;
  protected final LoggedDashboardChooser<Command> testChooser = new LoggedDashboardChooser<>("Test Mode Choices");

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        swerve = new SwerveSubsystem(
                new GyroIONavX(),
                new ModuleIOSpark(ModuleType.FRONT_LEFT),
                new ModuleIOSpark(ModuleType.FRONT_RIGHT),
                new ModuleIOSpark(ModuleType.BACK_LEFT),
                new ModuleIOSpark(ModuleType.BACK_RIGHT));

        vision = new VisionSubsystem(
                swerve::addVisionMeasurement,
                Pair.of(CameraType.LEFT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_REEF_CAMERA, swerve::getRotationFromGyro)),
                Pair.of(CameraType.RIGHT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_REEF_CAMERA, swerve::getRotationFromGyro)),
                Pair.of(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, swerve::getRotationFromGyro)),
                Pair.of(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, swerve::getRotationFromGyro)));

        elevator = new ElevatorSubsystem(new ElevatorIOSpark(ElevatorConstants.LEFT_SPARK_ID, ElevatorConstants.RIGHT_SPARK_ID));
        coralRollers = new CoralRollersSubsystem(new CoralRollersIOSpark(CoralRollersConstants.SPARK_ID), new LidarIOLaserCan());
        algaeRollers = new AlgaeRollersSubsystem(new AlgaeRollersIOSpark(AlgaeRollersConstants.SPARK_ID));
        algaePivot = new AlgaePivotSubsystem(new AlgaePivotIOSpark(AlgaePivotConstants.SPARK_ID));
        coralPivot = new CoralPivotSubsystem(new CoralArmIOSpark(CoralPivotConstants.ARM_LEFT_SPARK_ID, CoralPivotConstants.ARM_RIGHT_SPARK_ID), new CoralWristIOSpark(CoralPivotConstants.WRIST_SPARK_ID));
//        climb = new ClimbSubsystem(new ClimbIOSpark(ClimbConstants.SPARK_ID));
        break;
      case SIM:
        swerve = new SwerveSubsystem(new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision = new VisionSubsystem(
                swerve::addVisionMeasurement,
                Pair.of(CameraType.LEFT_REEF_CAMERA, new VisionIOPhotonVisionSim(CameraType.LEFT_REEF_CAMERA, swerve::getPose)),
                Pair.of(CameraType.RIGHT_REEF_CAMERA, new VisionIOPhotonVisionSim(CameraType.RIGHT_REEF_CAMERA, swerve::getPose)),
                Pair.of(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVisionSim(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, swerve::getPose)),
                Pair.of(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVisionSim(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, swerve::getPose)));

        elevator = new ElevatorSubsystem(new ElevatorIO() {});
        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {}, new LidarIO() {});
        algaeRollers = new AlgaeRollersSubsystem(new AlgaeRollersIO() {});
        algaePivot = new AlgaePivotSubsystem(new AlgaePivotIO() {});
        coralPivot = new CoralPivotSubsystem(new CoralArmIO() {}, new CoralWristIO() {});
//        climb = new ClimbSubsystem(new ClimbIO() {});
        break;
      case REPLAY:
      default:
        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        vision = new VisionSubsystem(swerve::addVisionMeasurement, Pair.of(CameraType.NONE, new VisionIO() {}), Pair.of(CameraType.NONE, new VisionIO() {}));
        elevator = new ElevatorSubsystem(new ElevatorIO() {});
        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {}, new LidarIO() {});
        algaeRollers = new AlgaeRollersSubsystem(new AlgaeRollersIO() {});
        algaePivot = new AlgaePivotSubsystem(new AlgaePivotIO() {});
        coralPivot = new CoralPivotSubsystem(new CoralArmIO() {}, new CoralWristIO() {});
//        climb = new ClimbSubsystem(new ClimbIO() {});
    }

    testChooser.addDefaultOption("Do Nothing", new InstantCommand());
    testChooser.addOption("Wheel Radius Characterization", new WheelRadiusCharacterizationCommand(swerve));
    testChooser.addOption("Drive Simple FF Characterization", new FeedforwardCharacterizationCommand(swerve));

    testChooser.addOption("Drive SysId (Quasistatic Forward)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
    testChooser.addOption("Drive SysId (Quasistatic Reverse)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
    testChooser.addOption("Drive SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
    testChooser.addOption("Drive SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));

    testChooser.addOption("Turn SysId (Quasistatic Forward)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
    testChooser.addOption("Turn SysId (Quasistatic Reverse)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
    testChooser.addOption("Turn SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
    testChooser.addOption("Turn SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));

    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    registerPP();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driverController.leftTrigger(0.8).onTrue(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(0.5))).onFalse(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(1.0)));
    driverController.leftBumper()
            .whileTrue(new SequentialCommandGroup(
                    new ConditionalCommand(new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)), new InstantCommand(), () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4),
                    new AlignToReefCoralIntermediateCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.LEFT, driverController),
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
                                    new WaitUntilCommand(elevator::atDesiredState),
                                    new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
                                    new AlignToReefCoralCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.LEFT, driverController),
                                    new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.25)),
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
                                    new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
                                    new AlignToReefCoralCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.LEFT, driverController),
                                    new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.25))
                            , () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4)
            )).onFalse(new SequentialCommandGroup(
//                    new WaitUntilCommand(this::isSafeToStow),
                    new ParallelCommandGroup(
                            new ConditionalCommand(
                                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)),
                                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING)),
                                    () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4
                            ),
                            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers)
                    ),
                    new WaitUntilCommand(coralPivot::atDesiredState),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
            ));

    driverController.rightBumper()
            .whileTrue(new SequentialCommandGroup(
                    new ConditionalCommand(new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)), new InstantCommand(), () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4),
                    new AlignToReefCoralIntermediateCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.RIGHT, driverController),
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
                                    new WaitUntilCommand(elevator::atDesiredState),
                                    new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
                                    new AlignToReefCoralCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.RIGHT, driverController),
                                    new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.25)),
                            new SequentialCommandGroup(
                                    new InstantCommand(() -> elevator.setDesiredState(selector.getLevel()), elevator),
                                    new InstantCommand(() -> coralPivot.setDesiredState(selector.getLevel()), coralPivot),
                                    new AlignToReefCoralCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.RIGHT, driverController),
                                    new RunCommand(() -> coralRollers.setDesiredState(selector.getLevel()), coralRollers).withTimeout(0.25))
                            , () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4)
            )).onFalse(new SequentialCommandGroup(
//                    new WaitUntilCommand(this::isSafeToStow),
                    new ParallelCommandGroup(
                            new ConditionalCommand(
                                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)),
                                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING)),
                                    () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4
                            ),
                            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers)
                    ),
                    new WaitUntilCommand(coralPivot::atDesiredState),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
            ));

    driverController.rightTrigger(0.8).whileTrue(new SequentialCommandGroup(
            new FullAlgaeKnockoffCommand(swerve, coralPivot, elevator, coralRollers, selector)
//            new AlignToReefAlgaeCommand(swerve)
    )).onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
            new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
    ));

//    driverController.x()
//            .whileTrue(new SequentialCommandGroup(
//                    new AlignToHumanPlayerCommand(swerve, HumanPlayerStationPosition.HumanPlayerStationSide.CLOSE),
//                    new FullIntakeCoralCommand(coralPivot, coralRollers, elevator)
//            )).onFalse(new ParallelCommandGroup(
//                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
//                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
//                    new IndexCoralCommand(coralRollers)
//            ));

    driverController.a()
            .whileTrue(new FullIntakeCoralCommand(coralPivot, coralRollers, elevator))
            .onFalse(new SequentialCommandGroup(
                    new IntakeCoralCommand(coralRollers),
                    new IndexCoralCommand(coralRollers),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot)
            ));

//    driverController.b()
//            .whileTrue(new SequentialCommandGroup(
//                    new AlignToHumanPlayerCommand(swerve, HumanPlayerStationPosition.HumanPlayerStationSide.FAR),
//                    new FullIntakeCoralCommand(coralPivot, coralRollers, elevator)
//            )).onFalse(new ParallelCommandGroup(
//                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
//                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot),
//                    new IndexCoralCommand(coralRollers)
//            ));

    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
    SmartDashboard.putData("Reset Field Orientated Drive", new ResetFieldOrientatedDriveCommand(swerve));
    driverController.povLeft().onTrue(new ResetFieldOrientatedDriveCommand(swerve));

    operatorController.povDown().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L1)).ignoringDisable(true));
    operatorController.povLeft().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L2)).ignoringDisable(true));
    operatorController.povUp().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L3)).ignoringDisable(true));
    operatorController.povRight().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L4)).ignoringDisable(true));

    operatorController.leftBumper().onTrue(new IndexCoralCommand(coralRollers));
    operatorController.rightBumper().whileTrue(new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.L4_OUTTAKE)))
            .onFalse(new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED)));
//    operatorController.leftBumper().onTrue(new InstantCommand(() -> climb.setDesiredState(ClimbState.STARTING)));
//    operatorController.rightBumper().onTrue(new InstantCommand(() -> climb.setDesiredState(ClimbState.DOWN)));


    operatorController.leftTrigger(0.8).onTrue(new ResetStatesCommand(coralRollers, elevator, coralPivot, algaePivot, algaeRollers, selector));
    operatorController.rightTrigger(0.8).onTrue(new ResetStatesCommand(coralRollers, elevator, coralPivot, algaePivot, algaeRollers, selector));

    operatorController.a().onTrue(new FullScoreCoralCommand(coralPivot, coralRollers, elevator, selector))
            .onFalse(new SequentialCommandGroup(
//                    new WaitUntilCommand(this::isSafeToStow),
                    new ParallelCommandGroup(
                            new ConditionalCommand(
                                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)),
                                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING)),
                                    () -> selector.getLevel() == ReefScoringPosition.ReefLevel.L4
                            ),
                            new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED), coralRollers)
                    ),
                    new WaitUntilCommand(coralPivot::atDesiredState),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator)
            ));

    operatorController.y()
            .whileTrue(new FullIntakeCoralCommand(coralPivot, coralRollers, elevator))
            .onFalse(new SequentialCommandGroup(
                    new IntakeCoralCommand(coralRollers),
                    new IndexCoralCommand(coralRollers),
                    new InstantCommand(() -> elevator.setDesiredState(ElevatorState.STARTING), elevator),
                    new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING), coralPivot)
            ));

    operatorController.b().whileTrue(new FullScoreAlgaeCommand(algaePivot, algaeRollers));
    operatorController.x().whileTrue(new RunCommand(() -> {
      algaePivot.setDesiredState(AlgaePivotState.INTAKE);
      algaeRollers.setDesiredState(AlgaeRollersState.INTAKE);
    }, algaePivot, algaeRollers).finallyDo(() -> {
      algaePivot.setDesiredState(AlgaePivotState.INTAKE_AFTER);
      algaeRollers.setDesiredState(AlgaeRollersState.STOPPED);
    }));

    SmartDashboard.putData("Reset Elevator Pos", new InstantCommand(elevator::resetPosition).ignoringDisable(true));

    SmartDashboard.putData("One", new InstantCommand(() -> coralPivot.setDesiredState(new CoralPivotState(1.95, coralPivot.getDesiredState().armPositionRad()))));
    SmartDashboard.putData("Two", new InstantCommand(() -> coralPivot.setDesiredState(new CoralPivotState(1.95, CoralPivotState.MOVING_UP.armPositionRad()))));
    SmartDashboard.putData("Three", new InstantCommand(() -> coralPivot.setDesiredState(CoralPivotState.MOVING_UP)));
    SmartDashboard.putData("GetOutOfStarting", new GetOutOfStartingCommand(coralPivot));

    SmartDashboard.putData("COPY GetOutOfStarting COPY", new GetOutOfStartingCopyCommand(coralPivot));
  }

  /**
    * Checks if all detected AprilTags are at least 7 inches away
    */
  private boolean isSafeToStow() {
    return VisionConstants.APRIL_TAG_FIELD.getTags().stream()
            .map(tag -> tag.pose.getTranslation().toTranslation2d().getDistance(swerve.getPose().getTranslation()))
            .allMatch(distance -> distance > Units.inchesToMeters(7) + SwerveConstants.TRACK_WIDTH_METERS / 2.0);
  }

  private void registerPP() {
//    NamedCommands.registerCommand("AutoAlignScoreL4", new AlignToReefCoralCommand(swerve, selector, ReefScoringPosition.ReefScoringSide.LEFT, driverController).andThen(new FullScoreCoralCopyCommand(coralPivot, coralRollers, elevator, selector)));
//    NamedCommands.registerCommand("L4Positions", allL4PositionsCommand);
//    NamedCommands.registerCommand("Intake", new FullIntakeCoralCommand(coralPivot, coralRollers, elevator));
//    NamedCommands.registerCommand("ScoreL4", new AutoScoreCoralL4Command(coralPivot, coralRollers, elevator));
//    NamedCommands.registerCommand("GetOutOfStarting", new GetOutOfStartingCopyCommand(coralPivot));
//
//    new EventTrigger("L4Positions").onTrue(allL4PositionsCommand);
//    new EventTrigger("Intake").onTrue(new FullIntakeCoralCommand(coralPivot, coralRollers, elevator));

    swerve.initializePP();

    autoChooser.addOption("Center-H", new CenterHAuto(swerve, coralPivot, coralRollers, elevator, selector));
    autoChooser.addOption("Center-G", new CenterGAuto(swerve, coralPivot, coralRollers, elevator, selector));
    autoChooser.addOption("StartingNonProcessor-J-K-L", new StartingNonProcessorJKLAuto(swerve, coralPivot, coralRollers, elevator, selector));
    autoChooser.addOption("StartingProcessor-E-D-C", new StartingProcessorEDCAuto(swerve, coralPivot, coralRollers, elevator, selector));
  }

  public static boolean isCompetitionMatch() {
    return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }

  public static boolean isTuningMode() {
    return Constants.TUNING_MODE && !isCompetitionMatch();
  }
}