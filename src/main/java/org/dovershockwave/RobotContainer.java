package org.dovershockwave;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.subsystems.elevator.ElevatorConstants;
import org.dovershockwave.subsystems.elevator.ElevatorIO;
import org.dovershockwave.subsystems.elevator.ElevatorIOSpark;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
//  protected final SwerveSubsystem swerve;
//  private final VisionSubsystem vision;
  private final ElevatorSubsystem elevator;
//  private final CoralRollersSubsystem coralRollers;
//  private final AlgaeRollerSubsystem algaeRollers;
//  private final AlgaePivotSubsystem algaePivot;
//  private final CoralPivotSubsystem coralPivot;
//  private final ClimbSubsystem climb;
  protected final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
  private final ReefScoringSelector selector = new ReefScoringSelector();

  protected final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // TODO: 2/2/2025 eventually get rid of this
    CanBridge.runTCP();
    switch (Constants.CURRENT_MODE) {
      case REAL:
//        swerve = new SwerveSubsystem(
//                new GyroIONavX(),
//                new ModuleIOSpark(ModuleType.FRONT_LEFT),
//                new ModuleIOSpark(ModuleType.FRONT_RIGHT),
//                new ModuleIOSpark(ModuleType.BACK_LEFT),
//                new ModuleIOSpark(ModuleType.BACK_RIGHT));
//
//        vision = new VisionSubsystem(
//                swerve::addVisionMeasurement,
//                Pair.of(CameraType.LEFT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_REEF_CAMERA, swerve::getRotation)),
//                Pair.of(CameraType.RIGHT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_REEF_CAMERA, swerve::getRotation)),
//                Pair.of(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, swerve::getRotation)),
//                Pair.of(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, swerve::getRotation)));

        elevator = new ElevatorSubsystem(new ElevatorIOSpark(ElevatorConstants.LEFT_SPARK_ID, ElevatorConstants.RIGHT_SPARK_ID));
//        coralRollers = new CoralRollersSubsystem(new CoralRollersIOSpark(CoralRollersConstants.SPARK_ID));
//        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIOSpark(AlgaeRollersConstants.SPARK_ID));
//        algaePivot = new AlgaePivotSubsystem(new AlgaePivotIOSpark(AlgaePivotConstants.SPARK_ID));
//        coralPivot = new CoralPivotSubsystem(new CoralArmIOSpark(CoralPivotConstants.ARM_LEFT_SPARK_ID, CoralPivotConstants.ARM_RIGHT_SPARK_ID), new CoralWristIOSpark(CoralPivotConstants.WRIST_SPARK_ID));
//        climb = new ClimbSubsystem(new ClimbIOSpark(ClimbConstants.SPARK_ID));
        break;
      case SIM:
//        swerve = new SwerveSubsystem(new GyroIO() {},
//                new ModuleIOSim(),
//                new ModuleIOSim(),
//                new ModuleIOSim(),
//                new ModuleIOSim());
//
//        vision = new VisionSubsystem(
//                swerve::addVisionMeasurement,
//                Pair.of(CameraType.LEFT_REEF_CAMERA, new VisionIOPhotonVisionSim(CameraType.LEFT_REEF_CAMERA, swerve::getPose)),
//                Pair.of(CameraType.RIGHT_REEF_CAMERA, new VisionIOPhotonVisionSim(CameraType.RIGHT_REEF_CAMERA, swerve::getPose)),
//                Pair.of(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVisionSim(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, swerve::getPose)),
//                Pair.of(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVisionSim(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, swerve::getPose)));

        elevator = new ElevatorSubsystem(new ElevatorIO() {});
//        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {});
//        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIO() {});
//        algaePivot = new AlgaePivotSubsystem(new AlgaePivotIO() {});
//        coralPivot = new CoralPivotSubsystem(new CoralArmIO() {}, new CoralWristIO() {});
//        climb = new ClimbSubsystem(new ClimbIO() {});
        break;
      case REPLAY:
      default:
//        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
//        vision = new VisionSubsystem(swerve::addVisionMeasurement, Pair.of(CameraType.NONE, new VisionIO() {}), Pair.of(CameraType.NONE, new VisionIO() {}));
        elevator = new ElevatorSubsystem(new ElevatorIO() {});
//        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {});
//        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIO() {});
//        algaePivot = new AlgaePivotSubsystem(new AlgaePivotIO() {});
//        coralPivot = new CoralPivotSubsystem(new CoralArmIO() {}, new CoralWristIO() {});
//        climb = new ClimbSubsystem(new ClimbIO() {});
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    if (!isCompetitionMatch()) {
//      autoChooser.addOption("Drive Simple FF Characterization", new FeedforwardCharacterizationCommand(swerve));
//      autoChooser.addOption("Drive SysId (Quasistatic Forward)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
//      autoChooser.addOption("Drive SysId (Quasistatic Reverse)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
//      autoChooser.addOption("Drive SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
//      autoChooser.addOption("Drive SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));
//
//      autoChooser.addOption("Turn SysId (Quasistatic Forward)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
//      autoChooser.addOption("Turn SysId (Quasistatic Reverse)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
//      autoChooser.addOption("Turn SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
//      autoChooser.addOption("Turn SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));
    }

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
//    driverController.leftTrigger(0.8).onTrue(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(0.5))).onFalse(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(1.0)));
//    driverController.leftBumper().whileTrue(new InstantCommand(() -> {
//      selector.setSide(ReefScoringPosition.ReefScoringSide.LEFT);
//    }).andThen(new AlignToReefCoralCommand(swerve, vision, selector)));
//
//    driverController.rightBumper().whileTrue(new InstantCommand(() -> {
//      selector.setSide(ReefScoringPosition.ReefScoringSide.RIGHT);
//    }).andThen(new AlignToReefCoralCommand(swerve, vision, selector)));
//
//    driverController.a().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.CLOSE));
//    driverController.x().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.CENTER));
//    driverController.y().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.FAR));
//    driverController.b().whileTrue(new TemporaryHeadingCommand(swerve, vision));
//
//    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
//    driverController.povDown().onTrue(new ResetFieldOrientatedDriveCommand(swerve));
//
//    operatorController.povDown().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L1)));
//    operatorController.povLeft().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L2)));
//    operatorController.povUp().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L3)));
//    operatorController.povRight().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L4)));

//    operatorController.a().toggleOnTrue(new IntakeCoralCommand(coralRollers));
//    operatorController.b().onTrue(new SequentialCommandGroup(new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.OUTTAKE)), new WaitCommand(0.25), new InstantCommand(() -> coralRollers.setDesiredState(CoralRollersState.STOPPED))));
    // TODO: 2/2/25 Add Algae commands

    SmartDashboard.putData("Reset Elevator Pos", new InstantCommand(elevator::resetPosition).ignoringDisable(true));
  }

  public static boolean isCompetitionMatch() {
    return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }
}
