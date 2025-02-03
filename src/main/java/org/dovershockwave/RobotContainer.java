package org.dovershockwave;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollerSubsystem;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersConstants;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersIO;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersIOSpark;
import org.dovershockwave.subsystems.coralpivot.CoralPivotConstants;
import org.dovershockwave.subsystems.coralpivot.CoralPivotIO;
import org.dovershockwave.subsystems.coralpivot.CoralPivotIOSpark;
import org.dovershockwave.subsystems.coralpivot.CoralPivotSubsystem;
import org.dovershockwave.subsystems.coralrollers.CoralRollersConstants;
import org.dovershockwave.subsystems.coralrollers.CoralRollersIO;
import org.dovershockwave.subsystems.coralrollers.CoralRollersIOSpark;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.elevator.ElevatorConstants;
import org.dovershockwave.subsystems.elevator.ElevatorIO;
import org.dovershockwave.subsystems.elevator.ElevatorIOSpark;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.elevator.lidar.LidarIO;
import org.dovershockwave.subsystems.elevator.lidar.LidarIOLaserCan;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.swerve.commands.FeedforwardCharacterizationCommand;
import org.dovershockwave.subsystems.swerve.commands.ResetFieldOrientatedDriveCommand;
import org.dovershockwave.subsystems.swerve.commands.SwerveDriveCommand;
import org.dovershockwave.subsystems.swerve.commands.TemporaryHeadingCommand;
import org.dovershockwave.subsystems.swerve.commands.sysid.SysIdDriveDynamicCommand;
import org.dovershockwave.subsystems.swerve.commands.sysid.SysIdDriveQuasistaticCommand;
import org.dovershockwave.subsystems.swerve.commands.sysid.SysIdTurnQuasistaticCommand;
import org.dovershockwave.subsystems.swerve.gyro.GyroIO;
import org.dovershockwave.subsystems.swerve.module.ModuleIO;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSim;
import org.dovershockwave.subsystems.vision.*;
import org.dovershockwave.subsystems.vision.commands.AlignToHumanPlayerCommand;
import org.dovershockwave.subsystems.vision.commands.AlignToReefCoralCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  protected final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final ElevatorSubsystem elevator;
  private final CoralRollersSubsystem coralRollers;
  private final AlgaeRollerSubsystem algaeRollers;
  private final CoralPivotSubsystem coralPivot;
  protected final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);
  private final ReefScoringSelector selector = new ReefScoringSelector();

  protected final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // TODO: 2/2/2025 eventually get rid of this
    CanBridge.runTCP();
    switch (Constants.CURRENT_MODE) {
      case REAL:
        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        vision = new VisionSubsystem(swerve::addVisionMeasurement, Pair.of(CameraType.NONE, new VisionIO() {}), Pair.of(CameraType.NONE, new VisionIO() {}));
//        swerve = new SwerveSubsystem(
//                new GyroIONavX(),
//                new ModuleIOSpark(ModuleType.FRONT_LEFT),
//                new ModuleIOSpark(ModuleType.FRONT_RIGHT),
//                new ModuleIOSpark(ModuleType.BACK_LEFT),
//                new ModuleIOSpark(ModuleType.BACK_RIGHT));
//
//        vision = new VisionSubsystem(
//                swerve::addVisionMeasurement,
//                Pair.of(CameraType.LEFT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_REEF_CAMERA)),
//                Pair.of(CameraType.RIGHT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_REEF_CAMERA)),
//                Pair.of(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA)),
//                Pair.of(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA)));

        elevator = new ElevatorSubsystem(new ElevatorIOSpark(ElevatorConstants.LEFT_SPARK_ID, ElevatorConstants.RIGHT_SPARK_ID), new LidarIOLaserCan());
        coralRollers = new CoralRollersSubsystem(new CoralRollersIOSpark(CoralRollersConstants.SPARK_ID));
        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIOSpark(AlgaeRollersConstants.SPARK_ID));
        coralPivot = new CoralPivotSubsystem(new CoralPivotIOSpark(CoralPivotConstants.WRIST_SPARK_ID, CoralPivotConstants.BIGGER_PIVOT_LEFT_SPARK_ID, CoralPivotConstants.BIGGER_PIVOT_RIGHT_SPARK_ID));
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

        elevator = new ElevatorSubsystem(new ElevatorIO() {}, new LidarIO() {});
        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {});
        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIO() {});
        coralPivot = new CoralPivotSubsystem(new CoralPivotIO() {});
        break;
      case REPLAY:
      default:
        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        vision = new VisionSubsystem(swerve::addVisionMeasurement, Pair.of(CameraType.NONE, new VisionIO() {}), Pair.of(CameraType.NONE, new VisionIO() {}));
        elevator = new ElevatorSubsystem(new ElevatorIO() {}, new LidarIO() {});
        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {});
        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIO() {});
        coralPivot = new CoralPivotSubsystem(new CoralPivotIO() {});
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    if (!isCompetitionMatch()) {
      autoChooser.addOption("Drive Simple FF Characterization", new FeedforwardCharacterizationCommand(swerve));
      autoChooser.addOption("Drive SysId (Quasistatic Forward)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Quasistatic Reverse)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Drive SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));

      autoChooser.addOption("Turn SysId (Quasistatic Forward)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Turn SysId (Quasistatic Reverse)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Turn SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Turn SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));
    }

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driverController.leftTrigger(0.8).onTrue(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(0.5))).onFalse(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(1.0)));
    driverController.leftBumper().whileTrue(new InstantCommand(() -> {
      selector.setSide(ReefScoringPosition.ReefScoringSide.LEFT);
    }).andThen(new AlignToReefCoralCommand(swerve, vision, selector)));

    driverController.rightBumper().whileTrue(new InstantCommand(() -> {
      selector.setSide(ReefScoringPosition.ReefScoringSide.RIGHT);
    }).andThen(new AlignToReefCoralCommand(swerve, vision, selector)));

    driverController.a().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.CLOSE));
    driverController.x().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.CENTER));
    driverController.y().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.FAR));
    driverController.b().whileTrue(new TemporaryHeadingCommand(swerve, vision));

    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
    driverController.povDown().onTrue(new ResetFieldOrientatedDriveCommand(swerve));

    operatorController.povDown().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L1)));
    operatorController.povLeft().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L2)));
    operatorController.povUp().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L3)));
    operatorController.povRight().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L4)));

    // TODO: 2/2/25 Add Algae commands
  }

  public static boolean isCompetitionMatch() {
    return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }
}
