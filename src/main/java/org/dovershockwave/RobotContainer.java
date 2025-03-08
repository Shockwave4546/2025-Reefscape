package org.dovershockwave;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.commands.FullIntakeCoralCommand;
import org.dovershockwave.commands.FullScoreCoralCommand;
import org.dovershockwave.commands.FullScoreCoralL4Command;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotConstants;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotIO;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotIOSpark;
import org.dovershockwave.subsystems.algaepivot.AlgaePivotSubsystem;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersConstants;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersIO;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersIOSpark;
import org.dovershockwave.subsystems.algaerollers.AlgaeRollersSubsystem;
import org.dovershockwave.subsystems.coralpivot.*;
import org.dovershockwave.subsystems.coralrollers.CoralRollersConstants;
import org.dovershockwave.subsystems.coralrollers.CoralRollersIO;
import org.dovershockwave.subsystems.coralrollers.CoralRollersIOSpark;
import org.dovershockwave.subsystems.coralrollers.CoralRollersSubsystem;
import org.dovershockwave.subsystems.coralrollers.lidar.LidarIO;
import org.dovershockwave.subsystems.coralrollers.lidar.LidarIOLaserCan;
import org.dovershockwave.subsystems.elevator.ElevatorConstants;
import org.dovershockwave.subsystems.elevator.ElevatorIO;
import org.dovershockwave.subsystems.elevator.ElevatorIOSpark;
import org.dovershockwave.subsystems.elevator.ElevatorSubsystem;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.swerve.commands.*;
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
import org.dovershockwave.subsystems.vision.commands.AlignToReefAlgaeCommand;
import org.dovershockwave.subsystems.vision.commands.AlignToReefCoralCommand;
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
                Pair.of(CameraType.LEFT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_REEF_CAMERA, swerve::getRotation)),
                Pair.of(CameraType.RIGHT_REEF_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_REEF_CAMERA, swerve::getRotation)),
                Pair.of(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.LEFT_HUMAN_PLAYER_STATION_CAMERA, swerve::getRotation)),
                Pair.of(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.RIGHT_HUMAN_PLAYER_STATION_CAMERA, swerve::getRotation)));

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

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

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
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driverController.leftTrigger(0.9).onTrue(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(0.5))).onFalse(new InstantCommand(() -> SwerveSubsystem.setVelocityMultiplier(1.0)));
    driverController.leftBumper().whileTrue(new AlignToReefCoralCommand(swerve, vision, selector, ReefScoringPosition.ReefScoringSide.LEFT));
    driverController.rightBumper().whileTrue(new AlignToReefCoralCommand(swerve, vision, selector, ReefScoringPosition.ReefScoringSide.RIGHT));
    driverController.povDown().whileTrue(new AlignToReefAlgaeCommand(swerve, vision));

//    driverController.a().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.CLOSE));
//    driverController.x().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.CENTER));
//    driverController.y().whileTrue(new AlignToHumanPlayerCommand(swerve, vision, HumanPlayerStationPosition.HumanPlayerStationSide.FAR));
    driverController.b().whileTrue(new TemporaryHeadingCommand(swerve, vision));

    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
    SmartDashboard.putData("Reset Field Orientated Drive", new ResetFieldOrientatedDriveCommand(swerve));

    operatorController.povDown().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L1)));
    operatorController.povLeft().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L2)));
    operatorController.povUp().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L3)));
    operatorController.povRight().onTrue(new InstantCommand(() -> selector.setLevel(ReefScoringPosition.ReefLevel.L4)));

//    operatorController.leftBumper().onTrue(new InstantCommand(() -> climb.setDesiredState(ClimbState.STARTING)));
//    operatorController.rightBumper().onTrue(new InstantCommand(() -> climb.setDesiredState(ClimbState.DOWN)));

    operatorController.a().onTrue(new FullScoreCoralL4Command(swerve, vision, coralPivot, coralRollers, elevator, selector));
    operatorController.y().toggleOnTrue(new FullIntakeCoralCommand(coralPivot, coralRollers, elevator));
    // TODO: 2/2/25 Add Algae commands

    SmartDashboard.putData("Reset Elevator Pos", new InstantCommand(elevator::resetPosition).ignoringDisable(true));
  }

  public static boolean isCompetitionMatch() {
    return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }

  public static boolean isTuningMode() {
    return Constants.TUNING_MODE && !isCompetitionMatch();
  }
}