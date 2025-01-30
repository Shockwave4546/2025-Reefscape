package org.dovershockwave;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
//  protected final SwerveSubsystem swerve;
//  private final VisionSubsystem vision;
  private final ElevatorSubsystem elevator;
  private final CoralRollersSubsystem coralRollers;
  private final AlgaeRollerSubsystem algaeRollers;
  private final CoralPivotSubsystem coralPivot;
  protected final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  protected final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
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
//                Pair.of(CameraType.REEF_CAMERA, new VisionIOPhotonVision(CameraType.REEF_CAMERA)),
//                Pair.of(CameraType.HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVision(CameraType.HUMAN_PLAYER_STATION_CAMERA)));

        elevator = new ElevatorSubsystem(new ElevatorIOSpark(ElevatorConstants.LEFT_SPARK_ID, ElevatorConstants.RIGHT_SPARK_ID), new LidarIOLaserCan());
        coralRollers = new CoralRollersSubsystem(new CoralRollersIOSpark(CoralRollersConstants.SPARK_ID));
        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIOSpark(AlgaeRollersConstants.SPARK_ID));
        coralPivot = new CoralPivotSubsystem(new CoralPivotIOSpark(CoralPivotConstants.WRIST_SPARK_ID, CoralPivotConstants.BIGGER_PIVOT_LEFT_SPARK_ID, CoralPivotConstants.BIGGER_PIVOT_RIGHT_SPARK_ID));
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
//                Pair.of(CameraType.REEF_CAMERA, new VisionIOPhotonVisionSim(CameraType.REEF_CAMERA, swerve::getPose)),
//                Pair.of(CameraType.HUMAN_PLAYER_STATION_CAMERA, new VisionIOPhotonVisionSim(CameraType.HUMAN_PLAYER_STATION_CAMERA, swerve::getPose)));

        elevator = new ElevatorSubsystem(new ElevatorIO() {}, new LidarIO() {});
        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {});
        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIO() {});
        coralPivot = new CoralPivotSubsystem(new CoralPivotIO() {});
        break;
      case REPLAY:
      default:
//        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
//        vision = new VisionSubsystem(swerve::addVisionMeasurement, Pair.of(CameraType.NONE, new VisionIO() {}), Pair.of(CameraType.NONE, new VisionIO() {}));
        elevator = new ElevatorSubsystem(new ElevatorIO() {}, new LidarIO() {});
        coralRollers = new CoralRollersSubsystem(new CoralRollersIO() {});
        algaeRollers = new AlgaeRollerSubsystem(new AlgaeRollersIO() {});
        coralPivot = new CoralPivotSubsystem(new CoralPivotIO() {});
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

//    if (!isCompetitionMatch()) {
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
//    }

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
//    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
//    driverController.b().onTrue(new ResetFieldOrientatedDriveCommand(swerve));
//    driverController.x().onTrue(new InstantCommand(swerve::stopWithX, swerve));
//
//    driverController.y().onTrue(new AlignToTagCommand(swerve, vision, CameraType.REEF_CAMERA));
//
//    driverController.leftBumper().onTrue(new InstantCommand(() -> swerve.multiplyFF(-0.1)).ignoringDisable(true));
//    driverController.rightBumper().onTrue(new InstantCommand(() -> swerve.multiplyFF(0.1)).ignoringDisable(true));
//    driverController.a().onTrue(new DriveLinearVelocityCommand(swerve, driverController, false));
//    driverController.b().onTrue(new DriveLinearVelocityCommand(swerve, driverController, true));
//
//    driverController.povDown().onTrue(new InstantCommand(() -> swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController))));
//    driverController.povUp().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().removeDefaultCommand(swerve)));
  }

  public static boolean isCompetitionMatch() {
    return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }
}
