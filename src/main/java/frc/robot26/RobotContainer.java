package frc.robot26;

import static edu.wpi.first.units.Units.Feet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.simulation.SimConstants;
import frc.robot26.commands.DriveCommands;
import frc.robot26.commands.SnapCommands;
import frc.robot26.generated.TunerConstants;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.drive.DriveSimConfig;
import frc.robot26.subsystems.drive.GyroIO;
import frc.robot26.subsystems.drive.GyroIONavX;
import frc.robot26.subsystems.drive.GyroIOSim;
import frc.robot26.subsystems.drive.ModuleIO;
import frc.robot26.subsystems.drive.ModuleIOSim;
import frc.robot26.subsystems.drive.ModuleIOTalonFX;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@SuppressFBWarnings("NM_SAME_SIMPLE_NAME_AS_SUPERCLASS")
public class RobotContainer extends frc.lib.infrastructure.RobotContainer {

  // Subsystems
  private Drive drive;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(
          DriveSimConfig.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

  private LoggedDashboardChooser<Command> autoChooser;

  @Override
  public String getRobotName() {
    return "Robot26";
  }

  @Override
  public boolean matchesMacAddress(String macAddress) {
    // Replace with actual MAC address for Robot26 when known
    return true;
  }

  @Override
  public void initialize() {
    // Create IO implementations
    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                driveSimulation::setSimulationWorldPose);
        break;
      case SIM:
        super.configureDriveSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        drive.setPose(SimConstants.SIM_INITIAL_FIELD_POSE);

        boolean isCI = System.getenv("CI") != null;
        break;
      case REPLAY:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driveSimulation::setSimulationWorldPose);
        break;
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driveSimulation::setSimulationWorldPose);
        throw new IllegalStateException(
            "SimConstants.CURRENT_MODE was invalid: " + SimConstants.CURRENT_MODE);
    }

    NamedCommands.registerCommand(
        "AngleToHub",
        SnapCommands.snapToAngle(
                drive,
                () -> {
                  Translation2d hubToRobot =
                      SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
                  double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());
                  return new Rotation2d(angleToRobot);
                })
            .withTimeout(1));

    NamedCommands.registerCommand(
        "SnapToHub", SnapCommands.snapToRadius(drive, Feet.of(7.5)).withTimeout(2));
    NamedCommands.registerCommand(
        "SnapToHub12", SnapCommands.snapToRadius(drive, Feet.of(11.5)).withTimeout(3));

    // NamedCommands.registerCommand(
    // "AutoShoot", RollerCommands.shootOpenLoop(shooter, floor, feeder,
    // intake).withTimeout(3));

    // NamedCommands.registerCommand("SnapToRadius", DriveCommands.snapToRadius(drive,
    // Feet.of(10.0)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  public RobotContainer() {
    // Constructor must be empty for ServiceLoader
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX()));

    // =========================================
    // ============ Driver Controls ============
    // =========================================

    // Reset gyro to 0when B button is pressed
    driverController
        .start()
        .or(driverController.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // driverController.a().whileTrue(SnapCommands.snapToRadius(drive, Feet.of(7.5)));
    // driverController.y().whileTrue(SnapCommands.snapToRadius(drive, Feet.of(11.5)));
    driverController.y().whileTrue(SnapCommands.tuneableSnapToRadius(drive));

    // Lock to hub when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> {
    //               Translation2d hubToRobot =
    //                   SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
    //               double angleToRobot = Math.PI + Math.atan2(hubToRobot.getY(),
    // hubToRobot.getX());
    //               Rotation2d targetAngle = new Rotation2d(angleToRobot);

    //               // Apply 0.5 degree deadzone — hold current heading if within threshold
    //               Rotation2d currentAngle = drive.getPose().getRotation();
    //               double errorDegrees = Math.abs(targetAngle.minus(currentAngle).getDegrees());
    //               if (errorDegrees < 1) {
    //                 return currentAngle;
    //               }
    //               return targetAngle;
    //             }));
  }

  private boolean hasBeenInTeleop = false;

  @Override
  public void teleopInit() {
    // drive.swerveBreak(true);
    hasBeenInTeleop = true;
  }

  @Override
  public void simulationInit() {
    if (!(SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena)) return;

    arena.getBlueHub().setOnScoredCallback((gp) -> System.out.println("Blue Hub Scored!"));
    arena.getRedHub().setOnScoredCallback((gp) -> System.out.println("Red Hub Scored!"));
    arena.setNeutralFuelCount(80); // for performance
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void disabledInit() {}

  @Override
  public void simulationPeriodic() {
    if (!(SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena)) return;

    Logger.recordOutput(
        "FieldSimulation/FuelPositions", arena.getGamePieceManager().getPosesArrayByType("Fuel"));
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput(
        "Drive/DistanceToHubCenterFeet", SnapCommands.distanceToHub(drive).in(Feet));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser != null ? autoChooser.get() : Commands.none();
  }

  @Override
  public Command getTestCommand() {
    return Commands.print("Test command!");
  }

  @Override
  public Pose2d getRobotPose() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRobotPose'");
  }
}
