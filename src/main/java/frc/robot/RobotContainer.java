// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterRotationManager;
import frc.robot.subsystems.shooter.ShooterSimulationIO;
import frc.robot.subsystems.shooter.ShooterSparkIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final ShooterRotationManager shooterRotationManager;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        shooterRotationManager =
            new ShooterRotationManager(() -> new Pose2d(), () -> drive.getPose());
        shooter =
            new Shooter(
                new ShooterSparkIO(Constants.ShooterConstants.ID),
                () -> shooterRotationManager.getDistance());

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        shooterRotationManager =
            new ShooterRotationManager(() -> new Pose2d(), () -> drive.getPose());
        shooter =
            new Shooter(new ShooterSimulationIO(), () -> shooterRotationManager.getDistance());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooterRotationManager =
            new ShooterRotationManager(() -> new Pose2d(), () -> drive.getPose());
        shooter = new Shooter(new ShooterIO() {}, () -> shooterRotationManager.getDistance());
        break;
    }

    // Set up auto routines
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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // EVENT TRIGGERS

    new EventTrigger("shoot").whileTrue(Commands.print("shoot"));
    new EventTrigger("feeder station").whileTrue(Commands.print("at feeder station")); // first feeder station
    new EventTrigger("shoot1l").whileTrue(Commands.print("shoot1l")); // first shoot from station 1 / left
    new EventTrigger("shoot1m").whileTrue(Commands.print("shoot1m")); // first shoot from station 2 / middle
    new EventTrigger("shoot1r").whileTrue(Commands.print("shoot1r")); // first shoot from station 3 / right
    new EventTrigger("feeder station1").whileTrue(Commands.print("feeder station 1")); // first feeder station 
    new EventTrigger("shoot2").whileTrue(Commands.print("feeder shoot")); // shooting OTW from feeder station 1 to 2
    new EventTrigger("feeder-station2").whileTrue(Commands.print("feeder station 2")); // second feeder station, the one where you open a gate
    new EventTrigger("shoot 3").whileTrue(Commands.print("feeder station 2 shoot")); // shooting in between feeder station 2 and climbing
    new EventTrigger("climb").whileTrue(Commands.print("climbing")); // at climbing

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        // DriveCommands.joystickDrive(
        //     drive,
        //     () -> -controller.getLeftY(),
        //     () -> -controller.getLeftX(),
        //     () -> -controller.getRightX()));

        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> {
              int pov = controller.getHID().getPOV(); // use dpad instead of right joystick
              if (pov == 90) return -0.75; // turn left
              if (pov == 270) return 0.75; // turn right
              return 0.0;
            }));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    controller.y().whileTrue(shooter.setManualSpeedCommand(0.25));
    controller.y().onFalse(shooter.setManualSpeedCommand(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerPath path;
    // try {
    //   path = PathPlannerPath.fromPathFile("shoot, climb");
    //   return AutoBuilder.followPath(path);
    // } catch (FileVersionException | IOException | ParseException e) {
    //   e.printStackTrace();
    //   return null;
    // }
    String osName = System.getProperty("os.name").toLowerCase();
    if (osName.contains("win")) {
      // Windows
      return autoChooser.get();
    } else if (osName.contains("nix") || osName.contains("nux")) {
      int station = DriverStation.getLocation().orElse(1);
      switch (station) {
        // switches paths easily on linux since no smart dashboard
        case 1:
          return new PathPlannerAuto("auto left feeder station");
        case 2:
          return new PathPlannerAuto("auto middle feeder station");
        case 3:
          return new PathPlannerAuto("auto right feeder station");
        default:
          return new PathPlannerAuto("auto left feeder station");
      }
    } else {
      return autoChooser.get();
    }
  }
}
