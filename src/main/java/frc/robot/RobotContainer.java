// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ChargeShooterWhenNeededCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedWhenValidCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederSimulationIO;
import frc.robot.subsystems.feeder.FeederSparkIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterRotationManager;
import frc.robot.subsystems.shooter.ShooterSimulationIO;
import frc.robot.subsystems.shooter.ShooterSparkIO;
import java.util.function.Supplier;
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
  private final Feeder feeder;
  private final Shooter shooter;
  private final ShooterRotationManager shooterRotationManager;

  private final Supplier<Pose2d> targetPose = () -> new Pose2d(4.76, 4, new Rotation2d());
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
        feeder = new Feeder(new FeederSparkIO(0));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        shooterRotationManager = new ShooterRotationManager(targetPose, drive);
        shooter = new Shooter(new ShooterSparkIO(), () -> shooterRotationManager.getDistance());

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
        feeder = new Feeder(new FeederSimulationIO());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        shooterRotationManager = new ShooterRotationManager(targetPose, drive);
        shooter =
            new Shooter(new ShooterSimulationIO(), () -> shooterRotationManager.getDistance());
        break;

      default:
        feeder = new Feeder(new FeederIO() {});
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooterRotationManager = new ShooterRotationManager(targetPose, drive);
        shooter =
            new Shooter(new ShooterSimulationIO(), () -> shooterRotationManager.getDistance());
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // decrease power of controller, orient shooter face targetPose
    Command orientDrive =
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY() * ShooterConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> -controller.getLeftX() * ShooterConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> shooterRotationManager.getHeading());

    // accelerate shooter to target speed(determined by distance), orient shooter, then start the
    // feeder to shoot
    // Command shootCommand =
    //     shooter
    //         .setAutomaticCommandConsistentEnd()
    //         .andThen(new WaitUntilCommand(() -> shooterRotationManager.isOriented()))
    //         .andThen(feeder.setSpeedRunCommand(0.25));

    // always orient drive and shoot at the same time... might be a little sloppy but a majority of

    Command feedWhenValid =
        new FeedWhenValidCommand(
            feeder,
            controller,
            shooter,
            shooterRotationManager,
            drive,
            () -> controller.a().getAsBoolean());

    // balls will make it
    Command shootSequence =
        Commands.parallel(orientDrive, shooter.setAutomaticCommand(), feedWhenValid);

    controller.leftTrigger().whileTrue(shootSequence);

    // Lock to 0° when A button is held
    controller
        .rightBumper()
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

    shooter.setDefaultCommand(
        new ChargeShooterWhenNeededCommand(
            shooter,
            () -> drive.getPose())); // make shooter go to this speed when it's not being used

    feeder.setDefaultCommand(feeder.setSpeedRunCommand(0));

    controller
        .rightTrigger()
        .whileTrue(shooter.setManualSpeedRunCommand(Constants.ShooterConstants.INTAKE_SPEED));

    controller.leftBumper().whileTrue(shooter.setAutomaticCommandRun());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public ShooterRotationManager getShooterRotationManager() {
    return shooterRotationManager;
  }

  public Drive getDrive() {
    return drive;
  }
}
