package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterRotationManager;
import org.littletonrobotics.junction.AutoLogOutput;

/*
Names
brief description

Runs the feeder only when the shot is valid
*/
// rename ts
public class FeedWhenValidCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  /** timer since it was last valid. The code waits a certain time before it is ok to shooot */
  private Timer validityTimer;

  // purely for logging purposes
  @AutoLogOutput(key = "Commands/FeedWhenValid/validitytime")
  private double validityTime = 0;

  private Feeder feeder;

  // controller to give feedback too
  private CommandXboxController controller;

  // these are just used for validity requirements
  private Shooter shooter;
  private ShooterRotationManager shooterRotationManager;

  /**
   * @param feeder only mechanism this command contorls
   * @param controller gives feedback
   */
  public FeedWhenValidCommand(
      Feeder feeder,
      CommandXboxController controller,
      Shooter shooter,
      ShooterRotationManager shooterRotationManager) {
    // addRequirements(null);

    this.feeder = feeder;
    this.controller = controller;

    this.shooter = shooter;
    this.shooterRotationManager = shooterRotationManager;

    validityTimer = new Timer();

    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    validityTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log();

    if (!isFeedable()) {
      feeder.setSpeed(0);
      controller.setRumble(RumbleType.kBothRumble, 0);
      return;
    }

    feeder.setSpeed(FeederConstants.FEED_POWER);

    // purely sim feedback
    controller.setRumble(RumbleType.kBothRumble, 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setSpeed(0);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @AutoLogOutput(key = "Commands/FeedWhenValid/isvalid")
  /** calculates wether or not it is currently ok to make a shot */
  private boolean isValid() {
    if (!shooter.isOnTarget()) return false;
    if (!shooterRotationManager.isOriented()) return false;

    // we can also do a slip test here by comparing acceleraomter and chassis speeds

    // we can also check for jerk here

    return true;
  }

  @AutoLogOutput(key = "Commands/FeedWhenValid/isfeedable")
  /** takes debounce time into consideration to see if we should we feed right away or still wait */
  private boolean isFeedable() {

    if (!isValid()) {
      validityTimer.restart();
      return false;
    }

    return validityTimer.hasElapsed(ShooterConstants.VALIDITY_DEBOUNCE_TIME_SEC);
  }

  private void log() {
    validityTime = validityTimer.get();
  }
}
