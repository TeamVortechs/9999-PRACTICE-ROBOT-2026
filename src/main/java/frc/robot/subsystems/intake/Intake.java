package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  // this shouldn't be here but it is for now because we're probably gonna move this
  public static final double TOLERANCE = IntakeConstants.TOLERANCE;

  // just here for the logging
  @AutoLogOutput private double speed = 0;

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs;

  /**
   * @param intakeIO the hardware interface
   */
  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    this.inputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("detection", inputs);

    // calculate speed that automatically updates with distance
    // automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());

    double speed = getSpeedTarget();
    intakeIO.setSpeed(speed);
  }

  // SUBSYSTEM METHODS

  /**
   * @param speed the speed the flywheel will pid too
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * @return the target speed that the flywheel is pid'ing to. Can be the manual or the automatic
   *     calculated speed
   */
  public double getSpeedTarget() {
    return speed;
  }

  /**
   * @return wether the speed is the target speed
   */
  public boolean isOnTarget() {
    return intakeIO.isOnTarget();
  }

  // COMMANDS
  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setSpeedCommand(double speed) {
    return new InstantCommand(() -> this.setSpeed(speed));
  }

  /**
   * sets the target speed command then ends when it reaches that speed
   *
   * @param speed the speed it gets set to
   * @return the finished command
   */
  public Command setSpeedCommandConsistentEnd(double speed) {
    return new InstantCommand(() -> this.setSpeed(speed))
        .andThen(new WaitUntilCommand(() -> this.isOnTarget()));
  }

  // the constants here should probably be more and move but that's later when this is transferred
  // to the right project
  // add this to the robot class or this won't work: SignalLogger.setPath("/media/sda1/");
  /**
   * Gets the system identification routine for this specific subsystem
   *
   * @return the sysid routine
   */
  public SysIdRoutine BuildSysIdRoutine() {

    SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(IntakeConstants.RAMP_RATE_VOLTS_SYSID)
                    .per(Seconds), // Ramp Rate in Volts / Seconds
                Volts.of(IntakeConstants.DYNAMIC_STEP_VOLTS_SYSID), // Dynamic Step Voltage
                null, // Use default timeout (10 s)
                (state) ->
                    SignalLogger.writeString(
                        "state", state.toString()) // Log state with Phoenix SignalLogger class
                ),
            new SysIdRoutine.Mechanism(
                (volts) -> intakeIO.setVoltage(volts.in(Volts)), null, this));
    return m_SysIdRoutine;
  }

  // this shoudl be in a helper method or somewhere in robot container
  /**
   * Binds the sys id routine to the controller so we can use it
   *
   * <p>pov up: dynamic forward pov down: dynamic backwards pov right: quasistatic forward pov left:
   * quasistatic reverse
   *
   * @param controller the controller this binds to(recommended to use a high id controller to
   *     prevent mishaps, id 2-3)
   * @param sysIdRoutine the routine that this controller will activate
   */
  public void configureSysIdBindings(CommandXboxController controller, SysIdRoutine sysIdRoutine) {
    controller.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    controller.povDown().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    controller.povRight().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    controller.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));
  }
}

/*
 * two modes: supply a distance and have it wind up to that or
 * supply hand values and have it wind up to that
 *
 */

 /* sysid routine:
 *   // To-do: Move sysId settings to the constants file
 public SysIdRoutine BuildSysIdRoutine()
 {
   this.m_SysIdRoutine = new SysIdRoutine(
     new SysIdRoutine.Config(
        Volts.of(0.25).per(Seconds),  // Ramp Rate in Volts / Seconds
        Volts.of(1), // Dynamic Step Voltage
        null,          // Use default timeout (10 s)
        (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
     ),
     new SysIdRoutine.Mechanism(
        (volts) -> m_ElevatorMotor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
     )
  );
  return this.m_SysIdRoutine;
 }


 want to slow ramp on elevator because it is has a fast hardstop

 can use sysid on drivetrain
 must all be done in one enable



   private void configureElevatorDebugBindings()
 {
   Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
   SmartDashboard.putData(elevator);

   CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();

   SysIdRoutine sysIdRoutine = elevator.BuildSysIdRoutine();

   controller.a().onTrue(elevator.MoveToLevel(HEIGHTS.ONE));
   controller.b().onTrue(elevator.MoveToLevel(HEIGHTS.TWO));
   controller.y().onTrue(elevator.MoveToLevel(HEIGHTS.THREE));
   controller.x().onTrue(elevator.MoveToLevel(HEIGHTS.FOUR));

   controller.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
   controller.povRight().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
   controller.povDown().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
   controller.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

   System.out.println("[Wolfpack] Elevator Debug bindings successfully configured.");
 }


     SignalLogger.setPath("/media/sda1/");


     take log -> convert to wpilog -> put in sysid -> select correct motor(the motor you ran it on) -> expand logs on that motor -> look for log entry called state -> pull velocity/position measurements depending on wether you are doing velocity or position measurements


     look over at the displacement from the previous frame to the current frame. We wanna see if it's the filter or not

     plot framerate over time


     can get away with one camera for just

     get away with as many cameras as you need to get full 360 degree coverage

     use hue saturation value for the object detection
     */
