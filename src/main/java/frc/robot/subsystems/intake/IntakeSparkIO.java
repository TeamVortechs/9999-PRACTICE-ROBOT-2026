package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class IntakeSparkIO implements IntakeIO {
  final SparkMax m_motor = new SparkMax(Constants.IntakeConstants.ID, MotorType.kBrushed);
  private final double targetSpeed = 0;
  private final boolean isOnTarget = false;

  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    inputs.amps = m_motor.getOutputCurrent();
    inputs.voltage = m_motor.getBusVoltage();
    inputs.speed = 0;

    inputs.targetSpeed = targetSpeed;
    inputs.isOnTarget = isOnTarget;
  }

  /*
   * sets speed -1 to 1
   */
  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.set(0);
  }

  /*
   * returns speed in RPM
   */
  public double getSpeed() {
    return 0;
  }

  public boolean isOnTarget() {
    // return Math.abs((getSpeed() - targetSpeed)) <= Constants.IntakeConstants.TOLERANCE;
    return false;
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}