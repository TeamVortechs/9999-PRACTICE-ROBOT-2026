package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class ShooterSparkIO implements ShooterIO {
  final SparkMax m_motor;
  private CANcoder m_encoder;
  private double targetSpeed = 0;
  private boolean isOnTarget = false;

  public ShooterSparkIO(int id, CANcoder encoder) {
    m_motor = new SparkMax(id, MotorType.kBrushed);
    m_encoder = encoder;
  }

  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    inputs.amps = m_motor.getOutputCurrent();
    inputs.voltage = m_motor.getBusVoltage();
    inputs.speed = m_encoder.getVelocity().getValueAsDouble(); // in rotations per second

    inputs.targetSpeed = targetSpeed;
    inputs.isOnTarget = isOnTarget;
  }

  /*
   * sets speed -1 to 1
   */
  public void setSpeed(double speed) {
    targetSpeed = speed;
    m_motor.set(speed);
  }

  public void stop() {
    targetSpeed = 0;
    m_motor.set(0);
  }

  /*
   * returns speed in RPM
   */
  public double getSpeed() {
    return m_encoder.getVelocity().getValueAsDouble();
  }

  public boolean isOnTarget() {
    return Math.abs((getSpeed() - targetSpeed)) <= Constants.ShooterConstants.TOLERANCE;
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
