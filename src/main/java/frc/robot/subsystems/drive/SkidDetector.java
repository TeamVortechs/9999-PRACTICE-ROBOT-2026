package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class SkidDetector {
  private ChassisSpeeds oldSpeed;

  private double oldTimestamp;

  private BuiltInAccelerometer accelerometer;

  private boolean isSkidding = false;

  public SkidDetector() {
    accelerometer = new BuiltInAccelerometer();

    oldSpeed = new ChassisSpeeds();
    oldTimestamp = 0;
  }

  public void update(ChassisSpeeds chassisSpeeds) {
    double curTimestamp = Timer.getFPGATimestamp();
    double deltaTime = curTimestamp - oldTimestamp;

    double xAccel = accelerometer.getX() * 9.81;
    double yAccel = accelerometer.getY() * 9.81;

    ChassisSpeeds deltaSpeeds = chassisSpeeds.minus(oldSpeed);
    ChassisSpeeds encoderAcceleration = deltaSpeeds.div(deltaTime);

    if (encoderAcceleration.vxMetersPerSecond - xAccel > ShooterConstants.SKID_THRESHOLD
        || encoderAcceleration.vyMetersPerSecond - yAccel > ShooterConstants.SKID_THRESHOLD) {
      isSkidding = true;
    } else {
      isSkidding = false;
    }

    Logger.recordOutput("acceleration/xA", xAccel);
    Logger.recordOutput("acceleration/yA", yAccel);

    Logger.recordOutput("acceleration/x", encoderAcceleration.vxMetersPerSecond);
    Logger.recordOutput("acceleration/y", encoderAcceleration.vyMetersPerSecond);

    Logger.recordOutput("acceleration/deltaX", deltaSpeeds.vxMetersPerSecond);
    Logger.recordOutput("acceleration/deltaTime", deltaTime);

    oldSpeed = chassisSpeeds;
    oldTimestamp = curTimestamp;
  }

  public boolean isSkidding() {
    return isSkidding;
  }
}
