package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class SkidDetectorPigeon2 {

  private Pigeon2 pigeon;

  private boolean isSkidding = false;

  public SkidDetectorPigeon2(int id, CANBus canbus) {
    pigeon = new Pigeon2(id, canbus);
  }

  public void update(ChassisSpeeds chassisSpeeds) {

    double xAccel = (pigeon.getAccelerationY().getValueAsDouble() - pigeon.getGravityVectorX().getValueAsDouble()) * 9.81;
    double yAccel = pigeon.getAccelerationX().getValueAsDouble() * 9.81;

    if (xAccel > ShooterConstants.SKID_THRESHOLD || yAccel > ShooterConstants.SKID_THRESHOLD) {
      isSkidding = true;
    } else {
      isSkidding = false;
    }

    Logger.recordOutput("acceleration/xA", xAccel);
    Logger.recordOutput("acceleration/yA", yAccel);

    // Logger.recordOutput("acceleration/x", encoderAcceleration.vxMetersPerSecond);
    // Logger.recordOutput("acceleration/y", encoderAcceleration.vyMetersPerSecond);

    // Logger.recordOutput("acceleration/deltaX", deltaSpeeds.vxMetersPerSecond);
    // Logger.recordOutput("acceleration/deltaTime", deltaTime);

    // oldSpeed = chassisSpeeds;
    // oldTimestamp = curTimestamp;
  }

  public boolean isSkidding() {
    return isSkidding;
  }
}
