package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterRotationManager {
  private Supplier<Pose2d> targetPose;
  private Drive drive;

  @AutoLogOutput private double distance = 0;
  @AutoLogOutput private double currentRadians;
  @AutoLogOutput private double targetRadians;
  @AutoLogOutput private boolean onTarget = false;
  /**
   * @param targetPose the pose of the area we want to shoot too
   * @param drive the pose of the robot
   */
  public ShooterRotationManager(Supplier<Pose2d> targetPose, Drive drive) {
    this.targetPose = targetPose;
    this.drive = drive;
  }

  /**
   * Get the distance from the robot pose to the target pose
   *
   * @return the distance in meters
   */
  public double getDistance() {
    distance = targetPose.get().getTranslation().getDistance(getPoseLeaded().getTranslation());

    return distance;
  }

  /**
   * Get the heading from target pose to the robot pose(FIELD CENTRIC)
   *
   * @return field centric heading
   */
  public Rotation2d getHeading() {
    // makes it so the robot will rotate towards where it is moving when driving to the pose
    Translation2d delta = targetPose.get().getTranslation().minus(getPoseLeaded().getTranslation());

    Rotation2d heading = new Rotation2d(delta.getX(), delta.getY());

    getDistance();

    return heading;
  }

  /**
   * Robot relative rotation to make the object point towards the target pose
   *
   * @return robot relative rotation
   */
  public Rotation2d getRobotRelativeRotation() {
    Rotation2d heading = getHeading();
    Rotation2d robotRotation = getPoseLeaded().getRotation();

    return heading.minus(robotRotation);
  }

  /**
   * converts the rotation to an encoder value on the motor so we can rotate it towards it(MAYBE
   * MOVE INTO MOTOR CLASS)
   *
   * @param rotation the rotation
   * @return the encoder value to aim towards in the PID loop
   */
  public static double convertRotationToMotorVal(Rotation2d rotation) {
    return 0;
  }

  /**
   * converts the distance into arm elevation to shoot towards it
   *
   * @param distance the distance in meters
   * @return the arm encoder value to aim towards
   */
  public static double convertDistToShooterAngle(double distance) {
    return 0;
  }

  /**
   * converts the distance into arm elevation to shoot towards it
   *
   * @param distance the distance in meters
   * @return the arm encoder value to aim towards
   */
  public boolean isOriented() {

    targetRadians = getHeading().getRadians();
    currentRadians = getPoseLeaded().getRotation().getRadians();

    onTarget =
        Math.abs(targetRadians - currentRadians) < Constants.ShooterConstants.ORIENTATION_TOLERANCE;

    return onTarget;
  }

  /**
   * gets the predicted pose after a k amount of seconds. This was we can adjust for robot lag and shooting lag
   * @return
   */
  public Pose2d getPoseLeaded() {
    Pose2d firstPose = drive.getPose();
    ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();

    //this scaling factor is a constnat we'll just need to test for. We can change it depending on if the shot is compensation to much or not enough. We can also make it zero to remove it
    ChassisSpeeds chassisSpeedsScaled = chassisSpeeds.times(ShooterConstants.LEAD_TIME_POSE_SEC);

    Transform2d speedsScaledTrans = new Transform2d(chassisSpeedsScaled.vxMetersPerSecond, chassisSpeedsScaled.vyMetersPerSecond, new Rotation2d());
    Pose2d updatedPose = firstPose.plus(speedsScaledTrans);
    
    return updatedPose;
  }


}
