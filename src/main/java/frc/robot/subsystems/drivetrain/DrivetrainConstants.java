package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.PIDFGains;

public class DrivetrainConstants {
  public static final int FRONT_LEFT_CAN_ID = 4;
  public static final int FRONT_RIGHT_CAN_ID = 1;
  public static final int BACK_LEFT_CAN_ID = 3;
  public static final int BACK_RIGHT_CAN_ID = 2;
  
  public static final boolean DRIVE_MOTOR_INVERTED = false;
  public static final double DRIVE_MOTOR_REDUCTION = 10.71;
  public static final double DRIVE_ENCODER_POSITION_FACTOR = (2 * Math.PI) / DRIVE_MOTOR_REDUCTION;
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((2 * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0;
  public static final PIDFGains DRIVE_PIDF = new PIDFGains(0.0, 0.0, 0.0, 0.0);

  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;
  public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;
  public static final PIDFGains TURN_PIDF = new PIDFGains(0.0, 0.0, 0.0, 0.0);

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3);

  public static final double ROBOT_LENGTH_X_METERS = 0.82;
  public static final double ROBOT_LENGTH_Y_METERS = 0.67;
  
  /**
   * Distance between centers of left and right wheels.
   */
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22);

  /**
   * Distance between front and back wheels on the robot.
   */
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(20);

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
    new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
    new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
    new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)
  };

  public static final double MAX_REAL_SPEED_METERS_PER_SECOND = 3.8;
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Units.degreesToRadians(540);
  public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED = 6.0; // Example value, adjust as needed.
  public static final double MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
}
