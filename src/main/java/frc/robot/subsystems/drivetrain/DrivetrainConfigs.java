package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

public class DrivetrainConfigs {
  public static final SparkBaseConfig DRIVE_CONFIG = new SparkMaxConfig();

  static {
    DRIVE_CONFIG
            .inverted(DRIVE_MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    DRIVE_CONFIG.encoder
            .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    DRIVE_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(DRIVE_PIDF.p(), DRIVE_PIDF.i(), DRIVE_PIDF.d(), DRIVE_PIDF.ff());
    DRIVE_CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
