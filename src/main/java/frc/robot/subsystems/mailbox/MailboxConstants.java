package frc.robot.subsystems.mailbox;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class MailboxConstants {
  public static final SparkBaseConfig CONFIG = new SparkMaxConfig();
  public static final int MAILBOX_CAN_ID = 8;

  public static final double REDUCTION = 1.0;
  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / REDUCTION;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / REDUCTION) / 60.0;

  static {
    CONFIG
            .inverted(false)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT) // TODO: 3/4/2025
            .voltageCompensation(12.0);
    CONFIG.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / Constants.ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
