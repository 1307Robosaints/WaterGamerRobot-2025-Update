package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ElevatorConfigs {
  public static final SparkBaseConfig LEFT_CONFIG = new SparkMaxConfig();

  static {
    LEFT_CONFIG
            .inverted(ElevatorConstants.DIRECTION_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .voltageCompensation(12.0);
    LEFT_CONFIG.encoder
            .positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(10);
    LEFT_CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.PID_GAINS.p(), ElevatorConstants.PID_GAINS.i(), ElevatorConstants.PID_GAINS.d(), ElevatorConstants.PID_GAINS.ff());
    LEFT_CONFIG.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / 100))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}
