package frc.robot.subsystems.mailboxpivot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utils.ArmFeedforwardGains;
import frc.robot.utils.ElevatorFeedforwardGains;
import frc.robot.utils.PIDFGains;

public class MailboxPivotConstants {
  public static final SparkBaseConfig CONFIG = new SparkMaxConfig();

  public static final int MAILBOX_ROT_CAN_ID = 6;
  public static final double REDUCTION = 15.0; // TODO: 3/4/2025
  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / REDUCTION;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / REDUCTION) / 60.0;
  public static final PIDFGains GAINS = new PIDFGains(0.0, 0.0, 0.0, 0.0);

  public static final double MIN_POS = 0.0;
  public static final double MAX_POS = 0.0;

  public static final ArmFeedforwardGains FEEDFORWARD_GAINS = new ArmFeedforwardGains(0.0, 0.0, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(2);

  static {
    CONFIG
            .inverted(true)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.NEO_550_CURRENT_LIMIT) 
            .voltageCompensation(12.0);
    CONFIG.absoluteEncoder
            .inverted(false)
            .zeroCentered(true)
            .positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR)
            .averageDepth(2);
    CONFIG.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
            .pidf(GAINS.p(), GAINS.i(), GAINS.d(), GAINS.ff());
    CONFIG.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / Constants.ODOMETRY_FREQUENCY))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
  }
}