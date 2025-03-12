package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.PIDFGains;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ElevatorFeedforwardGains;

public class ElevatorConstants {
  public static final int LEFT_SPARK_ID = 5; // Neo

  public static final boolean DIRECTION_INVERTED = false;

  public static final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / 48;
  public static final double VELOCITY_CONVERSION_FACTOR = ((2 * Math.PI) / 48) / 60.0;

  public static final double MIN_POS = 0.0;
  public static final double MAX_POS = 12.75;

  public static final ElevatorFeedforwardGains FEEDFORWARD_GAINS = new ElevatorFeedforwardGains(0.0, 0.11, 0.0, 0.0);
  public static final PIDFGains PID_GAINS = new PIDFGains(1.5, 0.0, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI);
  public static final double POSITION_TOLERANCE = Units.degreesToRadians(10);
}