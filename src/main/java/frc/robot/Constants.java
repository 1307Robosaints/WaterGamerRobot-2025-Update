package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final double ODOMETRY_FREQUENCY = 100.0; // Hz
  public static final int NEO_CURRENT_LIMIT = 50;
  public static final int NEO_550_CURRENT_LIMIT = 20;
  public static final String LOG_FOLDER_PATH = "/U/logs/";
  public static final boolean TUNING_MODE = true;
  public static final String TUNING_TABLE_NAME = "Tuning";
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static final int LEFT_FRONT_CAN_ID = 4;
  public static final int LEFT_BACK_CAN_ID = 3;
  public static final int RIGHT_FRONT_CAN_ID = 1;
  public static final int RIGHT_BACK_CAN_ID = 2;


  public enum Mode {
    /**
     * Running on a real robot.
     */
    REAL,

    /**
     * Running a physics simulator.
     */
    SIM,

    /**
     * Replaying from a log file.
     */
    REPLAY
  }
}