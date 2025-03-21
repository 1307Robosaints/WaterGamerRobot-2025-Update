package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.PIDFGains;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[]{};
    public double[] odometryDrivePositionsRad = new double[]{};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveVolts(double volts) {}

  default void setTurnVolts(double volts) {}

  default void setDriveVelocity(double velocityRadPerSec) {}

  default void setTurnPosition(Rotation2d rotation) {}

  default void setDrivePIDF(PIDFGains gains) {}

  default void setTurnPIDF(PIDFGains gains) {}
}