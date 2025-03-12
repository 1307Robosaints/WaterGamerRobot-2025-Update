package frc.robot.subsystems.elevator;

import frc.robot.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog class ElevatorIOInputs {
    public boolean leftConnected = false;
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setPosition(double rad, double ff) {}

  default void resetPosition() {}

  default void setPIDF(PIDFGains gains) {}
}