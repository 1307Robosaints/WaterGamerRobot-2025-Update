package frc.robot.subsystems.mailboxpivot;

import frc.robot.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLog;

public interface MailboxPivotIO {
  @AutoLog class MailboxPivotIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(MailboxPivotIOInputs inputs) {}

  default void setMailboxPivotPosition(double rad, double ff) {}

  default void setMailboxPivotPIDF(PIDFGains gains) {}
}