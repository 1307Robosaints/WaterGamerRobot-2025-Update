package frc.robot.subsystems.mailbox;

import org.littletonrobotics.junction.AutoLog;

public interface MailboxIO {
  @AutoLog class MailboxIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(MailboxIOInputs inputs) {}

  default void setVoltage(double volts) {}
}