package frc.robot.subsystems.mailbox;

public record MailboxState(double volts) {
  public static final MailboxState STOPPED = new MailboxState(0.0);
  public static final MailboxState INTAKE = new MailboxState( 0.0);
  public static final MailboxState OUTTAKE = new MailboxState( 0.0);
}