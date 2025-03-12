package frc.robot.subsystems.mailboxpivot;

public record MailboxPivotState(double positionRad) {
  public static final MailboxPivotState STARTING = new MailboxPivotState(0.0);
  public static final MailboxPivotState HUMAN_PLAYER = new MailboxPivotState(0.0);
  public static final MailboxPivotState L1 = new MailboxPivotState(0.0);
  public static final MailboxPivotState L2 = new MailboxPivotState(0.0);
  public static final MailboxPivotState L3 = new MailboxPivotState(0.0);
  public static final MailboxPivotState L4 = new MailboxPivotState(0.0);
}