package frc.robot.subsystems.mailbox;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MailboxSubsystem extends SubsystemBase {
  private final MailboxIO mailboxIO;
  private final MailboxIOInputsAutoLogged mailboxInputs = new MailboxIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Disconnected mailbox motor (" + MailboxConstants.MAILBOX_CAN_ID + ")", Alert.AlertType.kError);

  private MailboxState desiredState = MailboxState.STOPPED;

  public MailboxSubsystem(MailboxIO mailboxIO) {
    this.mailboxIO = mailboxIO;
  }

  @Override public void periodic() {
    mailboxIO.updateInputs(mailboxInputs);
    Logger.processInputs("Mailbox", mailboxInputs);

    disconnectedAlert.set(!mailboxInputs.connected);
  }

  public void setDesiredState(MailboxState desiredState) {
    mailboxIO.setVoltage(desiredState.volts());
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "Mailbox/State")
  public MailboxState getState() {
    return new MailboxState(mailboxInputs.appliedVolts);
  }

  @AutoLogOutput(key = "Mailbox/DesiredState")
  public MailboxState getDesiredState() {
    return this.desiredState;
  }
}