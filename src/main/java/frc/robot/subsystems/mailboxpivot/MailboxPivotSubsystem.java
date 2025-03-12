package frc.robot.subsystems.mailboxpivot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.utils.TunableArmGains;
import frc.robot.utils.TunableElevatorGains;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MailboxPivotSubsystem extends SubsystemBase {
  private final MailboxPivotIO mailboxPivotIO;
  private final MailboxPivotIOInputsAutoLogged mailboxPivotInputs = new MailboxPivotIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Disconnected mailbox pivot motor (" + MailboxPivotConstants.MAILBOX_ROT_CAN_ID + ")", Alert.AlertType.kError);

  private final TunableArmGains tunableGains = new TunableArmGains(
          "MailboxPivot/Gains/",
          MailboxPivotConstants.GAINS,
          MailboxPivotConstants.FEEDFORWARD_GAINS,
          MailboxPivotConstants.CONSTRAINTS,
          MailboxPivotConstants.MIN_POS,
          MailboxPivotConstants.MAX_POS
  );
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
          MailboxPivotConstants.FEEDFORWARD_GAINS.kS(),
          MailboxPivotConstants.FEEDFORWARD_GAINS.kG(),
          MailboxPivotConstants.FEEDFORWARD_GAINS.kV(),
          MailboxPivotConstants.FEEDFORWARD_GAINS.kA()
  );
  private TrapezoidProfile profile = new TrapezoidProfile(MailboxPivotConstants.CONSTRAINTS);

  private MailboxPivotState desiredState = MailboxPivotState.STARTING;

  public MailboxPivotSubsystem(MailboxPivotIO mailboxPivotIO) {
    this.mailboxPivotIO = mailboxPivotIO;
  }

  @Override public void periodic() {
    mailboxPivotIO.updateInputs(mailboxPivotInputs);
    Logger.processInputs("MailboxPivot", mailboxPivotInputs);

    tunableGains.periodic(mailboxPivotIO::setMailboxPivotPIDF, ffConstants -> {
      feedforward.setKs(ffConstants.kS());
      feedforward.setKg(ffConstants.kG());
      feedforward.setKv(ffConstants.kV());
      feedforward.setKa(ffConstants.kA());
    }, constraints -> {
      this.profile = new TrapezoidProfile(constraints);
    }, positionRad -> {
      setDesiredState(new MailboxPivotState(positionRad));
    });

    setDesiredState(desiredState);

    disconnectedAlert.set(!mailboxPivotInputs.connected);
  }

  public void setDesiredState(MailboxPivotState desiredState) {
    final var currentState = new TrapezoidProfile.State(mailboxPivotInputs.positionRad, mailboxPivotInputs.velocityRadPerSec);
    final var goalState = new TrapezoidProfile.State(desiredState.positionRad(), 0.0);
    final var nextState = profile.calculate(0.02, currentState, goalState);
    mailboxPivotIO.setMailboxPivotPosition(desiredState.positionRad(), feedforward.calculateWithVelocities(currentState.velocity, nextState.velocity));
    this.desiredState = desiredState;
  }

  @AutoLogOutput(key = "MailboxPivot/State")
  public MailboxPivotState getState() {
    return new MailboxPivotState(mailboxPivotInputs.positionRad);
  }

  @AutoLogOutput(key = "MailboxPivot/DesiredState")
  public MailboxPivotState getDesiredState() {
    return this.desiredState;
  }
}
