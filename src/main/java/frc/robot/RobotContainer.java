package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.drivetrain.DrivetrainConfigs;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.GyroIOAnalog;
import frc.robot.subsystems.drivetrain.module.ModuleIOSpark;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mailbox.MailboxConstants;
import frc.robot.subsystems.mailboxpivot.MailboxPivotConstants;
import frc.robot.subsystems.mailboxpivot.MailboxPivotIOSpark;
import frc.robot.subsystems.mailboxpivot.MailboxPivotSubsystem;
import frc.robot.subsystems.commands.com_PS4robotdrive;
//import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.LimelightVision;

public class RobotContainer {
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(new ElevatorIOSpark(ElevatorConstants.LEFT_SPARK_ID));
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(
       /* new GyroIOAnalog(),
        new ModuleIOSpark(DrivetrainConstants.FRONT_LEFT_CAN_ID),
        new ModuleIOSpark(DrivetrainConstants.FRONT_RIGHT_CAN_ID),
        new ModuleIOSpark(DrivetrainConstants.BACK_LEFT_CAN_ID),
        new ModuleIOSpark(DrivetrainConstants.BACK_RIGHT_CAN_ID) */
    );
    private final MailboxPivotSubsystem mailboxPivot = new MailboxPivotSubsystem(new MailboxPivotIOSpark(MailboxPivotConstants.MAILBOX_ROT_CAN_ID));
    // Instantiate the drivetrain subsystem
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    // Instantiate the Limelight vision subsystem using drivetrain's drive
    private final LimelightVision limelightVision = new LimelightVision(drivetrain.getMecanumDrive());

    // The robot's subsystems
    // public final Drivetrain m_drivetrain = new Drivetrain();
    // public final Elevator m_elevator = new Elevator();
    // public final Mailbox m_mailbox = new Mailbox();
    // public final Limelight m_limelight = new Limelight();
    // public final Gyro m_gyro = new Gyro();

    // Use a PS4Controller (port 1)
    private final CommandPS4Controller controller = new CommandPS4Controller(0);
    
    // Autonomous chooser
    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        drivetrainSubsystem.setDefaultCommand(new com_PS4robotdrive(
            () -> -controller.getLeftY(), 
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            drivetrainSubsystem)
        );
        // SmartDashboard Subsystems
        // SmartDashboard.putData(m_elevator);
        // SmartDashboard.putData(m_drivetrain);
        // Optionally add mailbox if desired:
        // SmartDashboard.putData(m_mailbox);

        // Autonomous
        // SmartDashboard.putData("com_autonomous", new com_autonomous());

        //Drive commands for PS4 Remote
         final com_PS4robotdrive com_PS4robotdrive = new com_PS4robotdrive(
                 () -> controller.getLeftX(),
                 () -> controller.getLeftY(),
                 () -> controller.getRightX(),
                 drivetrainSubsystem);
         SmartDashboard.putData("com_robotdrive: PS4Remote", com_PS4robotdrive);


        // Elevator commands
        // SmartDashboard.putData("com_elevatorRightUp", new com_elevatorRightUp(m_elevator));
        // SmartDashboard.putData("com_elevatorRightMid", new com_elevatorRightMid(m_elevator));
        // SmartDashboard.putData("com_elevatorRightDown", new com_elevatorRightDown(m_elevator));
        // SmartDashboard.putData("com_elevatorLeftUp", new com_elevatorLeftUp(m_elevator));
        // SmartDashboard.putData("com_elevatorLeftMid", new com_elevatorLeftMid(m_elevator));
        // SmartDashboard.putData("com_elevatorLeftDown", new com_elevatorLeftDown(m_elevator));
        // SmartDashboard.putData("com_elevatorUp", new com_elevatorUp(m_elevator));
        // SmartDashboard.putData("com_elevatorDown", new com_elevatorDown(m_elevator));

        // // Mailbox commands
        // SmartDashboard.putData("com_mailboxEject", new com_mailboxup(m_mailbox));
        // SmartDashboard.putData("com_mailboxDown", new com_mailboxdown(m_mailbox));

        configureButtonBindings();
        // (Optionally) set a default command that enables vision tracking mode:
        // For example, when no other command is active, run vision tracking.
        drivetrain.setDefaultCommand(Commands.run(() -> {
            limelightVision.trackTarget();
        }, drivetrain));

        // Default drive
        // m_drivetrain.setDefaultCommand(com_PS4robotdrive);

        // Autonomous chooser
        SmartDashboard.putData("Auto Mode", m_chooser);
    }

    private void configureButtonBindings() {
        // // PS4 Controller button bindings we gotta update numbers
        // final JoystickButton but_setElevator = new JoystickButton(pS4Controller, 10);
        // but_setElevator.whileTrue(new com_setElevator(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_setElevator", new com_setElevator(m_elevator));

        // final JoystickButton but_elevatorUp = new JoystickButton(pS4Controller, 4);
        // but_elevatorUp.whileTrue(new com_elevatorUp(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorUp", new com_elevatorUp(m_elevator));

        // final JoystickButton but_elevatorDown = new JoystickButton(pS4Controller, 2);
        // but_elevatorDown.whileTrue(new com_elevatorDown(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorDown", new com_elevatorDown(m_elevator));

        // final JoystickButton but_elevatorRightUp = new JoystickButton(pS4Controller, 6);
        // but_elevatorRightUp.whileTrue(new com_elevatorRightUp(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorRightUp", new com_elevatorRightUp(m_elevator));

        // final POVButton but_elevatorRightMid = new POVButton(pS4Controller, 90, 0);
        // but_elevatorRightMid.whileTrue(new com_elevatorRightMid(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorRightMid", new com_elevatorRightMid(m_elevator));

        // final JoystickButton but_elevatorRightDown = new JoystickButton(pS4Controller, 5);
        // but_elevatorRightDown.whileTrue(new com_elevatorRightDown(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorRightDown", new com_elevatorRightDown(m_elevator));

        // final JoystickButton but_elevatorLeftUp = new JoystickButton(pS4Controller, 11);
        // but_elevatorLeftUp.whileTrue(new com_elevatorLeftUp(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorLeftUp", new com_elevatorLeftUp(m_elevator));

        // final POVButton but_elevatorLeftMid = new POVButton(pS4Controller, 270, 0);
        // but_elevatorLeftMid.whileTrue(new com_elevatorLeftMid(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorLeftMid", new com_elevatorLeftMid(m_elevator));

        // final JoystickButton but_elevatorLeftDown = new JoystickButton(pS4Controller, 12);
        // but_elevatorLeftDown.whileTrue(new com_elevatorLeftDown(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_elevatorLeftDown", new com_elevatorLeftDown(m_elevator));

        // final JoystickButton but_mailboxUp = new JoystickButton(pS4Controller, 8);
        // but_mailboxUp.whileTrue(new com_mailboxup(m_mailbox).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_mailboxUp", new com_mailboxup(m_mailbox));

        // final JoystickButton but_mailboxDown = new JoystickButton(pS4Controller, 7);
        // but_mailboxDown.whileTrue(new com_mailboxdown(m_mailbox).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // SmartDashboard.putData("but_mailboxDown", new com_mailboxdown(m_mailbox));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public static boolean isTuningMode() {
        return true;
    }
}
