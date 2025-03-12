package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class LimelightVision extends SubsystemBase {
    // Mecanum drive instance from drivetrain
    private final MecanumDrive mecanumDrive;
    // Reference to the Limelight NetworkTable
    private final NetworkTable limelightTable;
    // PID controller for horizontal alignment using the Limelight's tx value
    private final PIDController pidController;

    // Constructor now accepts the mecanum drive from the drivetrain
    public LimelightVision(MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        // Get the Limelight table from the default NetworkTable instance
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // Initialize a PID controller with your tuned constants.
        // Here using a proportional constant 0.1 and zero for integral and derivative.
        pidController = new PIDController(0.1, 0.0, 0.0);
        pidController.setTolerance(1.0);
    }

    /**
     * Tracks a vision target using the Limelight.
     * If a valid target is found, the PID controller uses the horizontal offset (tx)
     * to compute a rotational command that is then sent to the mecanum drive.
     */
    public void trackTarget() {
        // Get whether the Limelight has any valid targets (tv = 0 means no target)
        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        if (tv < 1.0) {
            // No target detected; stop the drive output.
            mecanumDrive.stopMotor();
            return;
        }

        // Get the horizontal offset (tx)
        double tx = limelightTable.getEntry("tx").getDouble(0.0);
        // Compute a rotation correction using the PID controller
        double rotation = pidController.calculate(tx, 0.0);
        // Clamp the rotation to a safe range (-0.5 to 0.5)
        rotation = MathUtil.clamp(rotation, -0.5, 0.5);
        // Drive the robot (only rotate, no forward/strafe motion)
        mecanumDrive.driveCartesian(0.0, 0.0, rotation);
    }

    @Override
    public void periodic() {
        // Update tracking every robot loop iteration
        trackTarget();
    }
}
