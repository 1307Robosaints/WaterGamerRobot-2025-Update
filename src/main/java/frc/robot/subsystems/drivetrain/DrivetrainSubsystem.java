package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.codebases.DriveCode;
import frc.robot.utils.SparkUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DrivetrainSubsystem extends SubsystemBase {
    private final double deadzone = .075; //Goes with driverobot method btw
    private SparkMax mot_leftfront = new SparkMax(DrivetrainConstants.FRONT_LEFT_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax mot_leftback = new SparkMax(DrivetrainConstants.BACK_LEFT_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax mot_rightfront = new SparkMax(DrivetrainConstants.FRONT_RIGHT_CAN_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax mot_rightback = new SparkMax(DrivetrainConstants.BACK_RIGHT_CAN_ID, SparkLowLevel.MotorType.kBrushless);

    private final MecanumDrive mecanumDrive;

    public DrivetrainSubsystem() {
        final SparkBaseConfig leftFrontConfig = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
        final SparkBaseConfig leftBackConfig = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(false);
        final SparkBaseConfig rightFrontConfig = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(true);
        final SparkBaseConfig rightBackConfig = new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake).inverted(true);

        SparkUtils.tryUntilOk(mot_leftback, 5, spark -> spark.configure(leftBackConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
        SparkUtils.tryUntilOk(mot_leftfront, 5, spark -> spark.configure(leftFrontConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
        SparkUtils.tryUntilOk(mot_rightback, 5, spark -> spark.configure(rightBackConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
        SparkUtils.tryUntilOk(mot_rightfront, 5, spark -> spark.configure(rightFrontConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));

        mecanumDrive = new MecanumDrive(mot_leftfront, mot_leftback, mot_rightfront, mot_rightback);
        addChild("Mecanum Drive", mecanumDrive);
        mecanumDrive.setSafetyEnabled(true);
        mecanumDrive.setExpiration(0.1);
        mecanumDrive.setMaxOutput(1.0);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void driverobot(double xAxis, double yAxis, double r_xAxis, double Throttle) {

        //Deadzones


        if (Math.abs(xAxis) < deadzone) {
            xAxis = 0;
        }

        if (Math.abs(yAxis) < deadzone) {
            yAxis = 0;
        }

        if (Math.abs(r_xAxis) < deadzone) {
            r_xAxis = 0;
        }


        //Speed/Throttle Maths [-1,1] --> [0,1]
        Throttle *= -1;
        Throttle += 1;
        Throttle /= 2;

        xAxis *= Throttle;
        yAxis *= -Throttle;
        r_xAxis *= Throttle;

        //or Setmax Output
        //setMaxOutput(double Throttle);
       
        mecanumDrive.driveCartesian(yAxis, xAxis, r_xAxis);
       
        //Debug
        //System.out.println("LF: " + leftfront + ", LB:" + leftback + ", RF:" + rightfront + ", RB:" + rightback);
        //System.out.println("X:" + xAxis + ", Y:" + yAxis + ", Z:" + r_xAxis + ", Thr:" + Throttle);

    }


    //Stop Method
    public void stop() {
        mecanumDrive.stopMotor();
    }

    // Expose the underlying MecanumDrive
    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }
}