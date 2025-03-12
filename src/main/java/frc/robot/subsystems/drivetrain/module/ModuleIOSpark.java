package frc.robot.subsystems.drivetrain.module;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.DrivetrainConfigs;
import frc.robot.subsystems.drivetrain.SparkOdometryThread;
import frc.robot.utils.PIDFGains;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.robot.utils.SparkUtils.*;

public class ModuleIOSpark implements ModuleIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase driveSpark;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePID;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;

  private final Debouncer driveConnectedDebouncer = new Debouncer(0.5);

  public ModuleIOSpark(int canId) {
    driveSpark = new SparkMax(canId, SparkLowLevel.MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    drivePID = driveSpark.getClosedLoopController();

    tryUntilOk(driveSpark, 5, spark -> {
      spark.configure(DrivetrainConfigs.DRIVE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    tryUntilOk(driveSpark, 5, spark -> {
      spark.getEncoder().setPosition(0.0);
    });

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
  }

  @Override public void updateInputs(ModuleIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(driveSpark,
            new DoubleSupplier[]{driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
            (values) -> inputs.driveAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.driveConnected = driveConnectedDebouncer.calculate(!HAS_STICKY_FAULT);

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    timestampQueue.clear();
    drivePositionQueue.clear();
  }

  @Override public void setDriveVolts(double volts) {
    driveSpark.setVoltage(volts);
  }

  @Override public void setDriveVelocity(double velocityRadPerSec) {
    drivePID.setReference(velocityRadPerSec, SparkBase.ControlType.kVelocity);
  }
  
  @Override public void setDrivePIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(driveSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}