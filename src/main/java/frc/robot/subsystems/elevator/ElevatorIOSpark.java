package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.utils.PIDFGains;
import frc.robot.utils.SparkUtils;

import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase leftSpark;
  private final RelativeEncoder leftEncoder;

  private final Debouncer leftConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOSpark(int leftCanId) {
    this.leftSpark = new SparkMax(leftCanId, SparkLowLevel.MotorType.kBrushless);
    this.leftEncoder = leftSpark.getEncoder();

    SparkUtils.tryUntilOk(leftSpark, 5, spark -> {
      spark.configure(ElevatorConfigs.LEFT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });

    resetPosition();
  }

  @Override public void updateInputs(ElevatorIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    SparkUtils.useValueIfOk(leftSpark, leftEncoder::getPosition, (value) -> inputs.leftPositionRad = value, () -> HAS_STICKY_FAULT = true);
    SparkUtils.useValueIfOk(leftSpark, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    SparkUtils.useValuesIfOk(leftSpark,
            new DoubleSupplier[]{leftSpark::getAppliedOutput, leftSpark::getBusVoltage},
            (values) -> inputs.leftAppliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    SparkUtils.useValueIfOk(leftSpark, leftSpark::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.leftConnected = leftConnectedDebouncer.calculate(!HAS_STICKY_FAULT);
  }

  @Override public void setPosition(double rad, double ff) {
    SparkUtils.tryUntilOk(leftSpark, 5, spark -> {
      spark.getClosedLoopController().setReference(
              MathUtil.clamp(rad, ElevatorConstants.MIN_POS, ElevatorConstants.MAX_POS),
              SparkBase.ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              ff,
              SparkClosedLoopController.ArbFFUnits.kVoltage
      );
    });
  }

  @Override public void resetPosition() {
    SparkUtils.tryUntilOk(leftSpark, 5, spark -> {
      spark.getEncoder().setPosition(0);
    });

  }

  @Override public void setPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    SparkUtils.tryUntilOk(leftSpark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}