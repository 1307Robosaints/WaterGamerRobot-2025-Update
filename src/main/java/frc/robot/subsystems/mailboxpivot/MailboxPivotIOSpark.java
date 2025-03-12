package frc.robot.subsystems.mailboxpivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utils.PIDFGains;
import frc.robot.utils.SparkUtils;

import java.util.function.DoubleSupplier;

import static frc.robot.utils.SparkUtils.*;

public class MailboxPivotIOSpark implements MailboxPivotIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase spark;
  private final AbsoluteEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public MailboxPivotIOSpark(int canId) {
    this.spark = new SparkMax(canId, SparkLowLevel.MotorType.kBrushless);
    this.encoder = spark.getAbsoluteEncoder();

    tryUntilOk(spark, 5, spark -> {
      spark.configure(MailboxPivotConstants.CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(MailboxPivotIOInputs inputs) {
    HAS_STICKY_FAULT = false;
    useValueIfOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value, () -> HAS_STICKY_FAULT = true);
    useValueIfOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value, () -> HAS_STICKY_FAULT = true);
    useValuesIfOk(spark,
            new DoubleSupplier[]{spark::getAppliedOutput, spark::getBusVoltage},
            (values) -> inputs.appliedVolts = values[0] * values[1],
            () -> HAS_STICKY_FAULT = true);
    useValueIfOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value, () -> HAS_STICKY_FAULT = true);
    inputs.connected = connectedDebouncer.calculate(!HAS_STICKY_FAULT);
  }

  @Override public void setMailboxPivotPosition(double rad, double ff) {
    SparkUtils.tryUntilOk(spark, 5, spark -> {
      spark.getClosedLoopController().setReference(
              MathUtil.clamp(rad, MailboxPivotConstants.MIN_POS, MailboxPivotConstants.MAX_POS),
              SparkBase.ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              ff,
              SparkClosedLoopController.ArbFFUnits.kVoltage
      );
    });
  }

  @Override public void setMailboxPivotPIDF(PIDFGains gains) {
    final var config = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(gains.p(), gains.i(), gains.d(), gains.ff()));
    tryUntilOk(spark, 5, spark -> {
      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    });
  }
}
