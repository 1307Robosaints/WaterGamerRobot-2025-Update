package frc.robot.subsystems.mailbox;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;

import java.util.function.DoubleSupplier;

import static frc.robot.utils.SparkUtils.*;

public class MailboxIOSpark implements MailboxIO {
  private static boolean HAS_STICKY_FAULT = false;

  private final SparkBase spark;
  private final RelativeEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public MailboxIOSpark(int sparkCanId) {
    this.spark = new SparkMax(sparkCanId, SparkLowLevel.MotorType.kBrushless);
    this.encoder = spark.getEncoder();

    tryUntilOk(spark, 5, spark -> {
      spark.configure(MailboxConstants.CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    });
  }

  @Override public void updateInputs(MailboxIOInputs inputs) {
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

  @Override public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }
}