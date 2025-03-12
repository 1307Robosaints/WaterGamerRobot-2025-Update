package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.utils.PIDFGains;
import frc.robot.utils.TunablePIDF;

import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final TunablePIDF tunableDrivePIDF;

  private final Alert driveDisconnectedAlert;
  private double[] odometryPositions = new double[] {};

  private final String name;
  
  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
    tunableDrivePIDF = new TunablePIDF("Drive/" + name + "Module/DrivePID/", DrivetrainConstants.DRIVE_PIDF);

    driveDisconnectedAlert = new Alert("Disconnected " + name + "Module drive motor (" + name + ")", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name + "Module", inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new double[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DrivetrainConstants.WHEEL_RADIUS_METERS;
      odometryPositions[i] = positionMeters;
    }

    tunableDrivePIDF.periodic(io::setDrivePIDF, io::setDriveVelocity);

    driveDisconnectedAlert.set(!inputs.driveConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(double speedMetersPerSecond) {
    io.setDriveVelocity(speedMetersPerSecond / DrivetrainConstants.WHEEL_RADIUS_METERS);
  }

  public void setDrivePIDF(PIDFGains gains) {
    io.setDrivePIDF(gains);
    tunableDrivePIDF.setGains(gains);
  }

  public PIDFGains getDrivePIDF() {
    return tunableDrivePIDF.getGains();
  }

  public void runDriveCharacterization(double volts) {
    io.setTurnPosition(new Rotation2d());
    io.setDriveVolts(volts);
  }

  public void setTurnPosition(Rotation2d rotation) {
    io.setTurnPosition(rotation);
  }

  public void stop() {
    io.setDriveVolts(0.0);
    io.setTurnVolts(0.0);
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * DrivetrainConstants.WHEEL_RADIUS_METERS;
  }

  public double getVelocityRadPerSec() {
    return inputs.driveVelocityRadPerSec;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DrivetrainConstants.WHEEL_RADIUS_METERS;
  }

  public double[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}