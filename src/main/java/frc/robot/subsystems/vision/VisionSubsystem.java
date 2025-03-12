package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionConsumer consumer;
  // The transform from the robot's center to the Limelight's center, stored as a Transform3d.
  private final Transform3d robotToCamera;

  /**
   * @param consumer          The consumer that accepts processed vision poses.
   * @param io                The VisionIO instance for the Limelight.
   * @param robotToCameraPose The fixed mounting pose from the robot's center to the camera.
   */
  public VisionSubsystem(VisionConsumer consumer, VisionIO io, Pose3d robotToCameraPose) {
    this.consumer = consumer;
    this.io = io;
    // Convert the provided Pose3d into a Transform3d for use with transformBy().
    this.robotToCamera = new Transform3d(robotToCameraPose.getTranslation(), robotToCameraPose.getRotation());
  }

  @Override
  public void periodic() {
    // Update the Limelight inputs.
    io.updateInputs(inputs);
    Logger.processInputs("Vision/Limelight", inputs);

    final var robotPosesAccepted = new LinkedList<Pose3d>();

    // Process each vision pose observation.
    for (var observation : inputs.poseObservations) {
      // Transform the raw camera pose into a robot-centric pose by applying the mounting transform (without inversion)
      Pose3d rawPose = observation.pose();
      Pose3d robotPose = rawPose.transformBy(robotToCamera);

      // Apply rejection criteria to filter out bad observations.
      final boolean rejectPose = observation.tagCount() == 0
              || (observation.tagCount() == 1 && observation.ambiguity() > SINGLE_TAG_MAX_AMBIGUITY)
              || Math.abs(robotPose.getZ()) > MAX_Z_ERROR
              || robotPose.getX() < 0.0
              || robotPose.getX() > APRIL_TAG_FIELD.getFieldLength()
              || robotPose.getY() < 0.0
              || robotPose.getY() > APRIL_TAG_FIELD.getFieldWidth();

      if (rejectPose) continue;

      robotPosesAccepted.add(robotPose);

      // Calculate standard deviations based on observation quality.
      final double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      final double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
      final double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

      // Pass the robot-centric pose (converted to Pose2d) to the consumer.
      consumer.accept(robotPose.toPose2d(), observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    // Log accepted robot poses.
    Logger.recordOutput("Vision/Limelight/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<?, ?> visionMeasurementStdDevs);
  }
}
