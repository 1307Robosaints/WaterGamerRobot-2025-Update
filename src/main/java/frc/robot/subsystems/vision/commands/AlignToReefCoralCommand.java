/* 
package frc.robot.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefScoringPosition;
import frc.robot.ReefScoringSelector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.controllers.FullAlignController;
import org.littletonrobotics.junction.Logger;
import java.util.Optional;

public class AlignToReefCoralCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToReefCommand",
          VisionConstants.ALIGNMENT_OMEGA_PID,
          VisionConstants.ALIGNMENT_X_VELOCITY_PID,
          VisionConstants.ALIGNMENT_Y_VELOCITY_PID,
          VisionConstants.ALIGNMENT_RAD_TOLERANCE,
          VisionConstants.ALIGNMENT_X_METERS_TOLERANCE,
          VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE,
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
              DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED),
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_REAL_SPEED_METERS_PER_SECOND,
              DrivetrainConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED),
          new TrapezoidProfile.Constraints(
              DrivetrainConstants.MAX_REAL_SPEED_METERS_PER_SECOND,
              DrivetrainConstants.MAX_REAL_ACCELERATION_METERS_PER_SECOND_SQUARED)
  );

  private final DrivetrainSubsystem mecanum;
  private final VisionSubsystem vision;
  private final ReefScoringSelector selector;
  private final ReefScoringPosition.ReefScoringSide side;
  private boolean tagFound = false;

  public AlignToReefCoralCommand(DrivetrainSubsystem mecanum, VisionSubsystem vision, ReefScoringSelector selector, ReefScoringPosition.ReefScoringSide side) {
    this.mecanum = mecanum;
    this.vision = vision;
    this.selector = selector;
    this.side = side;
    selector.setSide(side);
    addRequirements(mecanum, vision);
  }

  @Override 
  public void initialize() {
    alignController.resetPIDErrors(mecanum.getRotation().getRadians(), mecanum.getPose().getTranslation());
    tagFound = false;
  }

  @Override 
  public void execute() {
    // Retrieve the best target observation from the vision subsystem.
    Optional<VisionSubsystem.TargetObservation> bestTargetOpt = vision.getBestTargetObservation();
    if (bestTargetOpt.isEmpty() || !bestTargetOpt.get().hasObservation()) {
      mecanum.stop();
      return;
    }
    VisionSubsystem.TargetObservation bestTarget = bestTargetOpt.get();

    ReefScoringPosition.getCoralPositionFor(bestTarget.tagId(), side, selector.getLevel()).ifPresentOrElse(position -> {
      tagFound = true;
      final var goalPose = new Pose2d(
              position.position().toTranslation2d(),
              position.robotHeading()
      );
      final var speeds = alignController.calculate(
              mecanum.getPose(),
              goalPose,
              String.valueOf(bestTarget.tagId())
      );

      Logger.recordOutput("AlignToReefCoralCommand/GoalPose", goalPose);
      Logger.recordOutput("AlignToReefCoralCommand/Vx", speeds.vxMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralCommand/Vy", speeds.vyMetersPerSecond);
      Logger.recordOutput("AlignToReefCoralCommand/Omega", speeds.omegaRadiansPerSecond);

      mecanum.runVelocityFieldRelative(speeds);
    }, mecanum::stop);
  }

  @Override 
  public void end(boolean interrupted) {
    mecanum.stop();
  }

  @Override 
  public boolean isFinished() {
    return tagFound && alignController.atGoal();
  }
}
*/