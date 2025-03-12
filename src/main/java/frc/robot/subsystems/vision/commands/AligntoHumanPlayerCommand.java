/*package frc.robot.subsystems.vision.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.HumanPlayerStationPosition;
import frc.robot.HumanPlayerStationPosition.HumanPlayerStationSide;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.controllers.FullAlignController;
import org.littletonrobotics.junction.Logger;
import java.util.Optional;

public class AlignToHumanPlayerCommand extends Command {
  private final FullAlignController alignController = new FullAlignController(
          "AlignToHumanPlayerCommand",
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

  private final DrivetrainSubsystem drivetrain;
  private final VisionSubsystem vision;
  private final HumanPlayerStationSide side;
  private boolean tagFound = false;

  public AlignToHumanPlayerCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, HumanPlayerStationSide side) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.side = side;
    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    alignController.resetPIDErrors(drivetrain.getRotation().getRadians(), drivetrain.getPose().getTranslation());
    tagFound = false;
  }

  @Override
  public void execute() {
    // Retrieve the best target observation from the vision subsystem.
    Optional<VisionSubsystem.TargetObservation> bestTargetOpt = vision.getBestTargetObservation();
    if (bestTargetOpt.isEmpty() || !bestTargetOpt.get().hasObservation()) {
      drivetrain.stop();
      return;
    }
    VisionSubsystem.TargetObservation bestTarget = bestTargetOpt.get();

    // Get the human player station position for the detected tag and desired side.
    HumanPlayerStationPosition.getPositionFor(bestTarget.tagId(), side).ifPresentOrElse(position -> {
      tagFound = true;
      // TODO: Adjust the xOffset (0.5 in this example) as needed.
      final var offsetTranslationGoal = new Translation2d(0.5, position.yOffsetMeters());
      final var speeds = alignController.calculate(
              drivetrain.getRotation().getRadians(),
              position.robotHeadingRad(),
              bestTarget.translation().toTranslation2d(),
              offsetTranslationGoal,
              String.valueOf(bestTarget.tagId())
      );
      drivetrain.runVelocity(speeds, false);
    }, drivetrain::stop);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return tagFound && alignController.atGoal();
  }
}
*/