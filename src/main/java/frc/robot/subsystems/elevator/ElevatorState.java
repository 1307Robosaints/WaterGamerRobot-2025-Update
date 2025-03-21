package frc.robot.subsystems.elevator;

public record ElevatorState(double positionRad) {
  public static final ElevatorState STARTING = new ElevatorState(0.0);
  public static final ElevatorState L1 = new ElevatorState(0.0);
  public static final ElevatorState L2 = new ElevatorState(0.0);
  public static final ElevatorState L3 = new ElevatorState(42.0);
  public static final ElevatorState L4 = new ElevatorState(97.5);
  public static final ElevatorState HUMAN_PLAYER = new ElevatorState(0.0);
}