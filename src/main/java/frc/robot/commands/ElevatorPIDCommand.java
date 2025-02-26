package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCommand extends Command {
    private Elevator elevatorSubsystem;
    private final PIDController pidController;
    private final ElevatorFeedforward feedforward;
    private static double kP = 0.068; //.068
    private static double kI = 0.0;
    private static double kD = 0.00; //.005
    private static double kS = 0.0;
    private static double kG = 0.075; //.07
    private static double kV = 0.0;
    private static double kA = 0.0;
    private double tolerance = 1;

    public ElevatorPIDCommand(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(kP, kI, kD);
        this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        // pidController.setSetpoint(setpoint);
        addRequirements(elevatorSubsystem);
        
      }

  @Override
  public void initialize() {
    System.out.println("ElevatorPIDCommand started!");
    elevatorSubsystem.setElevatorMotor(tolerance);
  }

  @Override
  public void execute() {
    if (pidController.getSetpoint() != elevatorSubsystem.targetSetpoint)
      pidController.setSetpoint(elevatorSubsystem.targetSetpoint);

    double speed = pidController.calculate(elevatorSubsystem.getElevatorEncoder1()) + 
      feedforward.calculate(pidController.getSetpoint());
    
    elevatorSubsystem.setElevatorMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorMotor(0);
    System.out.println("ElevatorPIDCommand ended!");
  }


}
