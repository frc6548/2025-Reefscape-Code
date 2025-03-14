package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberPIDCommand extends Command {
    private Climber climberSubsystem;
    private final PIDController pidController;
    private final ArmFeedforward feedforward;
    private static double kP = 0.03;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kG = 0.0; 
    private static double kV = 0.0;
    private double tolerance = .5;

    public ClimberPIDCommand(Climber climberSubsystem) {      
      this.climberSubsystem = climberSubsystem;
      this.pidController = new PIDController(kP, kI, kD);
      this.feedforward = new ArmFeedforward(kS, kG, kV);
      addRequirements(climberSubsystem);
    }

  @Override
  public void initialize() {
    System.out.println("ClimberPIDCommand started!");
    climberSubsystem.setClimberMotor(tolerance);
  }

  @Override
  public void execute() {
    if (pidController.getSetpoint() != climberSubsystem.targetSetpoint)
    pidController.setSetpoint(climberSubsystem.targetSetpoint);

    double speed = pidController.calculate(climberSubsystem.getClimberEncoder1()) +
    feedforward.calculate(0, pidController.getSetpoint());
    climberSubsystem.setClimberMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setClimberMotor(0);
    System.out.println("ClimberPIDCommand ended!");
  }
}
