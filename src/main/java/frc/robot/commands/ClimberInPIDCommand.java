package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberIn;

public class ClimberInPIDCommand extends Command {
    private ClimberIn climberSubsystem;
    private final PIDController pidController;
    private final ArmFeedforward feedforward;
    private static double kP = 0.07;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kG = 0.01; 
    private static double kV = 0.0;

    public ClimberInPIDCommand(ClimberIn climberSubsystem, double setpoint) {      
      this.climberSubsystem = climberSubsystem;
      this.pidController = new PIDController(kP, kI, kD);
      this.feedforward = new ArmFeedforward(kS, kG, kV);
      pidController.setSetpoint(setpoint);
    }

  @Override
  public void initialize() {
    climberSubsystem.setClimberMotor(0);
    pidController.reset();
    System.out.println("ClimberPIDCommand started!");
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(climberSubsystem.getClimberEncoder1()) +
    feedforward.calculate(0, pidController.getSetpoint());
    climberSubsystem.setClimberMotor(speed);
  }

  public boolean isAtPosition() {
    double error = climberSubsystem.getClimberEncoder1() - pidController.getSetpoint();
    return (Math.abs(error) < .4);
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setClimberMotor(0);
    System.out.println("ClimberPIDCommand ended!");
  }

    @Override
    public boolean isFinished() {
        return isAtPosition(); 
    }


}