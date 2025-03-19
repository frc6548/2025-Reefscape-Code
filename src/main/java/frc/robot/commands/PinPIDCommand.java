package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class PinPIDCommand extends Command {
    private Climber ClimberSubsystem;
    private final PIDController pidController;
   
    public PinPIDCommand(Climber ClimberSubsystem, double setpoint) {
        this.ClimberSubsystem = ClimberSubsystem;
        this.pidController = new PIDController(0.2, 0, 0);
        pidController.setSetpoint(setpoint);
}
@Override
    public void initialize() {
        ClimberSubsystem.setPinMotor(0);
        pidController.reset();
        System.out.println("PinPIDCommand started!");
    }
    @Override
    public void execute() {
        double speed = pidController.calculate(ClimberSubsystem.getPinEncoder());
        ClimberSubsystem.setPinMotor(speed);
    }
    @Override
    public void end(boolean interrupted) {
        ClimberSubsystem.setPinMotor(0);
        System.out.println("PinPIDCommand ended!");
    }
    @Override
    public boolean isFinished() {
        return false; 
    }
}
