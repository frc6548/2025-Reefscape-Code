package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class OuttakeCommand extends Command {
    private Intake IntakeSubsystem;
    private LEDs ledSubsystem;
    private final PIDController pidController;
    
    public OuttakeCommand(Intake IntakeSubsystem, LEDs ledSubsystem, double setpoint) {
        this.IntakeSubsystem = IntakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.pidController = new PIDController(.2, 0, 0);
        pidController.setSetpoint(setpoint);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        IntakeSubsystem.resetIntakeEncoder();
        IntakeSubsystem.setIntakeMotor(0);
        pidController.reset();
        System.out.println("OuttakeCommand started!");
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(IntakeSubsystem.getIntakeEncoder());
        IntakeSubsystem.setIntakeMotor(speed);


    }
    
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.setIntakeMotor(0);
        ledSubsystem.SetLEDBuffer(0, 0, 0);
        System.out.println("OuttakeCommand ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}