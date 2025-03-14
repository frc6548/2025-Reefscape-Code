package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class IntakeCommandV2 extends Command {
    private Intake IntakeSubsystem;
    private LEDs ledSubsystem;
    private final PIDController pidController;
    
    public IntakeCommandV2(Intake IntakeSubsystem, LEDs ledSubsystem) {
        this.IntakeSubsystem = IntakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.pidController = new PIDController(.003, 0, 0);
        pidController.setSetpoint(10);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        IntakeSubsystem.resetIntakeEncoder();
        IntakeSubsystem.setIntakeMotor(0);
        pidController.reset();
        System.out.println("IntakeCommand started!");
    }

    @Override
    public void execute() {
        if (IntakeSubsystem.getPEStatus()) {
            IntakeSubsystem.resetIntakeEncoder();
            ledSubsystem.FlashLed();
            double speed = pidController.calculate(IntakeSubsystem.getIntakeEncoder());
            IntakeSubsystem.setIntakeMotor(speed);
        } else {
            IntakeSubsystem.setIntakeMotor(.3);
            ledSubsystem.SetLEDBuffer(255, 0, 0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.setIntakeMotor(0);
        ledSubsystem.SetLEDBuffer(0, 0, 0);
        // ledSubsystem.FlashLed();
        System.out.println("IntakeCommand ended!");
    }

    @Override
    public boolean isFinished() {
        // return IntakeSubsystem.getPEStatus();   
        return false;
    }
}