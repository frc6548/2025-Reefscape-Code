package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class OuttakeCommandV2 extends Command {
    private Intake IntakeSubsystem;
    private LEDs ledSubsystem;
    private final PIDController pidController;
    
    public OuttakeCommandV2(Intake IntakeSubsystem, LEDs ledSubsystem, double setpoint) {
        this.IntakeSubsystem = IntakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.pidController = new PIDController(.0475, 0, 0);
        pidController.setSetpoint(setpoint);
        addRequirements(ledSubsystem);
    }

         public boolean isAtPosition() {
        double error = IntakeSubsystem.getIntakeEncoder() - pidController.getSetpoint();
        return (Math.abs(error) < .4);
      }

    @Override
    public void initialize() {
        IntakeSubsystem.resetIntakeEncoder();
        IntakeSubsystem.setIntakeMotor(0);
        pidController.reset();
        System.out.println("OuttakeCommandV2 started!");
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
        System.out.println("OuttakeCommandV2 ended!");
    }

    @Override
    public boolean isFinished() {
        return isAtPosition(); 
    }
}