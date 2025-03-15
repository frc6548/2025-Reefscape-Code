package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePivotPIDCommand extends Command {
    private Intake IntakeSubsystem;
    private double wantedSetpoint; //every "new IntakePidCommand" thats created will be its own object with its own wanted setpoint (for each button and position you create)

    public IntakePivotPIDCommand(Intake IntakeSubsystem, double _wantedSetpoint) {  
      wantedSetpoint =_wantedSetpoint;    
      this.IntakeSubsystem = IntakeSubsystem;
      addRequirements(IntakeSubsystem);
    }
    
  //this is called every time the command is scheduled. 
  @Override
  public void initialize() {
    System.out.println("IntakePIDCommand to position " + wantedSetpoint + "Started!");
    IntakeSubsystem.targetSetpoint = wantedSetpoint; //the intake subsystem controls the pid, its always running, this will just change the setpoint. 
  }

  @Override
  public void execute() {
    //periodic things are all handled by the main subsystem. this command just sets the setpoint and waits for arrival. 
  }

  @Override
  public boolean isFinished() {
      boolean isatPosition = MathUtil.isNear(wantedSetpoint, IntakeSubsystem.getPivotEncoder(), IntakeSubsystem.positionalTolerance);
      return isatPosition; //will return true when we are at setpoint. which will then run the END command. 
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted)
    {
      System.out.println("IntakePIDCommand to position " + wantedSetpoint + "ended because another command was scheduled or this one was cancelled!");
    }
    else
    {
      System.out.println("IntakePIDCommand to position " + wantedSetpoint + "ended because it arrived at the location!");
    }
  }
}
