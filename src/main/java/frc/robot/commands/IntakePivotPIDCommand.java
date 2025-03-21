package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePivotPIDCommand extends Command {
    private Intake intakeSubsystem;
    private double wantedSetpoint; //every "new IntakePidCommand" thats created will be its own object with its own wanted setpoint (for each button and position you create)

  //this is ran once when command is created, in robotcontainer, bindingconfig only runs once at boot. 
    public IntakePivotPIDCommand(Intake intakeSubsystem, double _wantedSetpoint) {  
      wantedSetpoint =_wantedSetpoint;    
      this.intakeSubsystem = intakeSubsystem;
      // addRequirements(intakeSubsystem);
    }
    
  //this is called every time the command is scheduled. 
  @Override
  public void initialize() {
    System.out.println("IntakePIDCommand to position " + wantedSetpoint + "Started!");
    intakeSubsystem.targetSetpoint = wantedSetpoint; //the intake subsystem controls the pid, its always running, this will just change the setpoint. 
  }

  @Override
  public void execute() {
    //periodic things are all handled by the main subsystem. this command just sets the setpoint and waits for arrival. 
  }

  @Override
  public boolean isFinished() {
      return true; // END INSTANTLY because this command doesn't need to be running after we set the desired position
      // boolean isatPosition = MathUtil.isNear(wantedSetpoint, intakeSubsystem.getPivotEncoder(), intakeSubsystem.
      // positionalTolerance);
      // return isatPosition; //will return true when we are at setpoint. which will then run the END command. 
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