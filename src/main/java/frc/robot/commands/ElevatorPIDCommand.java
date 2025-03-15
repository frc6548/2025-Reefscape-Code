package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCommand extends Command {
  private Elevator elevatorSubsystem;
  private double wantedSetpoint;//every "new elevatorPidCommand" thats created will be its own object with its own wanted setpoint (for each button and position you create)

  //this is ran once when command is created, in robotcontainer, bindingconfig only runs once at boot. 
  public ElevatorPIDCommand(Elevator elevatorSubsystem, double _wantedSetpoint) {
      wantedSetpoint =_wantedSetpoint;
      this.elevatorSubsystem = elevatorSubsystem;
      addRequirements(elevatorSubsystem);
    }

  //this is called every time the command is scheduled. 
  @Override
  public void initialize() {
    System.out.println("ElevatorPIDCommand to position " + wantedSetpoint + "Started!");
    elevatorSubsystem.targetSetpoint = wantedSetpoint;//the elevator subsystem controls the pid, its always running, this will just change the setpoint. 
  }

  @Override
  public void execute() {
    //periodic things are all handled by the main subsystem. this command just sets the setpoint and waits for arrival. 
  }

  @Override
  public boolean isFinished() {
      boolean isatPosition = MathUtil.isNear(wantedSetpoint, elevatorSubsystem.getElevatorEncoder1(), elevatorSubsystem.positionalTolerance);
      return isatPosition;//will return true when we are at setpoint. which will then run the END command. 
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted)
    {
      System.out.println("ElevatorPIDCommand to position " + wantedSetpoint + "ended because another command was scheduled or this one was cancelled!");
    }
    else
    {
      System.out.println("ElevatorPIDCommand to position " + wantedSetpoint + "ended because it arrived at the location!");
    }
    
  }
}