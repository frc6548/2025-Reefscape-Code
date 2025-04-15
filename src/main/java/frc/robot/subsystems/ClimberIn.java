package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIn extends SubsystemBase{
    private final SparkMax RightClimberMotor = new SparkMax(21, MotorType.kBrushless); 
    public final RelativeEncoder ClimberEncoder2 = RightClimberMotor.getEncoder();
    public double targetSetpoint;

    public Command setClimberSetpoint(double setpoint) {
      return Commands.runOnce(() -> targetSetpoint = setpoint);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Velocity2", ClimberEncoder2.getVelocity());
        SmartDashboard.putNumber("Climber Speed", RightClimberMotor.getAppliedOutput());

        SmartDashboard.putNumber("Climber Encoder2", ClimberEncoder2.getPosition());   
}
public void setClimberMotor(double speed) {
    RightClimberMotor.set(speed);
}

public double getClimberEncoder1() {
    return ClimberEncoder2.getPosition();
}

public void resetClimberEncoder1() {
    ClimberEncoder2.setPosition(0);
}
}