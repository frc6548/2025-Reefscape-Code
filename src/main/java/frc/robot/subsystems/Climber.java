package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final SparkMax LeftClimberMotor = new SparkMax(16, MotorType.kBrushless); 
    private final SparkMax RightClimberMotor = new SparkMax(21, MotorType.kBrushless); 
    private final SparkMax PinMotor = new SparkMax(20, MotorType.kBrushless); 
    public final RelativeEncoder ClimberEncoder1 = LeftClimberMotor.getEncoder();
    public final RelativeEncoder ClimberEncoder2 = RightClimberMotor.getEncoder();
    public final RelativeEncoder PinEncoder = PinMotor.getEncoder();
    public double targetSetpoint;

    public Command setClimberSetpoint(double setpoint) {
      return Commands.runOnce(() -> targetSetpoint = setpoint);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Velocity1", ClimberEncoder1.getVelocity());
        SmartDashboard.putNumber("Climber Velocity2", ClimberEncoder2.getVelocity());
        SmartDashboard.putNumber("Pin Velocity", PinEncoder.getVelocity());
        SmartDashboard.putNumber("Climber Encoder1", ClimberEncoder1.getPosition());
        SmartDashboard.putNumber("Climber Encoder2", ClimberEncoder2.getPosition());   
        SmartDashboard.putNumber("Pin Encoder", PinEncoder.getPosition());
}
public void setClimberMotor(double speed) {
    LeftClimberMotor.set(speed);
    // RightClimberMotor.set(speed);
}

public void bringCageIn(double speed) {
    LeftClimberMotor.set(0.1*speed);
    RightClimberMotor.set(speed);
}

public void setPinMotor(double speed) {
    PinMotor.set(speed);
}

public double getClimberEncoder1() {
    return ClimberEncoder1.getPosition();
}

public double getPinEncoder() {
    return PinEncoder.getPosition();
}

public void resetClimberEncoder1() {
    ClimberEncoder1.setPosition(0);
}

public void resetPinEncoder() {
    PinEncoder.setPosition(0);
}
}