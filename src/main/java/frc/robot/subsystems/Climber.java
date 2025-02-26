package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final SparkMax LeftClimberMotor = new SparkMax(16, MotorType.kBrushless); 
    private final SparkMax RightClimberMotor = new SparkMax(17, MotorType.kBrushless); 
    public final RelativeEncoder ClimberEncoder1 = LeftClimberMotor.getEncoder();
    public final RelativeEncoder ClimberEncoder2 = RightClimberMotor.getEncoder();

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Climber Velocity1", ClimberEncoder1.getVelocity());
        SmartDashboard.putNumber("Climber Velocity2", ClimberEncoder2.getVelocity());
        SmartDashboard.putNumber("Climber Encoder1", ClimberEncoder1.getPosition());
        SmartDashboard.putNumber("Climber Encoder2", ClimberEncoder2.getPosition());   
}
public void setClimberMotor(double speed) {
    LeftClimberMotor.set(speed);
    RightClimberMotor.set(speed);
}
public double getClimberEncoder1() {
    return ClimberEncoder1.getPosition();
}
public void resetClimberEncoder1() {
    ClimberEncoder1.setPosition(0);
}
}