package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//This is the sybsystem that houses our Elevator Motor
public class Elevator extends SubsystemBase{
    private final SparkMax FrontElevatorMotor = new SparkMax(14, MotorType.kBrushless); 
    private final SparkMax BackElevatorMotor = new SparkMax(15, MotorType.kBrushless); 
    public final RelativeEncoder ElevatorEncoder1 = FrontElevatorMotor.getEncoder();
    public final RelativeEncoder ElevatorEncoder2 = BackElevatorMotor.getEncoder();
    public double targetSetpoint;

    public Command setElevatorSetpoint(double setpoint) {
        return Commands.runOnce(() -> targetSetpoint = setpoint);
    }

    @Override
    public void periodic() {
        //Display sensor readings to ShuffleBoard
        SmartDashboard.putNumber("Elevator Velocity1", ElevatorEncoder1.getVelocity());
        SmartDashboard.putNumber("Elevator Velocity2", ElevatorEncoder2.getVelocity());
        SmartDashboard.putNumber("Elevator Encoder1", ElevatorEncoder1.getPosition());
        SmartDashboard.putNumber("Elevator Encoder2", ElevatorEncoder2.getPosition());
}
    //This function sets both motors to one adjustable speed
    public void setElevatorMotor(double speed) {
        FrontElevatorMotor.set(speed);
        BackElevatorMotor.set(speed);
}
    //Since both motors move at the same time, only use one encoder
    public double getElevatorEncoder1() {
        return ElevatorEncoder1.getPosition();
}
    public void resetElevatorEncoder1() {
        ElevatorEncoder1.setPosition(0);
}
}