package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

    private final PIDController pidController;
    private final ElevatorFeedforward feedforward;
    private static double kP = 0.068; //.068
    private static double kI = 0.0;
    private static double kD = 0.00; //.005
    private static double kS = 0.0;
    private static double kG = 0.075; //.07
    private static double kV = 0.0;
    private static double kA = 0.0;
    private double tolerance = 1;
    public double positionalTolerance = 1; //if the encoder is less than this distance (1 revolution) it is considered at the setpoint. this is used by the elevator commands.

    public Elevator()//Same name as the class is called the constructor. it runs once when the object of the same name is created
    {
        this.pidController = new PIDController(kP, kI, kD);
        this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    }
    public Command setElevatorSetpoint(double setpoint) {
        return Commands.runOnce(() -> targetSetpoint = setpoint);
    }
    public void executePid() {
        if (pidController.getSetpoint() != targetSetpoint)
          pidController.setSetpoint(targetSetpoint);
    
        double speed = pidController.calculate(getElevatorEncoder1()) + 
        feedforward.calculate(pidController.getSetpoint());
        setElevatorMotor(speed);
    }

    @Override
    public void periodic() {
        //update pid, will set new setpoints and adjust motor control every periodic
        executePid();
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