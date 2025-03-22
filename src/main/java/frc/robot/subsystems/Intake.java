package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//This is the sybsystem that houses our Intake Motor + Intake Pivot Motor
public class Intake extends SubsystemBase{
    private final SparkMax IntakeMotor = new SparkMax(19, MotorType.kBrushless);
    private final SparkMax PivotMotor = new SparkMax(18, MotorType.kBrushless);
    public final RelativeEncoder IntakeEncoder = IntakeMotor.getEncoder();
    public final RelativeEncoder PivotEncoder = PivotMotor.getEncoder();
    //This is our photoelectric sensor, connected to DIO 0
    public final DigitalInput Input = new DigitalInput(0);
    public double targetSetpoint;

    SparkMaxConfig pivotConfig;

    SparkClosedLoopController sparkPIDController; 
    
    // private final PIDController pidController;
    // private final ArmFeedforward feedforward;
    private static double kP = 055; //.6 
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kG = 0.025; //.07
    private static double kV = 0.0;
    public double positionalTolerance = 0.2; //if the encoder is less than this distance (1 revolution) it is considered at the setpoint. this is used by the intake commands.


    public Intake() {
      // this.pidController = new PIDController(kP, kI, kD);
      // this.feedforward = new ArmFeedforward(kS, kG, kV);

      //this.pidController.setTolerance(positionalTolerance);

      pivotConfig = new SparkMaxConfig();

      sparkPIDController = PivotMotor.getClosedLoopController();

      pivotConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(30,35);

      this.PivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

      this.resetPivotEncoder();
    }
    public Command setPivotSetpoint(double setpoint) {
      return Commands.runOnce(() -> targetSetpoint = setpoint);
  }

  public void executePid() {
    //if (pidController.getSetpoint() != targetSetpoint)
    //pidController.setSetpoint(targetSetpoint);
    //double speed = pidController.calculate(getPivotEncoder());
    //+ feedforward.calculate(0, pidController.getSetpoint());
      //MathUtil.inputModulus(speed, speed, speed);
    //setPivotMotor(speed);
    sparkPIDController.setReference(targetSetpoint, ControlType.kPosition);
    
}

     @Override
     public void periodic() {
        //Display sensor readings to ShuffleBoard
        executePid();
        SmartDashboard.putNumber("Intake Velocity", IntakeEncoder.getVelocity());
        SmartDashboard.putNumber("Intake Encoder", IntakeEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Velocity", PivotEncoder.getVelocity());
        SmartDashboard.putNumber("Pivot Encoder", PivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Current", PivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot Setpoint", targetSetpoint);
        SmartDashboard.putNumber("Pivot Error", targetSetpoint - PivotEncoder.getPosition());
        SmartDashboard.putBoolean("Photoelectric Sensor Status", Input.get());
     }

     public void setIntakeMotor(double speed) {
      IntakeMotor.set(speed);
    }  

    public void setPivotMotor(double speed) {
      PivotMotor.set(speed);
    }  

    public double getPivotEncoder() {
      return PivotEncoder.getPosition();
    }

    public double getIntakeEncoder() {
      return IntakeEncoder.getPosition();
    }

    //This is the status of our Photoelectric sensor
    public boolean getPEStatus(){
      return Input.get();
    }

    public void resetPivotEncoder() {
      PivotEncoder.setPosition(0);
    }  

    public void resetIntakeEncoder() {
      IntakeEncoder.setPosition(0);
    }
}