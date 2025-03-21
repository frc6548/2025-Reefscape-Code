// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.ClimberPIDCommand;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommandV2;
import frc.robot.commands.IntakePivotPIDCommand;
import frc.robot.commands.OuttakeCommand;
// import frc.robot.commands.PinPIDCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Climber;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    

    //Creates new xbox controller on port 0
    private final CommandXboxController joystick = new CommandXboxController(0);

    //Create instances of our sybsystems 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intakeSubsystem = new Intake();
    public final Elevator elevatorSubsystem = new Elevator();
    public final LEDs ledSubsystem = new LEDs();
    public final Climber ClimberSubsystem = new Climber();

    //Creates instances of our Commands
    public final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, ledSubsystem);
    public final IntakeCommandV2 intakeCommandV2 = new IntakeCommandV2(intakeSubsystem, ledSubsystem);

    //Creates instance of autochooser 
    private SendableChooser<Command> autoChooser;

    public RobotContainer() { //Called only once when robot first turns on
        //Sets default autonomous routine
        //Puts the autonomous choser in Shuffleboard
        //Converts to commands that are useable only in Pathplanner
        NamedCommands.registerCommand("elevatorUp", new ElevatorPIDCommand(elevatorSubsystem,27.8+1.285715222358704));
        NamedCommands.registerCommand("elevatorDown", new ElevatorPIDCommand(elevatorSubsystem,0+1.285715222358704));
        NamedCommands.registerCommand("wristUp", new IntakePivotPIDCommand(intakeSubsystem, 12.5-16.35712432861328));
        NamedCommands.registerCommand("outtake", new OuttakeCommand(intakeSubsystem, ledSubsystem, -11));
        NamedCommands.registerCommand("intake", new IntakeCommand(intakeSubsystem, ledSubsystem));
        NamedCommands.registerCommand("elevatorIntake", new ElevatorPIDCommand(elevatorSubsystem,1.75+1.285715222358704));
        NamedCommands.registerCommand("wristIntake", new IntakePivotPIDCommand(intakeSubsystem,-.5-16.35712432861328));

        // NamedCommands.registerCommand("elevatorDown", new AutonomousElevatorCommand(elevatorSubsystem, 2)); 
        autoChooser = AutoBuilder.buildAutoChooser("Right Coral");
        SmartDashboard.putData("Auto Mode", autoChooser);
        //Runs functions for one time robot configurations
        configureBindings();
        ledSubsystem.ConfigureLEDs();

        Pose2d latestMt1 = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        new SequentialCommandGroup (
        new InstantCommand(() -> drivetrain.resetPose(new Pose2d(new Translation2d(0, 0), latestMt1.getRotation()))),
        (new InstantCommand(() -> System.out.println(1))),
        (new WaitCommand(.25)),
        (new InstantCommand(() -> drivetrain.resetPose(new Pose2d(new Translation2d(0, 0), latestMt1.getRotation())))),
        (new InstantCommand(() -> System.out.println(1))),
        (new WaitCommand(.25)),
        (new InstantCommand(() -> drivetrain.resetPose(new Pose2d(new Translation2d(0, 0), latestMt1.getRotation())))),
        (new InstantCommand(() -> System.out.println(1))),
        (new WaitCommand(.25)),
        (new InstantCommand(() -> drivetrain.resetPose(new Pose2d(new Translation2d(0, 0), latestMt1.getRotation())))),
        (new InstantCommand(() -> System.out.println(1)))).ignoringDisable(true).schedule();
        }
    
    public double elevatorSetpointOffset = 1.285715222358704;//only have to change this in 1 spot to update the entire robot
    public double pivotSetpointOffset =-14; //-16.35712432861328;//only have to change this in 1 spot to update the entire robot and dont have to hunt for numbers in code anywhere else. 

    private void configureBindings() { //Binds Xbox controller bottons to Commands
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //INTAKE
        joystick.y().toggleOnTrue(intakeCommandV2);
        joystick.y().onTrue(new ElevatorPIDCommand(elevatorSubsystem,1.75+elevatorSetpointOffset));
        joystick.y().onTrue(new IntakePivotPIDCommand(intakeSubsystem, -.5+pivotSetpointOffset));
        joystick.y().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(4)));

        //OUTTAKE
        joystick.rightBumper().onTrue(new OuttakeCommand(intakeSubsystem, ledSubsystem, -11));
        //L4
        joystick.povUp().onTrue(new ElevatorPIDCommand(elevatorSubsystem,28.35+elevatorSetpointOffset)); //27.5
        joystick.povUp().onTrue(new IntakePivotPIDCommand(intakeSubsystem,12.5+pivotSetpointOffset));
        joystick.povUp().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(1)));
        //L3
        joystick.povRight().onTrue(new ElevatorPIDCommand(elevatorSubsystem,13.6+elevatorSetpointOffset));
        joystick.povRight().onTrue(new IntakePivotPIDCommand(intakeSubsystem,12.5+pivotSetpointOffset)); 
        joystick.povRight().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(2.3)));
        //DEALGIFY L3
        joystick.rightTrigger().onTrue(new ElevatorPIDCommand(elevatorSubsystem,21.4+elevatorSetpointOffset));
        joystick.rightTrigger().onTrue(new IntakePivotPIDCommand(intakeSubsystem,7+pivotSetpointOffset));
        joystick.rightTrigger().onTrue(new OuttakeCommand(intakeSubsystem, ledSubsystem, 30));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(1.5)));
        //L2
        joystick.povDown().onTrue(new ElevatorPIDCommand(elevatorSubsystem,4.7+elevatorSetpointOffset));
        joystick.povDown().onTrue(new IntakePivotPIDCommand(intakeSubsystem,12.5+pivotSetpointOffset)); 
        joystick.povDown().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(6)));
        //DEALGIFY L2
        joystick.leftTrigger().onTrue(new ElevatorPIDCommand(elevatorSubsystem,12.5+elevatorSetpointOffset));
        joystick.leftTrigger().onTrue(new IntakePivotPIDCommand(intakeSubsystem,7+pivotSetpointOffset));
        joystick.leftTrigger().onTrue(new OuttakeCommand(intakeSubsystem, ledSubsystem, 30));
        joystick.leftTrigger().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(3)));
        //Process
        // joystick.x().onTrue(new ElevatorPIDCommand(elevatorSubsystem,6+elevatorSetpointOffset));
        // joystick.x().onTrue(intakeSubsystem.setPivotSetpoint(3+pivotSetpointOffset));
        // joystick.x().onTrue(new OuttakeCommand(intakeSubsystem, ledSubsystem, 30));
        // joystick.x().onTrue(new InstantCommand(() -> SetDriveTrainSpeed(4.5)));
        
        joystick.x().onTrue(Commands.parallel(
            new ElevatorPIDCommand(elevatorSubsystem,6+elevatorSetpointOffset),
            new IntakePivotPIDCommand(intakeSubsystem,3+pivotSetpointOffset),
            new OuttakeCommand(intakeSubsystem, ledSubsystem, 30),
            new InstantCommand(() -> SetDriveTrainSpeed(4.5))
            ));

        //CLIMBER OUT + PIN PULL
        // joystick.start().and(joystick.x()).onTrue((new PinPIDCommand(ClimberSubsystem, 7.5)));
        // joystick.start().and(joystick.a()).onTrue((new ClimberPIDCommand(ClimberSubsystem, 42.6)));
        // // CLIMB UP
        // joystick.start().and(joystick.b()).onTrue(new ClimberPIDCommand(ClimberSubsystem, 10));

        // CTRE SysID Tests, not used yet 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        //KS = 0.051356, KV = 0.11241, KA = 0.012258

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void SetDriveTrainSpeed(double speed){
        MaxSpeed = speed;
        drive.withDeadband(MaxSpeed * 0.1); // Update the deadband based on the new MaxSpeed
    }
}