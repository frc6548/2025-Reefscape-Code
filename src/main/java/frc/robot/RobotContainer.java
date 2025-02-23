// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePivotPIDCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

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
    
    //Creates instances of our Commands
    public final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, ledSubsystem);

    //Creates instance of autochooser 
    private SendableChooser<Command> autoChooser;

    
    public RobotContainer() { //Called only once when robot first turns on
        //Sets default autonomous routine
        autoChooser = AutoBuilder.buildAutoChooser("Right Coral");
        //Puts the autonomous choser in Shuffleboard
        SmartDashboard.putData("Auto Mode", autoChooser);
        //Converts to commands that are useable only in Pathplanner
        NamedCommands.registerCommand("elevatorUp", new ElevatorPIDCommand(elevatorSubsystem, 16));
        NamedCommands.registerCommand("elevatorDown", new ElevatorPIDCommand(elevatorSubsystem, 2));
        //Runs functions for one time robot configurations
        configureBindings();
        ledSubsystem.ConfigureLEDs();
    }

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

        joystick.y().toggleOnTrue(intakeCommand);
        joystick.rightBumper().whileTrue(new InstantCommand(() -> intakeSubsystem.setIntakeMotor(-1))); 
        joystick.rightBumper().whileFalse(new InstantCommand(() -> intakeSubsystem.setIntakeMotor(0)));
        joystick.povDown().onTrue(new ElevatorPIDCommand(elevatorSubsystem, 2));
        joystick.povUp().onTrue(new ElevatorPIDCommand(elevatorSubsystem, 25));
        joystick.povLeft().onTrue(new ElevatorPIDCommand(elevatorSubsystem, 4.5));
        joystick.povRight().onTrue(new ElevatorPIDCommand(elevatorSubsystem, 16));
        joystick.leftTrigger().onTrue(new IntakePivotPIDCommand(intakeSubsystem, 14));
        joystick.rightTrigger().onTrue(new IntakePivotPIDCommand(intakeSubsystem, 2));

        //Our SysID Tests, not used yet
        joystick.start().whileTrue(new InstantCommand(() -> drivetrain.m_sysIdRoutineSteer.dynamic(Direction.kForward)));
        joystick.back().whileTrue(new InstantCommand(() -> drivetrain.m_sysIdRoutineRotation.dynamic(Direction.kForward)));

        // CTRE SysID Tests, not used yet TODO
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
