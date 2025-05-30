// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.util.Units;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final boolean kUseLimelight = true;
  private boolean beforeMatch = true;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    CommandScheduler.getInstance().run(); 
      // if (Math.random() > .95) {
      //  System.out.println("headingDeg: " + headingDeg);
      // }
      if (kUseLimelight && !DriverStation.isTeleop()) {
        var driveState = m_robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
  
        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
      }
    }
  
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // m_robotContainer.drivetrain.resetPose(new Pose2d(new Translation2d(0, 0), latestMt1.getRotation()));
    if (llMeasurement != null && llMeasurement.tagCount > 0 && beforeMatch) {
      m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    beforeMatch = false;
  }

  @Override
  public void autonomousPeriodic() {
    // if (kUseLimelight) {
    //   var driveState = m_robotContainer.drivetrain.getState();
    // double headingDeg = driveState.Pose.getRotation().getDegrees();
    // double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    // LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
    // var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
    // if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
    //   m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
    // }
  }
  

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
