package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        UsbCamera frontCamera = CameraServer.startAutomaticCapture("Front Camera", 0);
        frontCamera.setResolution(320, 240);
        frontCamera.setFPS(15);

        /*
        UsbCamera backCamera = CameraServer.startAutomaticCapture("Back Camera", 1);
        backCamera.setResolution(320, 240);
        backCamera.setFPS(15);
        */

        SignalLogger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void disabledInit() {
        m_robotContainer.limelightSubsystem.setAutoMode(false);
        m_robotContainer.limelightSubsystem.setLEDsOff();
        SignalLogger.stop();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.limelightSubsystem.setAutoMode(true);
        m_robotContainer.limelightSubsystem.setVisionUpdatesEnabled(true);

        // Attempt to snap the starting pose to vision before the auto runs.
        // Requires at least 2 tags within 3.5 m. Falls back to PathPlanner pose if it fails.
        m_robotContainer.limelightSubsystem.resetPoseFromVision();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_robotContainer.limelightSubsystem.setAutoMode(false);
        m_robotContainer.limelightSubsystem.setVisionUpdatesEnabled(true);

        // Seed pose from vision so the jump filter doesn't reject measurements
        // if the robot was never put through auto first.
        m_robotContainer.limelightSubsystem.resetPoseFromVision();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}