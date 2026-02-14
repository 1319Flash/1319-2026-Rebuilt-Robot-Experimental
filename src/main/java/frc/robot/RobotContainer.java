// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AprilTagAlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.UptakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Speed multipliers
    private final double[] speedMultiplier = {0.7};
    private final double SLOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.7;
    private final double TURBO_SPEED = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    //Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public final FlyWheelSubsystem flyWheelSubsystem = new FlyWheelSubsystem();
    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public final UptakeSubsystem uptakeSubsystem = new UptakeSubsystem();

    //Path follower - NOT final so we can initialize it in constructor
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        limelightSubsystem.setDrivetrain(drivetrain);
        
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        // Home the hood on robot startup
        hoodSubsystem.homeCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMultiplier[0]) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMultiplier[0]) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * speedMultiplier[0]) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Speed Control
        driverController.leftTrigger().whileTrue(
            Commands.runOnce(() -> speedMultiplier[0] = SLOW_SPEED)
        ).onFalse(
            Commands.runOnce(() -> speedMultiplier[0] = NORMAL_SPEED)
        );
        driverController.rightTrigger().whileTrue(
            Commands.runOnce(() -> speedMultiplier[0] = TURBO_SPEED)
        ).onFalse(
            Commands.runOnce(() -> speedMultiplier[0] = NORMAL_SPEED)
        );

        // Brake mode
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));
        // Points wheels in controller direction
        driverController.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Drives forwards
        driverController.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        // Drives backwards
        driverController.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // IMPORTANT: Re-run SysId after enabling FOC - the gains will be different!
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Tracks AprilTag
        driverController.rightBumper().whileTrue(
            new AprilTagAlignCommand(
                drivetrain,
                limelightSubsystem,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                MaxSpeed,
                MaxAngularRate,
                speedMultiplier[0]
            )
        );

        //Shoot
        operatorController.rightTrigger().whileTrue(
            Commands.sequence(
                flyWheelSubsystem.shootCommand(),
                flyWheelSubsystem.waitUntilAtSpeed(),
                uptakeSubsystem.runCommand()))
        .onFalse(
            Commands.parallel(
                flyWheelSubsystem.stopCommand(),
                uptakeSubsystem.stopCommand()
            )
        );

        // Hood manual control
        hoodSubsystem.setDefaultCommand(
            hoodSubsystem.manualControlCommand(() -> -operatorController.getLeftY() * 0.1)
        );

        // Hood preset positions (quick overrides during match)
        operatorController.povUp().onTrue(hoodSubsystem.farShotCommand());     // Far shot preset
        operatorController.povLeft().onTrue(hoodSubsystem.closeShotCommand()); // Close shot preset
        operatorController.povDown().onTrue(hoodSubsystem.stowCommand());      // Stow position

        // Manual hood home 
        operatorController.back().onTrue(hoodSubsystem.homeCommand());

        // Manual position reset
        operatorController.start().onTrue(hoodSubsystem.resetPositionCommand());
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}