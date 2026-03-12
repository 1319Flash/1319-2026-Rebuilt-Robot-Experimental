package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.UptakeSubsystem;

public class RobotContainer {
    // Max speeds
    private final double kMaxSpeed       = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Speed multiplier levels
    private static final double kSlowSpeed   = 0.5;
    private static final double kNormalSpeed = 0.7;
    private static final double kTurboSpeed  = 1.0;

    // Mutable speed multiplier — single-element array allows mutation inside lambdas
    private final double[] m_speedMultiplier = {kNormalSpeed};

    // Swerve requests
    private final SwerveRequest.FieldCentric m_fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * 0.1)
        .withRotationalDeadband(kMaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake m_brake       = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    m_pointWheels = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric m_robotCentricNudge = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Controllers
    private final CommandXboxController m_driverController   = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain         = TunerConstants.createDrivetrain();
    public final LimelightSubsystem      limelightSubsystem = new LimelightSubsystem();
    public final FlywheelSubsystem       flywheelSubsystem  = new FlywheelSubsystem();
    public final UptakeSubsystem         uptakeSubsystem    = new UptakeSubsystem();
    public final ClimberSubsystem        climberSubsystem   = new ClimberSubsystem();

    // Telemetry
    private final Telemetry m_telemetry = new Telemetry(kMaxSpeed);

    // Auto chooser
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        limelightSubsystem.setDrivetrain(drivetrain);

        registerNamedCommands();

        // "Do Nothing" is the safe default — create a PathPlanner auto with this name
        // containing no paths and no commands to ensure no unexpected auto runs
        m_autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        SmartDashboard.putData("Auto Mode", m_autoChooser);

        configureBindings();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void registerNamedCommands() {
        // SpinUp — triggered mid-path via PathPlanner event marker on far shot paths
        // so the flywheel is already at speed when the path ends
        NamedCommands.registerCommand("SpinUp",
            flywheelSubsystem.shootAtVelocity(65.0)
        );

        NamedCommands.registerCommand("ShootClose",
            Commands.sequence(
                flywheelSubsystem.shootAtVelocity(42.5),
                flywheelSubsystem.waitUntilAtSpeed(),
                Commands.deadline(
                    Commands.waitSeconds(5.0),
                    uptakeSubsystem.runCommand()
                ),
                Commands.parallel(
                    flywheelSubsystem.stopCommand(),
                    uptakeSubsystem.stopCommand()
                )
            )
        );

        // ShootFar expects SpinUp to have already been triggered mid-path.
        // Runs flywheel and uptake together for 5 seconds then stops both.
        NamedCommands.registerCommand("ShootFar",
            Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(5.0),
                    uptakeSubsystem.runCommand(),
                    flywheelSubsystem.shootAtVelocity(65.0)
                ),
                Commands.parallel(
                    flywheelSubsystem.stopCommand(),
                    uptakeSubsystem.stopCommand()
                )
            )
        );

        NamedCommands.registerCommand("Climb",
            climberSubsystem.toggleCommand()
        );
    }

    private void configureBindings() {
        // Default drive command — field-centric with live speed multiplier
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                m_fieldCentricDrive
                    .withVelocityX(-m_driverController.getLeftY()  * kMaxSpeed * m_speedMultiplier[0])
                    .withVelocityY(-m_driverController.getLeftX()  * kMaxSpeed * m_speedMultiplier[0])
                    .withRotationalRate(-m_driverController.getRightX() * kMaxAngularRate * m_speedMultiplier[0])
            )
        );

        // Idle while disabled so neutral mode is applied correctly
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        // Speed control
        m_driverController.leftTrigger()
            .onTrue(Commands.runOnce(() -> m_speedMultiplier[0] = kSlowSpeed))
            .onFalse(Commands.runOnce(() -> m_speedMultiplier[0] = kNormalSpeed));

        m_driverController.rightTrigger()
            .onTrue(Commands.runOnce(() -> m_speedMultiplier[0] = kTurboSpeed))
            .onFalse(Commands.runOnce(() -> m_speedMultiplier[0] = kNormalSpeed));

        // X-brake
        m_driverController.b().whileTrue(drivetrain.applyRequest(() -> m_brake));

        // Point wheels toward stick direction
        m_driverController.x().whileTrue(drivetrain.applyRequest(() ->
            m_pointWheels.withModuleDirection(
                new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX())
            )
        ));

        // D-pad nudge
        m_driverController.povUp().whileTrue(
            drivetrain.applyRequest(() -> m_robotCentricNudge.withVelocityX(0.5).withVelocityY(0))
        );
        m_driverController.povDown().whileTrue(
            drivetrain.applyRequest(() -> m_robotCentricNudge.withVelocityX(-0.5).withVelocityY(0))
        );

        // SysId routines
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading
        m_driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(m_telemetry::telemeterize);

        // Right bumper — AprilTag rotation alignment, driver controls translation
        m_driverController.rightBumper().whileTrue(
            new AprilTagAlignCommand(
                drivetrain,
                limelightSubsystem,
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                kMaxSpeed,
                kMaxAngularRate,
                () -> m_speedMultiplier[0]
            )
        );

        // Operator right trigger — distance-based shot using limelight
        m_operatorController.rightTrigger().whileTrue(
            Commands.sequence(
                Commands.defer(() -> {
                    double distance = limelightSubsystem.getDistanceToTargetSafe();
                    return flywheelSubsystem.shootAtDistance(distance);
                }, Set.of(flywheelSubsystem)),
                flywheelSubsystem.waitUntilAtSpeed(),
                uptakeSubsystem.runCommand()
            )
        ).onFalse(
            Commands.parallel(
                flywheelSubsystem.stopCommand(),
                uptakeSubsystem.stopCommand()
            )
        );

        // Operator start — climber toggle
        m_operatorController.start().onTrue(climberSubsystem.toggleCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}