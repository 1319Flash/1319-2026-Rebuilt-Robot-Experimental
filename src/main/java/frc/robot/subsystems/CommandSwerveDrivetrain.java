package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/** Swerve drivetrain subsystem built on Phoenix 6 with 250 Hz odometry and vision measurement integration. */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;

    private static final Rotation2d kBlueAlliancePerspective = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspective  = Rotation2d.k180deg;

    private boolean m_hasAppliedOperatorPerspective = false;

    private Notifier m_simNotifier = null;
    private double   m_lastSimTime;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains  m_steerCharacterization       = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation    m_rotationCharacterization    = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, 250.0, modules);
        configureProFeatures();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        configureProFeatures();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        configureProFeatures();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Configures odometry-critical signals to 250 Hz and throttles non-critical
     * signals to reduce CAN bus utilization.
     */
    private void configureProFeatures() {
        for (int i = 0; i < 4; i++) {
            var module = getModule(i);

            module.getDriveMotor().getPosition().setUpdateFrequency(250);
            module.getDriveMotor().getVelocity().setUpdateFrequency(250);
            module.getSteerMotor().getPosition().setUpdateFrequency(250);
            module.getEncoder().getPosition().setUpdateFrequency(250);
            module.getEncoder().getAbsolutePosition().setUpdateFrequency(250);

            module.getDriveMotor().getMotorVoltage().setUpdateFrequency(50);
            module.getDriveMotor().getSupplyCurrent().setUpdateFrequency(50);
            module.getDriveMotor().getStatorCurrent().setUpdateFrequency(50);
            module.getDriveMotor().getDeviceTemp().setUpdateFrequency(4);
            module.getSteerMotor().getMotorVoltage().setUpdateFrequency(50);
            module.getSteerMotor().getSupplyCurrent().setUpdateFrequency(50);
            module.getSteerMotor().getStatorCurrent().setUpdateFrequency(50);
            module.getSteerMotor().getDeviceTemp().setUpdateFrequency(4);
        }

        getPigeon2().getYaw().setUpdateFrequency(250);
        getPigeon2().getAngularVelocityZWorld().setUpdateFrequency(250);

        getPigeon2().getAccelerationX().setUpdateFrequency(100);
        getPigeon2().getAccelerationY().setUpdateFrequency(100);
        getPigeon2().getTemperature().setUpdateFrequency(4);
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds
                        .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /** Returns a command that continuously applies the given swerve request. */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspective
                        : kBlueAlliancePerspective
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("Swerve/OdometryHz",   1.0 / getState().OdometryPeriod);
        SmartDashboard.putNumber("Swerve/TotalCurrent", getTotalCurrentDraw());
    }

    /** Returns the sum of all drive and steer motor stator currents. */
    private double getTotalCurrentDraw() {
        double total = 0;
        for (int i = 0; i < 4; i++) {
            total += getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble();
            total += getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble();
        }
        return total;
    }

    /**
     * Adds a vision measurement to the pose estimator with caller-supplied standard deviations.
     *
     * @param visionRobotPoseMeters the pose reported by vision
     * @param timestampSeconds      timestamp of the measurement in seconds
     * @param xyStdDev              translational standard deviation in meters
     * @param thetaStdDev           rotational standard deviation in radians
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        double xyStdDev,
        double thetaStdDev
    ) {
        addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        );
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        );
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}