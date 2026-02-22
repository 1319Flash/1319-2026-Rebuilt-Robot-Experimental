package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the dual-motor flywheel shooter.
 *
 * Velocity model: v(d) = 32.165 + 10.599d (linear, least squares fit)
 * Calibration data (limelight lens to AprilTag center):
 *   0.94 m → 42.5 RPS
 *   1.73 m → 50.0 RPS
 *   2.67 m → 60.0 RPS
 *   3.05 m → 65.0 RPS
 */
public class FlyWheelSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX m_shooterMotor1;
    private final TalonFX m_shooterMotor2;
    private final VelocityVoltage m_velocityRequest;

    // CAN IDs
    private static final int    kMotor1Id = 11;
    private static final int    kMotor2Id = 12;
    private static final String kCanBus   = "canivore";

    // PID gains — kP is constant across all velocities
    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.2;
    private static final double kV = 0.133;
    private static final double kA = 0.01;

    // Linear velocity model coefficients: v = kVelocityIntercept + kVelocitySlope * d
    private static final double kVelocityIntercept = 32.165;
    private static final double kVelocitySlope     = 10.599;

    // Velocity limits
    private static final double kMinVelocityRps = 30.0;
    private static final double kMaxVelocityRps = 75.0;

    // At-speed tolerance and safety timeout
    private static final double kVelocityToleranceRps  = 2.0;
    private static final double kAtSpeedTimeoutSeconds = 3.0;

    // State
    private double m_targetVelocity = 0.0;

    public FlyWheelSubsystem() {
        m_shooterMotor1 = new TalonFX(kMotor1Id, kCanBus);
        m_shooterMotor2 = new TalonFX(kMotor2Id, kCanBus);
        m_velocityRequest = new VelocityVoltage(0).withSlot(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_shooterMotor1.getConfigurator().apply(config);
        m_shooterMotor2.setControl(new Follower(kMotor1Id, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Velocity",       getCurrentVelocity());
        SmartDashboard.putNumber("Shooter/TargetVelocity", m_targetVelocity);
        SmartDashboard.putBoolean("Shooter/AtSpeed",       isAtSpeed());
    }

    // ========== Getters ==========

    public double getCurrentVelocity() {
        return m_shooterMotor1.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed() {
        return Math.abs(m_targetVelocity - getCurrentVelocity()) < kVelocityToleranceRps;
    }

    // ========== Private Helpers ==========

    /**
     * Calculates target velocity for a given distance using the linear model.
     * v(d) = 32.165 + 10.599d
     */
    private double calculateVelocityForDistance(double distanceMeters) {
        double velocity = kVelocityIntercept + kVelocitySlope * distanceMeters;
        return Math.max(kMinVelocityRps, Math.min(kMaxVelocityRps, velocity));
    }

    private void setVelocity(double velocityRps) {
        m_targetVelocity = velocityRps;
        m_shooterMotor1.setControl(m_velocityRequest.withVelocity(velocityRps));
    }

    private void setVelocityForDistance(double distanceMeters) {
        setVelocity(calculateVelocityForDistance(distanceMeters));
    }

    private void stop() {
        m_targetVelocity = 0.0;
        m_shooterMotor1.set(0);
    }

    // ========== Commands ==========

    /** Spins the flywheel to the correct velocity for the given distance in meters. */
    public Command shootAtDistance(double distanceMeters) {
        return runOnce(() -> setVelocityForDistance(distanceMeters));
    }

    /** Spins the flywheel to a specific velocity in RPS. */
    public Command shootAtVelocity(double velocityRps) {
        return runOnce(() -> setVelocity(velocityRps));
    }

    /** Stops the flywheel. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /**
     * Waits until the flywheel reaches target speed.
     * Times out after {@value kAtSpeedTimeoutSeconds} seconds as a safety measure.
     */
    public Command waitUntilAtSpeed() {
        return run(() -> {}).until(this::isAtSpeed).withTimeout(kAtSpeedTimeoutSeconds);
    }
}