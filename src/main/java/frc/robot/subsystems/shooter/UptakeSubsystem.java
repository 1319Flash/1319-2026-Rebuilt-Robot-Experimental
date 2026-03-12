package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the uptake and roller motors that feed notes into the flywheel. */
public class UptakeSubsystem extends SubsystemBase {
    private final TalonFX m_uptakeMotor;
    private final TalonFX m_rollerMotor;

    private static final int    kUptakeMotorId = 9;
    private static final int    kRollerMotorId = 10;
    private static final String kCanBus        = "";
    private static final double kUptakeSpeed   = 1.0;

    // Current limits — lower than flywheel since uptake is a lighter mechanism
    private static final double kStatorCurrentLimit = 40.0;
    private static final double kSupplyCurrentLimit = 30.0;

    public UptakeSubsystem() {
        m_uptakeMotor = new TalonFX(kUptakeMotorId, kCanBus);
        m_rollerMotor = new TalonFX(kRollerMotorId, kCanBus);

        configureMotors();
    }

    private void configureMotors() {
        // Neutral mode and current limits must be set through the configurator
        // to persist correctly through device resets
        TalonFXConfiguration uptakeConfig = new TalonFXConfiguration();
        uptakeConfig.MotorOutput.NeutralMode             = NeutralModeValue.Brake;
        uptakeConfig.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        uptakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        uptakeConfig.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_uptakeMotor.getConfigurator().apply(uptakeConfig);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode             = NeutralModeValue.Coast;
        rollerConfig.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_rollerMotor.getConfigurator().apply(rollerConfig);

        // Roller follows uptake in the opposite direction to feed toward the shooter
        m_rollerMotor.setControl(new Follower(kUptakeMotorId, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {}

    private void setUptakeSpeed(double speed) {
        m_uptakeMotor.set(speed);
    }

    private void stop() {
        m_uptakeMotor.set(0);
    }

    /** Runs the uptake at full speed. */
    public Command runCommand() {
        return runOnce(() -> setUptakeSpeed(kUptakeSpeed));
    }

    /** Stops the uptake. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /** Runs the uptake at a custom speed. */
    public Command runAtSpeed(double speed) {
        return runOnce(() -> setUptakeSpeed(speed));
    }

    /** Reverses the uptake for unjamming. */
    public Command reverseCommand() {
        return runOnce(() -> setUptakeSpeed(-kUptakeSpeed));
    }
}