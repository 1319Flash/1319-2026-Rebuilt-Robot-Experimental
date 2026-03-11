package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the single uptake motor that feeds notes into the flywheel. */
public class UptakeSubsystem extends SubsystemBase {
    private final TalonFX m_uptakeMotor;

    private static final int    kUptakeMotorId = 10;
    private static final String kCanBus        = "";
    private static final double kUptakeSpeed   = -1.0;

    // Current limits — uptake is more likely to jam so limits are lower than flywheel
    private static final double kStatorCurrentLimit = 40.0;
    private static final double kSupplyCurrentLimit = 30.0;

    public UptakeSubsystem() {
        m_uptakeMotor = new TalonFX(kUptakeMotorId, kCanBus);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration uptakeConfig = new TalonFXConfiguration();

        uptakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        uptakeConfig.CurrentLimits.StatorCurrentLimit       = kStatorCurrentLimit;
        uptakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        uptakeConfig.CurrentLimits.SupplyCurrentLimit       = kSupplyCurrentLimit;
        uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_uptakeMotor.getConfigurator().apply(uptakeConfig);
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