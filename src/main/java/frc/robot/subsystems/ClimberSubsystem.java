package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the pneumatic climber cylinder. */
public class ClimberSubsystem extends SubsystemBase {
    private final DoubleSolenoid m_climberSolenoid;

    private static final int kSolenoidForwardChannel = 0;
    private static final int kSolenoidReverseChannel = 1;

    public ClimberSubsystem() {
        m_climberSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            kSolenoidForwardChannel,
            kSolenoidReverseChannel
        );

        m_climberSolenoid.set(Value.kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber/Extended", m_climberSolenoid.get() == Value.kForward);
    }

    /** Returns true if the climber is currently extended. */
    public boolean isExtended() {
        return m_climberSolenoid.get() == Value.kForward;
    }

    /** Toggles the climber between extended and retracted. */
    public Command toggleCommand() {
        return Commands.runOnce(() -> m_climberSolenoid.toggle(), this);
    }
}