// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final DoubleSolenoid climberDoubleSolenoid;
  private final Compressor m_Compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    climberDoubleSolenoid.set(Value.kReverse);
  }

  public boolean getClimberDoubleSolenoidState() {
    return climberDoubleSolenoid.get() == Value.kReverse;
  }

  public double getPressureValve(){
    return m_Compressor.getPressure();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Activation", getClimberDoubleSolenoidState());
    SmartDashboard.putNumber("Total Pressure", getPressureValve());
  }

  public Command toggleCommand() {
    return Commands.runOnce(() -> climberDoubleSolenoid.toggle(), this);
  }
}