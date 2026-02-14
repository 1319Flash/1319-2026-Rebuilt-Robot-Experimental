// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UptakeSubsystem extends SubsystemBase {
  //Motor
  private final TalonFX uptakeMotor;
  private final TalonFX rollerMotor;
  
  //Constants
  private static final int UPTAKE_MOTOR_ID = 9;
  private static final int ROLLER_MOTOR_ID = 10;
  private static final String CAN_BUS = "canivore";
  private static final double UPTAKE_SPEED = 1.0;

  public UptakeSubsystem() {
    uptakeMotor = new TalonFX(UPTAKE_MOTOR_ID, CAN_BUS);
    rollerMotor = new TalonFX(ROLLER_MOTOR_ID, CAN_BUS);
    
    //Configure motor to brake when stopped
    uptakeMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);

    rollerMotor.setControl(new Follower(UPTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
  }

  // Private Helper Methods
  
  //Runs the uptake motor at specified speed
  private void setUptakeSpeed(double speed) {
    uptakeMotor.set(speed);
  }

  //Stops the uptake motor
  private void stop() {
    uptakeMotor.set(0);
  }

  //Public Commands
  
  //Command to run the uptake at default speed
  public Command runCommand() {
    return runOnce(() -> setUptakeSpeed(UPTAKE_SPEED));
  }

  //Command to stop the uptake
  public Command stopCommand() {
    return runOnce(() -> stop());
  }
  
  //Command to run uptake at custom speed
  public Command runAtSpeed(double speed) {
    return runOnce(() -> setUptakeSpeed(speed));
  }
  
  //Command to reverse the uptake (for unjamming)
  public Command reverseCommand() {
    return runOnce(() -> setUptakeSpeed(-UPTAKE_SPEED));
  }
}