// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheelSubsystem extends SubsystemBase {
  // Motors
  private final TalonFX shooterMotor1;
  private final TalonFX shooterMotor2;

  // Control request
  private final VelocityVoltage velocityRequest;
  
  // Constants
  private static final int MOTOR_1_ID = 11;
  private static final int MOTOR_2_ID = 12;
  private static final String CAN_BUS = "canivore";

  // Target velocity
  private static final double SHOOTER_VELOCITY_RPS = 42.5;

  // PID Gains
  private static final double kP = 3.0;   //Proportional
  private static final double kI = 0.1;   //Intergral 
  private static final double kD = 0.0;   // Derivative
  private static final double kS = 0.2;   // Static friction
  private static final double kV = 0.1333;    // Velocity
  private static final double kA = 0.01;    // Acceleration

  // Tolerance for "at speed" check
  private static final double VELOCITY_TOLERANCE = 2.0;

    // Current target velocity
  private double targetVelocity = 0.0;

  public FlyWheelSubsystem() {
    shooterMotor1 = new TalonFX(MOTOR_1_ID, CAN_BUS);
    shooterMotor2 = new TalonFX(MOTOR_2_ID, CAN_BUS);
    
    // Create velocity control request
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    
    configureMotors();
  }
  
  /**
   * Configures both shooter motors with PID gains and proper settings
   */
  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Configure PID gains for Slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;  // Static feedforward
    slot0.kV = kV;  // Velocity feedforward
    slot0.kA = kA;  // Acceleration feedforward
    
    // Set motors to coast mode for flywheel (spins down naturally)
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Apply configuration to motor 1
    shooterMotor1.getConfigurator().apply(config);
    
    // Motor 2 follows motor 1 in opposite direction
    // Note: Motor 2 doesn't need PID config since it's just following
    shooterMotor2.setControl(new Follower(MOTOR_1_ID, MotorAlignmentValue.Opposed)); // true = opposed direction
    
    System.out.println("FlyWheel motors configured with PID velocity control");
  }

  @Override
  public void periodic() {
    // Log telemetry for monitoring and tuning
    double currentVelocity = getCurrentVelocity();
    double velocityError = targetVelocity - currentVelocity;
    
    SmartDashboard.putNumber("Shooter/Current_Velocity_RPS", currentVelocity);
    SmartDashboard.putNumber("Shooter/Target_Velocity_RPS", targetVelocity);
    SmartDashboard.putNumber("Shooter/Velocity_Error_RPS", velocityError);
    SmartDashboard.putBoolean("Shooter/At_Speed", isAtSpeed());
    SmartDashboard.putNumber("Shooter/Motor1_Voltage", shooterMotor1.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Motor1_Current", shooterMotor1.getStatorCurrent().getValueAsDouble());
  }

  // ========== Getters ==========
  
  /**
   * Gets the current velocity of shooter motor 1 (in RPS)
   */
  public double getCurrentVelocity() {
    return shooterMotor1.getVelocity().getValueAsDouble();
  }
  
  /**
   * Gets the target velocity (in RPS)
   */
  public double getTargetVelocity() {
    return targetVelocity;
  }
  
  /**
   * Checks if the shooter is at the target speed (within tolerance)
   */
  public boolean isAtSpeed() {
    return Math.abs(targetVelocity - getCurrentVelocity()) < VELOCITY_TOLERANCE;
  }

  // ========== Private Helper Methods ==========
  
  /**
   * Sets the shooter velocity using closed-loop control
   * The PID controller will automatically adjust voltage to maintain this speed
   */
  private void setVelocity(double velocityRPS) {
    targetVelocity = velocityRPS;
    shooterMotor1.setControl(velocityRequest.withVelocity(velocityRPS));
  }

  /**
   * Stops both shooter motors
   */
  private void stop() {
    targetVelocity = 0.0;
    shooterMotor1.set(0);
  }

  // ========== Public Commands ==========
  
  /**
   * Command to spin up shooter to target velocity
   * This will maintain speed automatically as balls are shot
   */
  public Command shootCommand() {
    return runOnce(() -> setVelocity(SHOOTER_VELOCITY_RPS));
  }

  /**
   * Command to stop the shooter wheels completely
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }
  
  /**
   * Command to run shooter at a custom velocity
   * @param velocityRPS Target velocity in rotations per second
   */
  public Command shootAtVelocity(double velocityRPS) {
    return runOnce(() -> setVelocity(velocityRPS));
  }
  
  /**
   * Command that waits until the shooter is at target speed
   * Useful for sequencing (e.g., don't feed balls until wheels are ready)
   */
  public Command waitUntilAtSpeed() {
    return run(() -> {}).until(this::isAtSpeed);
  }
  
  /**
   * Command that spins up and waits until ready, then runs a follow-up command
   * Example: spinUpAndShoot.andThen(feedBallsCommand)
   */
  public Command spinUpAndWaitCommand() {
    return shootCommand()
        .andThen(waitUntilAtSpeed())
        .withName("SpinUpAndWait");
  }
}