// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

// Rev imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


// WPI imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  // Adds CANSparkMax
    private CANSparkMax climberMotor;

  // Adds Encoder
    private RelativeEncoder climberEncoder;
    private String climberName;

  public Climber(int motor_ID, boolean isInverted, String climberName) {
    this.climberName = climberName;
    // Init CANSparkMotors
    climberMotor = new CANSparkMax(motor_ID, MotorType.kBrushless);

    // Sets up climber encoder
    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0);

    // Resets Motors, factory defaults
    climberMotor.restoreFactoryDefaults();

    // Allows an idle stage
    climberMotor.setIdleMode(IdleMode.kBrake);

    // Inverts motor direction
    climberMotor.setInverted(isInverted);
  }

// Get climber motor position
  public double getPosition() {
    return climberEncoder.getPosition();
  }
  

// Get climber motor velocity
  public double getVelocity() {
    return climberEncoder.getVelocity();
  }

// Climber up
  public void climb(double speed) {
    climberMotor.set(speed);
  }

// Climber down


  // Reset encoder
  public void resetEncoder() {
    climberEncoder.setPosition(0);
  } 

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Displays climber encoder value on SmartDashBoard
    SmartDashboard.putNumber(climberName + " Climber Motor Encoder Value: ", getPosition());
  }
}