// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  // Adds CANSparkMax
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;

  // Adds Encoder
    private RelativeEncoder leftClimberEncoder;
    private RelativeEncoder rightClimberEncoder;

  public Climber() {
    // Init CANSparkMotors
    leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.LEFTCLIMBER_ID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.RIGHTCLIMBER_ID, MotorType.kBrushless);

    // Sets up climber encoders
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);

    // Resets Motors, factory defaults
    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    // Allows an idle stage
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
  }

// Get left climber motor position
  public double getLeftPosition() {
    return leftClimberEncoder.getPosition();
  }
  
// Get right climber motor position
  public double getRightPosition() {
    return rightClimberEncoder.getPosition();
  }

// Get left climber motor velocity
  public double getLeftVelocity() {
    return leftClimberEncoder.getVelocity();
  }

// Get left climber motor velocity
  public double getRightVelocity() {
    return rightClimberEncoder.getVelocity();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}