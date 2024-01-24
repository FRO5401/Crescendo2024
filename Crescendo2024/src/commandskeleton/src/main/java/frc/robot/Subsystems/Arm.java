// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax armMotor;

  public Arm() {
    armMotor = new CANSparkMax(Constants.ArmConstants.ARM_ID, MotorType.kBrushless);

    //reset arm
    armMotor.restoreFactoryDefaults();
    //idle mode, lets mode be
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void armForward(){
    armMotor.set(0.60);
  }

  public void armBackward(){
    armMotor.set(-0.60);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
