// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  CANSparkMax armMotor;

  public Arm() {
    armMotor = new CANSparkMax(Constants.ArmMotorConstants.ARM_DRIVE_ID, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
  }
  public void ArmForwards(double arm) {
    armMotor.set(0.6);
  }

  public void ArmBackwards(double arm) {
    armMotor.set(-0.6);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
