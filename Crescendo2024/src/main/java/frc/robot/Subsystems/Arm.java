// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
CANSparkMax Arm;

  public Arm() {
    Arm = new CANSparkMax(1, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
