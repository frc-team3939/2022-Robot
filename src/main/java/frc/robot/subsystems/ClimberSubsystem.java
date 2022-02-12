// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotMap.*;

public class ClimberSubsystem extends SubsystemBase {
  public CANSparkMax winch;
  public DoubleSolenoid leftArm;
  public DoubleSolenoid rightArm;
  public DigitalInput leftLimitSwitch;
  public DigitalInput rightLimitSwitch;
  
  public ClimberSubsystem() {
    winch = new CANSparkMax(climbermotor, MotorType.kBrushless);
    leftArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);
    rightArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8); //change ids for 1-8
    leftLimitSwitch = new DigitalInput(2);
    rightLimitSwitch = new DigitalInput(3);

    winch.setIdleMode(IdleMode.kBrake);
  }

  public void angleArms() {
    leftArm.set(kForward);
    rightArm.set(kForward);
  }

  public void uprightArms(){
    leftArm.set(kReverse);
    rightArm.set(kReverse);
  }

  public void pullWinch(double s) {
    winch.set(s);
  }
  
  public boolean checkIfAtLimit() {
    if (leftLimitSwitch.get() == true || rightLimitSwitch.get() == true) {
      return true;
    } else {
      return false;
    }
  }

  public void releaseWinch() {
    winch.setIdleMode(IdleMode.kCoast);
  }

  public void lockWinch() {
    winch.setIdleMode(IdleMode.kBrake);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
