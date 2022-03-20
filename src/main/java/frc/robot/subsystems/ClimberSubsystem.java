// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  public RelativeEncoder encoder;
  public SparkMaxPIDController winchPID;
  public SparkMaxLimitSwitch limit;
  
  public ClimberSubsystem() {
    winch = new CANSparkMax(climbermotor, MotorType.kBrushless);
    encoder = winch.getEncoder(Type.kHallSensor, 42);
    limit = winch.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
    limit.enableLimitSwitch(true);
    winchPID = winch.getPIDController();

    leftArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 4);
    rightArm = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7); //change ids for 1-8
    leftLimitSwitch = new DigitalInput(4);
    rightLimitSwitch = new DigitalInput(5);

    winch.setIdleMode(IdleMode.kBrake);
    winch.setSoftLimit(SoftLimitDirection.kForward, 350);

    winchPID.setOutputRange(-1, 1);
    winchPID.setP(5e-5);
    winchPID.setI(1e-6);
    winchPID.setD(0);
    winchPID.setIZone(0);
    winchPID.setFF(0.000156);
    winchPID.setSmartMotionMaxVelocity(4200, 0);
    winchPID.setSmartMotionMinOutputVelocity(0, 0);
    winchPID.setSmartMotionMaxAccel(3250, 0);
    winchPID.setSmartMotionAllowedClosedLoopError(0.01, 0);
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
  
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public double getEncoder() {
    return encoder.getPosition();
  }

  public void goToEncoder(double pos) {
    winchPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
  }
  public boolean checkIfAtLimit() {
    if (limit.isPressed() == true) {
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

  public void stopWinch() {
    winchPID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
