// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public DoubleSolenoid solenoidLeft;
  public DoubleSolenoid solenoidRight;
  public TalonSRX intakeMotor;
  public TalonSRX intermediaryMotor;
  public DigitalInput middleSwitch;

  public IntakeSubsystem() {
    solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 2);
    solenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    intakeMotor = new TalonSRX(intakemotor);
    intermediaryMotor = new TalonSRX(intermediarymotor);
    middleSwitch = new DigitalInput(0);

    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  // Below functions fire/retract the pistons that control the intake extension.
  public void intakeExtend() {
    solenoidLeft.set(kForward);
    solenoidRight.set(kForward);
  }

  public void intakeRetract() {
    solenoidLeft.set(kReverse);
    solenoidRight.set(kReverse);
  }

  public void intakeRunFullSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void intakeReverse() {
    intakeMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void intakeStop() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void intakeRunSetSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public void runMiddleMotor(double speed) {
    intermediaryMotor.set(ControlMode.PercentOutput, speed); //gonna need hte power for this
  }

  public boolean isMiddleLimitActivated() {
    return middleSwitch.get();
  }
  public void runFullIntake(double speed) {
    intermediaryMotor.set(ControlMode.PercentOutput, -1); 
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }
}