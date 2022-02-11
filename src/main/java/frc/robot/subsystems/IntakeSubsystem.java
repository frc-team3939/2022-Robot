// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  // TODO Verify real pnumeatic hub IDs when completed
  public DoubleSolenoid solenoidLeft;
  public DoubleSolenoid solenoidRight;
  public TalonSRX intakeMotor;

  public IntakeSubsystem() {
    solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    solenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);
    intakeMotor = new TalonSRX(intakemotor);

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
  
  // TODO add runIntake() and stopIntake() functions when motor controller is apparent
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
