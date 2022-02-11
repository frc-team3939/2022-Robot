// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  // TODO Verify real pnumeatic hub IDs when completed
  DoubleSolenoid solenoidLeft;
  DoubleSolenoid solenoidRight;
  public IntakeSubsystem() {
    solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    solenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);
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

  // TODO add runIntake() and stopIntake() functions when motor controller is apparent
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
