/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//Test of git
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import static frc.robot.RobotMap.*;

import edu.wpi.first.wpilibj.DigitalInput;
//import frc.robot.Robot;
//import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * PG188 controlling the angle of the shooter and a cim / 2 redlines for the wheel spinning to shoot the balls
 */
public class ShooterSubsystem extends SubsystemBase {

  public TalonSRX angleMotor;
  public TalonSRX shooterMotor;
  public VictorSPX shooterSlave;
  public TalonSRX timingMotor;
  public DigitalInput timingLimitSwitch;

  public ShooterSubsystem() {
    angleMotor = new TalonSRX(anglemotor);
    shooterMotor = new TalonSRX(shootermotor);
    shooterSlave = new VictorSPX(shooterslave);
    timingMotor = new TalonSRX(timingmotor);
    timingLimitSwitch = new DigitalInput(1);
    
    //Adding in voltage compesation for the battery voltage
    shooterMotor.enableVoltageCompensation(true);
    shooterSlave.enableVoltageCompensation(true);

    shooterMotor.configVoltageCompSaturation(11.5);
    shooterMotor.enableVoltageCompensation(true);
    shooterMotor.config_kP(0, shooterP);
    shooterMotor.config_kI(0, shooterI);
    shooterMotor.config_kD(0, shooterD);
    shooterMotor.setInverted(false);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterSlave.follow(shooterMotor);
    shooterSlave.setInverted(false);
    angleMotor.setInverted(true);
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.configReverseSoftLimitThreshold(-1200);
    angleMotor.configReverseSoftLimitEnable(true);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    timingMotor.setNeutralMode(NeutralMode.Coast);
    timingMotor.setInverted(true);
  }
  
  public void initDefaultCommand() {
  }

  public void angleAdjust(double angle) {
    angleMotor.set(ControlMode.Position, angle);
  }
  
  /**
   * Sets the hood motor to a percentage output in speed
   * @param speed -1.0 to 1.0 percent output
   */
  public void moveAngle(double speed){
    angleMotor.set(ControlMode.PercentOutput, speed);
  }
  public void shooterSpin() {
    shooterMotor.set(ControlMode.PercentOutput, 1.0);
  }
  public void shooterStop() {
    shooterMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public void setshooterSpeed(double shooterSpeed) {
    shooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
  }
  public void feederSpeed(double timerspeed) {
    timingMotor.set(ControlMode.PercentOutput, timerspeed);
  }
  public void resetLimit(){
    angleMotor.setSelectedSensorPosition(0);
  }
  public double shooterencoder(){
    return -angleMotor.getSelectedSensorPosition();
  }

  public void setEncoderPosition(int position){
    angleMotor.setSelectedSensorPosition(0);
  }
  /**
   * Returns the value of the limit switch.
   * @return  true if it is pressed, false if it is not pressed
   */
  public boolean limitCheck(){
    if (angleMotor.isFwdLimitSwitchClosed() == 1){
      return true;
    } else {
      return false;
    }
  }

  public boolean feederLimitCheck(){
    if (timingLimitSwitch.get() == true){
      return false;
    } else {
      return true;
    }
  }
}
