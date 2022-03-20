package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import static frc.robot.RobotMap.*;

/**
 * Add your docs here.
 */
public class DriveModule {
  CANSparkMax driveMotor;
  TalonSRX angleMotor;

  AnalogInput absEncoder;

  public int offset_revs = 0;
  double speedMult = 1.0;
  double encoder_offset = 0.0;

  long time_before_next_up = 0;
  long time_before_next_down = 0;
  public double set_encoder_count = 0;

  public DriveModule(int driveId, int angleId, int encoderId, double encoderoffset) {
    driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
    angleMotor = new TalonSRX(angleId);

    absEncoder = new AnalogInput(encoderId);

    absEncoder.setAverageBits(1028);

    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setNeutralMode(NeutralMode.Coast);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, kPIDLoopIdx, kTimeoutMs);
    angleMotor.setSensorPhase(kSensorPhase);
    angleMotor.setInverted(kMotorInvert);
    angleMotor.configNominalOutputForward(0, kTimeoutMs);
    angleMotor.configNominalOutputReverse(0, kTimeoutMs);
    angleMotor.configPeakOutputForward(1, kTimeoutMs);
    angleMotor.configPeakOutputReverse(-1, kTimeoutMs);
    angleMotor.configAllowableClosedloopError(10, kPIDLoopIdx, kTimeoutMs);
    angleMotor.config_kP(0, 5);

    driveMotor.set(0.0);
    angleMotor.set(ControlMode.Disabled, 0.0);

    encoder_offset = encoderoffset;

    updatePID();
  }

  public void stop() {
    angleMotor.set(ControlMode.Disabled, 0.0);
    driveMotor.set(0.0);
  }

  /**
   * Not being used
   * @return 0
   */
  public double getRawAbsEncoder(){
    return 0;//absEncoder.getVoltage();
  }

  /**
   * Not being used
   * 
   * This returns a value 0-5 that has been rotated so that 0 is the passed in adjusted value
   * @return returns 0 //Value between 0-5V
   */
  public double getAdjustedAbsEncoder(){
    if (getRawAbsEncoder() < encoder_offset) {
      return 0;//(5-(getRawAbsEncoder()-encoder_offset));
    } else {
      return 0;//(getRawAbsEncoder()-encoder_offset);
    }
  }

  public void driveAnglePower(double percentOutput) {
    angleMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Return offset as a whole number of revolutions in angle scale -1.0 to 1.0
   */
  public double getOffset() {
    return offset_revs * 2.0;
  }

  public void setSpeed(double speed) {
    driveMotor.set(speed * speedMult);
  }

  public void setSpeedMultiplier(double newMult) {
    speedMult = newMult;
  }

  /**
   * Returns the angle on the current revolution, scale is -1.0 to 1.0 for a
   * single revolution.
   */
  public double getAngle() {
    return (getEncoder() / countPerHalfRevolution) - getOffset();
  }


  public void setAngle(double goal) {
    //long curr = System.currentTimeMillis();

    /*double diff = Math.abs(getAngle() - goal) % 2;
    if (diff > 1.0) {
      if (goal >= 0.0 && curr >= time_before_next_down) {
        offset_revs -= 1;
        time_before_next_down = curr + DEBOUNCE;
        time_before_next_up = 0;
      } else if (goal < 0.0 && curr >= time_before_next_up) {
        offset_revs += 1;
        time_before_next_up = curr + DEBOUNCE;
        time_before_next_down = 0;
      }
    }

    goal += getOffset();
    */
    set_encoder_count = goal * countPerHalfRevolution;
    angleMotor.set(ControlMode.Position, set_encoder_count);
    
  }

  /**
   * @return Returns raw encoder location
   */
  public double getEncoder() {
    return angleMotor.getSelectedSensorPosition();
  }

  /**
   * Sets the encoder postion
   * @param angle Expects an input between -1 and 1
   */
  public void setEncoder(double angle) {
    int steps = (int) (angle * countPerHalfRevolution);
    angleMotor.setSelectedSensorPosition(steps);
  }

  public void move(double speed, double angle) {
    setSpeed(speed);
    setAngle(angle);
  }

  /**
   * Re-calculate the offset based on the current absolute angle read from the
   * encoder. offset = floor(raw_angle / counts_per_rev) * one_rev
   */
  public void fixOffset() {
    /*double curr_rev = getEncoder() / countPerHalfRevolution;
    double rot_num = Math.floor(curr_rev) * 1.5;
    offset_revs = (int) rot_num;
    */
    offset_revs = 0;
  }

  public void updatePID() {
    angleMotor.config_kP(0, kP);
    angleMotor.config_kI(0, kI);
    angleMotor.config_kD(0, kD);
    angleMotor.config_kF(0, kF);
    angleMotor.config_IntegralZone(0, kIzone);
  }

  public double getDriveEncoder(){
    return driveMotor.getEncoder().getPosition();
  }
}
