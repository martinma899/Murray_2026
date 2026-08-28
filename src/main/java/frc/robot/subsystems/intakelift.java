// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeLiftingConstants;

public class intakelift extends SubsystemBase {

  private final SparkMax m_leftMotor; 
  private final SparkMax m_rightMotor; 

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private double intakePosition; // degree, average of the two encoders

  /** Creates a new ExampleSubsystem. */
  public intakelift() {

    m_leftMotor = new SparkMax(IntakeLiftingConstants.kLeftMotorCanID,MotorType.kBrushless);
    m_rightMotor = new SparkMax(IntakeLiftingConstants.kRightMotorCanID,MotorType.kBrushless);


    SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig(); 
    SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig(); 
    
    leftSparkMaxConfig.apply(IntakeLiftingConstants.kIntakeLiftConfig); 
    rightSparkMaxConfig.apply(IntakeLiftingConstants.kIntakeLiftConfig); 

    leftSparkMaxConfig.inverted(IntakeLiftingConstants.kLeftMotorInverted);
    rightSparkMaxConfig.inverted(IntakeLiftingConstants.kRightMotorInverted);

    rightSparkMaxConfig.follow(m_leftMotor,true);

    m_leftMotor.configure(leftSparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightSparkMaxConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);

  }

  public Command simpleMotorSpeedControlCommand(double speed) {
    // a simple command to set a motor speed
    return runOnce(() -> setMotorSpeed(speed));
  }

   /** 
   * returns a command to deploy the intake 
   */
  public Command deployIntakeCommand(){
    return run(() -> setMotorSpeed(IntakeLiftingConstants.kIntakeDeployDutyCycle))
    .until(() -> isIntakeDeployed())
    .withTimeout(3.0)
    .andThen(runOnce(() -> stopMotor()));
  }

  // public Command deployIntakeCommand2(){
  //   return run(() -> setMotorSpeed(IntakeLiftingConstants.kIntakeDeployDutyCycle))
  //   .until(() -> isIntakeDeployed())
  //   .withTimeout(1)
  //   .andThen(runOnce(() -> stopMotor())); 
  // }

   /** 
   * returns a command to retract the intake 
   */
  public Command retractIntakeCommand(){
    return run(() -> setMotorSpeed(IntakeLiftingConstants.kIntakeRetractDutyCycle))
    .until(() -> isIntakeRetracted())
    .withTimeout(3.0)
    .andThen(run(() -> {}).withTimeout(0.25))
    .andThen(runOnce(() -> stopMotor()));
  }

   /** 
   * returns a continuous background check command to stop motors if the motors are left stalled,
   * and if so, stop them
   */
  public Command defaultCurrentSafetyCheck(){
    return run(() -> {if (isMotorStalled()) {stopMotor();}});
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public boolean isIntakeDeployed(){
    intakePosition = getIntakeLiftPosition();
    return intakePosition >= IntakeLiftingConstants.kIntakeDeployedThreshold; 
  }

  public boolean isIntakeRetracted(){
    intakePosition = getIntakeLiftPosition();
    return intakePosition <= IntakeLiftingConstants.kIntakeRetractedThreshold; 
  }

   /**
   * Query of whether either the left or the right motor 
   * is statically stalled below an RPM and beyond a current value. 
   *
   * @return true if motor is stalled
   */
  public boolean isMotorStalled(){
    // current threshold 10, velocity threshold 100 rpm
    boolean bool = false;
    if (((Math.abs(m_leftMotor.getOutputCurrent())>= 10) 
    & Math.abs(m_leftEncoder.getVelocity()) <= 100)
    | (Math.abs(m_rightMotor.getOutputCurrent()) >= 10
    & Math.abs(m_rightEncoder.getVelocity()) <= 100)) {
      bool = true; 
    }
    return bool; 
  }

  /** 
   * set intake lift motor speed. Unit = duty cycle 
   */
  public void setMotorSpeed(double speed) {
    m_leftMotor.set(speed);
  }

  /** 
   * stop intake motors 
   */
  public void stopMotor() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

    /** 
   * zero both encoders 
   */
  public void zeroEncoders(){
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
  }

  /**
   * gets the intake position as the average of the two encoders
   */
  public double getIntakeLiftPosition(){
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
