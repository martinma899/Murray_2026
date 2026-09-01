// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeLiftingConstants;
import frc.robot.Constants.IntakeRollerConstants;

public class intakeroller extends SubsystemBase {
  
  private final SparkMax m_rollerMotor; 

  private final RelativeEncoder m_rollerMotorEncoder; 
  
  private final Timer m_stallRecoveryTimer = new Timer(); 

  private boolean stallMonitorBool = false; // true when a stall is detected and being monitored
  
  public intakeroller() {
    m_rollerMotor = new SparkMax(IntakeRollerConstants.kIntakeRollerMotorCanID,MotorType.kBrushless);
    m_rollerMotor.configure(IntakeRollerConstants.kIntakeRollerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rollerMotorEncoder = m_rollerMotor.getEncoder();
  }

  /**
   * Command to turn on the intake 
   *
   * The command waits until the motor speed is at least 100 rpm before handing it over to the default stall check command
   */
  public Command turnOnIntakeCommand() {
    return run(() -> setMotorSpeed(IntakeRollerConstants.kIntakeRollerInwardSpeed))
    .until(() -> m_rollerMotorEncoder.getVelocity() >= 100)
    .withTimeout(1.0);
  }

  public Command turnOffIntakeCommand() {
    return runOnce(() -> stopIntakeMotor());
  }

  public Command oneButtonWhileTrueCommand() {
    return run(() -> setMotorSpeed(IntakeRollerConstants.kIntakeRollerInwardSpeed))
    .finallyDo(this::stopIntakeMotor);
  }

  public Command spitObjectOutCommand() {
    return run(() -> setMotorSpeed(IntakeRollerConstants.kIntakeRollerOutwardSpeed))
    .withTimeout(2.0)
    .andThen(turnOffIntakeCommand());
  }

  public Command defaultCurrentSafetyCheck(){
    return run(() -> {
      if (isMotorStalled() & !stallMonitorBool){ // if motor is stalled first time
        m_stallRecoveryTimer.restart(); // start timer to time how long the motor is stalled
        stallMonitorBool = true; 
      }
      if (stallMonitorBool & m_stallRecoveryTimer.get() >= 2.0) { // if motor is currently stalled and at 2 second continuous stall
        stopIntakeMotor(); // stop motor and reset boolean
        stallMonitorBool =false;
      }
      if (stallMonitorBool & !isMotorStalled()){ // if during a stall, the motor is unstalled before 2 seconds
        stallMonitorBool = false; // stop monitoring motor
      }
    });
  }

  public void setMotorSpeed(double speed){
    m_rollerMotor.set(speed);
  }

  public void stopIntakeMotor(){
    m_rollerMotor.stopMotor();
  }

  public boolean isMotorStalled(){
    boolean bool = false; 
    if (((Math.abs(m_rollerMotor.getOutputCurrent())>= 20) 
    & Math.abs(m_rollerMotorEncoder.getVelocity()) < 100)) {
      bool = true; 
    }
    return bool; 
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
