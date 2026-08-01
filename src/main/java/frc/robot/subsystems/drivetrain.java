// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DrivetrainConstants;

public class drivetrain extends SubsystemBase {

private final SparkMax m_leftFrontMotor;
private final SparkMax m_rightFrontMotor;
private final SparkMax m_leftRearMotor;
private final SparkMax m_rightRearMotor;

private final RelativeEncoder m_leftFrontEncoder;
private final RelativeEncoder m_rightFrontEncoder;
private final RelativeEncoder m_leftRearEncoder;
private final RelativeEncoder m_rightRearEncoder;

private final SparkClosedLoopController m_leftFrontController;
private final SparkClosedLoopController m_rightFrontController;
private final SparkClosedLoopController m_leftRearController;
private final SparkClosedLoopController m_rightRearController;

// advantage scope logging
private DoubleLogEntry m_leftFrontMotorLog;

  /** Creates a new ExampleSubsystem. */
  public drivetrain() {

    // instantiate motor controllers with CAN ID
    m_leftFrontMotor = new SparkMax(DrivetrainConstants.kLeftFrontMotorCANID, MotorType.kBrushless);
    m_rightFrontMotor = new SparkMax(DrivetrainConstants.kRightFrontMotorCANID, MotorType.kBrushless);
    m_leftRearMotor = new SparkMax(DrivetrainConstants.kLeftRearMotorCANID, MotorType.kBrushless);
    m_rightRearMotor = new SparkMax(DrivetrainConstants.kRightRearMotorCANID, MotorType.kBrushless);

    // apply motor controller configurations from constants
    m_leftFrontMotor.configure(DrivetrainConstants.kDriveConfigLeftFront,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFrontMotor.configure(DrivetrainConstants.kDriveConfigRightFront,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftRearMotor.configure(DrivetrainConstants.kDriveConfigLeftRear,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightRearMotor.configure(DrivetrainConstants.kDriveConfigRightRear,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // instantiate primary drive encoders
    m_leftFrontEncoder = m_leftFrontMotor.getEncoder();
    m_rightFrontEncoder = m_rightFrontMotor.getEncoder();
    m_leftRearEncoder = m_leftRearMotor.getEncoder();
    m_rightRearEncoder = m_rightRearMotor.getEncoder();

    // instantiate closed loop controllers
    m_leftFrontController = m_leftFrontMotor.getClosedLoopController();
    m_rightFrontController = m_rightFrontMotor.getClosedLoopController();
    m_leftRearController = m_leftRearMotor.getClosedLoopController();
    m_rightRearController = m_rightRearMotor.getClosedLoopController();

    // instantiate logs
    DataLog log = DataLogManager.getLog();
    m_leftFrontMotorLog = new DoubleLogEntry(log, "/leftFrontDriveMotor");


  }
    

  /**
   * driving with sticks command.
   *
   * @return a command
   */
  public Command driveWithStick(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return run(() -> setDesiredSpeedsFromStick(x.getAsDouble(),y.getAsDouble(),rot.getAsDouble()));
  }

  public void setDesiredSpeedsFromStick(double x, double y, double rot){
    double leftFrontSpeed = DrivetrainConstants.kChassisFreeSpeed * 
    (x * DrivetrainConstants.kTranslationMaxSpeed - y * DrivetrainConstants.kSideMaxSpeed  - rot * DrivetrainConstants.kRotationMaxSpeed); // in/s
    double rightFrontSpeed = DrivetrainConstants.kChassisFreeSpeed * 
    (x * DrivetrainConstants.kTranslationMaxSpeed + y * DrivetrainConstants.kSideMaxSpeed  + rot * DrivetrainConstants.kRotationMaxSpeed); // in/s
    double leftRearSpeed = DrivetrainConstants.kChassisFreeSpeed * 
    (x * DrivetrainConstants.kTranslationMaxSpeed + y * DrivetrainConstants.kSideMaxSpeed  - rot * DrivetrainConstants.kRotationMaxSpeed); // in/s
    double rightRearSpeed = DrivetrainConstants.kChassisFreeSpeed * 
    (x * DrivetrainConstants.kTranslationMaxSpeed - y * DrivetrainConstants.kSideMaxSpeed  + rot * DrivetrainConstants.kRotationMaxSpeed); // in/s
    setDesiredSpeeds(leftFrontSpeed, rightFrontSpeed, leftRearSpeed, rightRearSpeed);
  }

  public void setDesiredSpeeds(double leftFrontSpeed, double rightFrontSpeed, double leftRearSpeed, double rightRearSpeed){
    m_leftFrontController.setSetpoint(leftFrontSpeed, ControlType.kMAXMotionVelocityControl);
    m_rightFrontController.setSetpoint(rightFrontSpeed, ControlType.kMAXMotionVelocityControl);
    m_leftRearController.setSetpoint(leftRearSpeed, ControlType.kMAXMotionVelocityControl);
    m_rightRearController.setSetpoint(rightRearSpeed, ControlType.kMAXMotionVelocityControl);
  }

  public Command exampleMethodCommand(){
    return runOnce(() -> {});
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

    //m_leftFrontMotorLog.append(m_leftFrontController.getMAXMotionSetpointVelocity());
    //m_leftFrontMotorLog.append(3.0);
    //m_leftFrontMotorLog.append(m_leftFrontEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
