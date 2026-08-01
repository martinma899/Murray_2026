// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SignalsConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DrivetrainConstants {
    public static final int kLeftFrontMotorCANID = 2;
    public static final int kRightFrontMotorCANID = 3;
    public static final int kLeftRearMotorCANID = 4;
    public static final int kRightRearMotorCANID = 1;

    public static final boolean kLeftFrontMotorInverted = false;
    public static final boolean kRightFrontMotorInverted = true;
    public static final boolean kLeftRearMotorInverted = false;
    public static final boolean kRightRearMotorInverted = true;

    public static final int kDriveMotorCurrentLimit = 50; //amp

    public static final double kWheelDiameter = 6; // inch
    public static final double kWheelGearRatio = 10.71; 
    public static final double kMotorFreeSpeed = 5676; // rpm
    public static final double kChassisFreeSpeed = 166.4956107; // in/s
    public static final double kSpeedConversionFactor = 0.02933326474; // wheel in/s per motor rpm

    public static final double kTranslationMaxSpeed = 0.25; // 1 = 100%
    public static final double kSideMaxSpeed = 0.25; // 1 = 100%
    public static final double kRotationMaxSpeed = 0.25; // 1 = 100%

    //public static final double kp = 0.0005; // solo P controller gain
    //public static final double kp = 0.0068; // duty cycle per in/s 
    public static final double kp = 0.0068*0.8; // duty cycle per in/s 
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double kv = 12.0 / (kChassisFreeSpeed); // volt / (in/s)
    //public static final double kv = 0;
    //public static final double kMaxAccel = 193.1102362; //in/s/s, = 0.5g
    public static final double kMaxAccel = 386.2204724; //in/s/s, 1.0 g

    //public static final SparkMaxConfig kDriveConfigCommon = new SparkMaxConfig();
    public static final SparkMaxConfig kDriveConfigLeftFront = new SparkMaxConfig();
    public static final SparkMaxConfig kDriveConfigRightFront = new SparkMaxConfig();
    public static final SparkMaxConfig kDriveConfigLeftRear = new SparkMaxConfig();
    public static final SparkMaxConfig kDriveConfigRightRear = new SparkMaxConfig();
    
    static {
      kDriveConfigLeftFront
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kDriveMotorCurrentLimit)
          .inverted(kLeftFrontMotorInverted);
      kDriveConfigLeftFront.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kp, ki, kd)
          .outputRange(-1, 1)
          .feedForward.kV(kv);
      kDriveConfigLeftFront.closedLoop.maxMotion.maxAcceleration(kMaxAccel);
      kDriveConfigLeftFront.encoder.velocityConversionFactor(kSpeedConversionFactor);
      kDriveConfigLeftFront.signals
        .maxMotionSetpointVelocityAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true);

      kDriveConfigRightFront
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kDriveMotorCurrentLimit)
          .inverted(kRightFrontMotorInverted);
      kDriveConfigRightFront.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kp, ki, kd)
          .outputRange(-1, 1)
          .feedForward.kV(kv);
      kDriveConfigRightFront.closedLoop.maxMotion.maxAcceleration(kMaxAccel);
      kDriveConfigRightFront.encoder.velocityConversionFactor(kSpeedConversionFactor);
            kDriveConfigRightFront.signals
        .maxMotionSetpointVelocityAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true);
      
      kDriveConfigLeftRear
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kDriveMotorCurrentLimit)
          .inverted(kLeftRearMotorInverted);
      kDriveConfigLeftRear.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kp, ki, kd)
          .outputRange(-1, 1)
          .feedForward.kV(kv);
       kDriveConfigLeftRear.closedLoop.maxMotion.maxAcceleration(kMaxAccel);
       kDriveConfigLeftRear.encoder.velocityConversionFactor(kSpeedConversionFactor);
             kDriveConfigLeftRear.signals
        .maxMotionSetpointVelocityAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true);

      kDriveConfigRightRear
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kDriveMotorCurrentLimit)
          .inverted(kRightRearMotorInverted);
      kDriveConfigRightRear.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kp, ki, kd)
          .outputRange(-1, 1)
          .feedForward.kV(kv);   
      kDriveConfigRightRear.closedLoop.maxMotion.maxAcceleration(kMaxAccel);
      kDriveConfigRightRear.encoder.velocityConversionFactor(kSpeedConversionFactor);
            kDriveConfigRightRear.signals
        .maxMotionSetpointVelocityAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true);       
    }


  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }
}
