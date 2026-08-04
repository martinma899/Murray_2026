// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }

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

  public static class IntakeLiftingConstants {
    public static int kLeftMotorCanID = 6;
    public static int kRightMotorCanID = 5; 

    public static boolean kLeftMotorInverted = false;
    public static boolean kRightMotorInverted = false;

    public static int kIntakeLiftMotorCurrentLimit = 30; // amp

    public static double kIntakeLiftGearRatio = 30.0 ; // gear down 
    public static double kPositionConversionFactor = 360.0 / kIntakeLiftGearRatio; // actuator degree / motor rotation

    public static SparkMaxConfig kIntakeLiftConfig = new SparkMaxConfig(); // common intake motor lift configuration

    static{
        kIntakeLiftConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(kIntakeLiftMotorCurrentLimit);
        kIntakeLiftConfig.encoder.positionConversionFactor(kPositionConversionFactor);
        kIntakeLiftConfig.signals.primaryEncoderPositionAlwaysOn(true);
    }

    public static double kIntakeDeployDutyCycle = 0.2; // constant duty cycle applied to deploy intake
    public static double kIntakeRetractDutyCycle = -0.3; // constant duty cycle applied to retract intake

    // these threshold are only used for safety and completion checks internal to the intakelift susystem itself
    public static double kIntakeDeployedThreshold = 84.0; // deg, threshold beyond which intake is recognized as deployed
    public static double kIntakeRetractedThreshold = 10; // deg, threshold beyond which intake is recognized as retracted
  }

  public static class IntakeRollerConstants {

    public static int kIntakeRollerMotorCanID = 7; 
    public static boolean kIntakeRollerMotorInverted = false; 
    public static int kIntakeRollerMotorCurrentLimit = 30; // amp

    public static double kIntakeRollerInwardSpeed = 1.0; // intake in speed
    public static double kIntakeRollerOutwardSpeed = -0.3; // intake reverse speed

    public static SparkMaxConfig kIntakeRollerConfig = new SparkMaxConfig();

    static{
      kIntakeRollerConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(kIntakeRollerMotorCurrentLimit);
      kIntakeRollerConfig.signals.primaryEncoderPositionAlwaysOn(true);
    }
  }

  public static class ElevatorConstants {

    public static int kMotorCanID = 8; 
    public static boolean kMotorInverted = true; 
    public static int kMotorCurrentLimit = 40; // amp

    public static double kSprocketPD = 2.149; // elevator drive sprocket PD, in
    public static double kSprocketCirc = kSprocketPD * Math.PI; // elevator drive sprocket circumference, in
    public static double kGearing = 27; // reduction

    public static double kP = 6; // V per inch error, need tuning
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double ks = 0; // static friction voltage gain
    public static double kg = 0.2632; // V to hold lift stationary
    public static double kv = 0.5073; // V per in/s
    public static double ka = 0; // 


    public static double kLowerLimitSoft = 0; // soft lower limit, in
    public static double kUpperLimitSoft = 50; // soft upper limit, in

    public static double kLowerLimitHard = 0; // soft lower limit, in
    public static double kUpperLimitHard = 52.5; // soft upper limit, in

    public static double kMaxVelocity = 12; // in/s
    public static double kMaxAccel = 115.8661; // in/s/s = 0.3g
    // public static SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

    // static{
    //   kElevatorConfig
    //   .idleMode(IdleMode.kCoast)
    //   .smartCurrentLimit(kElevatorMotorCurrentLimit);
    //   kElevatorConfig.signals.primaryEncoderPositionAlwaysOn(true);
    // }
  }


}
