// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;

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

    public static double kIntakeDeployDutyCycle = 0.15; // constant duty cycle applied to deploy intake
    public static double kIntakeRetractDutyCycle = -0.2; // constant duty cycle applied to retract intake

    // these threshold are only used for safety and completion checks internal to the intakelift susystem itself
    public static double kIntakeDeployedThreshold = 84.0; // deg, threshold beyond which intake is recognized as deployed
    public static double kIntakeRetractedThreshold = 10; // deg, threshold beyond which intake is recognized as retracted
  }

  public static class IntakeRollerConstants {

    public static int kIntakeRollerMotorCanID = 7; 
    public static boolean kIntakeRollerMotorInverted = false; 
    public static int kIntakeRollerMotorCurrentLimit = 30; // amp

    public static double kIntakeRollerInwardSpeed = 0.4; // intake in speed
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

    //public static double kSprocketPD = 2.149; // elevator drive sprocket PD, in
    public static double kSprocketCirc = 6.3; // elevator drive sprocket circumference, in
    public static double kGearing = 25; // reduction

    public static double kSpeedConversionFactor = 0.0042; // in/s per motor rpm
    public static double kPositionConversionFactor = 0.252; // in per motor rotation

    public static double kP = 0.07; // V per in error, need tuning
    public static double kI = 0.00005; // V per in error integral
    // public static double kD = 0.5;
    //public static double kI = 0.0002; // V per in error integral
    public static double kD = 0.0000;

    // kP 0.07
    // kI 0.0002
    // kD 10

    public static double ks = 0.17076; // static friction voltage gain
    public static double kg = 0.41614; // V to hold lift stationary
    public static double kv = 0.495173; // V per in/s
    public static double ka = 0.0010922; // V per in/s/s


    public static double kLowerLimitSoft = 0; // soft lower limit, in
    public static double kUpperLimitSoft = 50; // soft upper limit, in

    public static double kLowerLimitHard = 0; // soft lower limit, in
    public static double kUpperLimitHard = 52.5; // soft upper limit, in

    public static double kMaxVelocity = 23.8392; // in/s
    public static double kCruiseVelocity = 15; // in/s
    public static double kMaxAccel = 386.2204724*0.05; // in/s/s = 1g
    
    public static double kAllowedCommandEndError = 0.1; // in, ends command when target is within this tolerance

    public static SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

    static{
      kElevatorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(kMotorCurrentLimit)
      .inverted(kMotorInverted);

      kElevatorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kP, kI, kD)
          .allowedClosedLoopError(0,ClosedLoopSlot.kSlot0)
          .outputRange(-1, 1)
          .iZone(1);
        kElevatorConfig.closedLoop.feedForward
        .kV(kv)
        .kS(ks)
        .kA(ka)
        .kG(kg);
       kElevatorConfig.closedLoop.maxMotion
       .maxAcceleration(kMaxAccel)
       .cruiseVelocity(kCruiseVelocity)
       .allowedProfileError(12);
       kElevatorConfig.encoder.velocityConversionFactor(kSpeedConversionFactor)
       .positionConversionFactor(kPositionConversionFactor);

      kElevatorConfig.signals.primaryEncoderPositionAlwaysOn(true);
    }
  }

   public static class ArmConstants {

    public static int kMotorCanID = 11; 
    public static boolean kMotorInverted = true; 
    public static int kMotorCurrentLimit = 40; // amp

    public static double kGearing = 120; // reduction

    public static double kSpeedConversionFactor = 0.05; // deg/s per motor rpm
    public static double kPositionConversionFactor = 3.0; // deg per motor rotation
    public static double kCosRatio = 1.0/360.0; // mechanism rotation per degree
  

    public static double kP = 0.01; // V per deg error, need tuning
    //public static double kP = 0.0; // V per deg error, need tuning
    public static double kI = 0.00002; // V per deg error integral
    //public static double kI = 0.0; // V per deg error integral
    public static double kD = 0.000; // V per deg/s error

    public static double ks = 0.24892*0.7; // static friction voltage gain
    //public static double ks = 0; // static friction voltage gain
    //public static double kg = 0.36249; // V to hold lift stationary
    public static double kv = 0.03817777778; // V per deg/s
    public static double ka = 0.089/360.0; // V per deg/s/s
    public static double kcos = 0.36249; // V per rotation
    //public static double kcos = 0.0; // V per rotation

    public static double kStartingPosition = -82.28069095; // deg, set such that 0 = CG horizontal, from CAD measurement and calculation
    //public static double kStartingPosition = 0; // deg, set such that 0 = CG horizontal, from CAD measurement and calculation

    public static double kLowerLimitSoft = kStartingPosition; // soft lower limit, deg
    public static double kUpperLimitSoft = 10.0; // soft upper limit, deg

    public static double kLowerLimitHard = kStartingPosition; // soft lower limit, deg
    public static double kUpperLimitHard = 10.0; // soft upper limit, deg

    public static double kMaxVelocity = 283.8; // deg/s
    public static double kCruiseVelocity = 180; // deg/s
    public static double kMaxAccel = 180; // deg/s/s = 2s to 180 deg/s
    
    public static double kAllowedCommandEndError = 1; // deg, ends command when target is within this tolerance

    public static SparkMaxConfig kArmMotorConfig = new SparkMaxConfig();

    static{
      kArmMotorConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(kMotorCurrentLimit)
      .inverted(kMotorInverted);

      kArmMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(kP, kI, kD)
          .allowedClosedLoopError(0,ClosedLoopSlot.kSlot0)
          .outputRange(-1, 1)
          .iZone(10);
        kArmMotorConfig.closedLoop.feedForward
        .kV(kv,ClosedLoopSlot.kSlot0)
        .kS(ks,ClosedLoopSlot.kSlot0)
        .kA(ka,ClosedLoopSlot.kSlot0)
        .kG(0,ClosedLoopSlot.kSlot0)
        .kCos(kcos,ClosedLoopSlot.kSlot0)
        .kCosRatio(kCosRatio,ClosedLoopSlot.kSlot0);
       kArmMotorConfig.closedLoop.maxMotion
       .maxAcceleration(kMaxAccel)
       .cruiseVelocity(kCruiseVelocity)
       .allowedProfileError(90);
       kArmMotorConfig.encoder.velocityConversionFactor(kSpeedConversionFactor)
       .positionConversionFactor(kPositionConversionFactor);

      kArmMotorConfig.signals.primaryEncoderPositionAlwaysOn(true);
    }
  }
  public static class Safety{
    // multi-subsystem safety checks

    // safety boundary lines 
    // arm angle = x axis, elevator height = y axis
    // arm angle: bottom = -82.28069095, unit = deg
    // elevator height: bottom = 0, unit = in
    // lines are y = mx + b
    
    // intake up boundary line
    public static double m1 = -0.256295005; 
    public static double b1 = -3.588130064;
    // intake down boundary line 1
    public static double m2 = 0.65;
    public static double b2 = 47.125;
    // intake down boundary line 2
    public static double m3 = -0.282608696;
    public static double b3 = -11.16304348;

    // this is the threshold that changes the arm limiting behavior when intake is down. 
    // if arm is below this angle then it is recognized as being inside the downed intake and can only go down. 
    // if arm is above this angle then it is recognized as being outside the downed intake and can only go up. 
    public static double kArmLimitBehaviorChangePoint = -62.5; 
  }

}
