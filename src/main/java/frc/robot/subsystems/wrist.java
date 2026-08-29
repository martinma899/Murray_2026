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
// import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.WristConstants;

public class wrist extends SubsystemBase {

    private final SparkMax m_wristMotor;
    private final RelativeEncoder m_wristEncoder;

    /** Creates a new ExampleSubsystem. */
    public wrist() {
        m_wristMotor = new SparkMax(WristConstants.kWristMotorCanID, MotorType.kBrushless);
        m_wristMotor.configure(WristConstants.kWristConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_wristEncoder = m_wristMotor.getEncoder();
        m_wristEncoder.setPosition(0.0);
    }

    public Command simpleMotorSpeedControlCommand(double speed) {
        // a simple command to set a motor speed
        return runOnce(() -> setMotorSpeed(speed));
    }

    /**
     * returns a command to deploy the intake
     */
    public Command moveWristVerticalCommand() {
        return run(() -> setMotorSpeed(WristConstants.kWristDutyCycle))
                .until(() -> isWristVertical())
                .withTimeout(2.0)
                //.andThen(run(() -> {
                //}).withTimeout(0.25))
                .andThen(runOnce(() -> stopMotor()));
    }

    /**
     * returns a command to retract the intake
     */
    public Command moveWristHorizontalCommand() {
        return run(() -> setMotorSpeed(-1.0 * WristConstants.kWristDutyCycle))
                .until(() -> isWristHorizontal())
                .withTimeout(2.0)
                //.andThen(run(() -> {
                //}).withTimeout(0.25))
                .andThen(runOnce(() -> stopMotor()));
    }

    /**
     * returns a continuous background check command to stop motors if the motors
     * are left stalled,
     * and if so, stop them
     */
    public Command defaultCurrentSafetyCheck() {
        return run(() -> {
            if (isMotorStalled()) {
                stopMotor();
            }
        });
    }

    public boolean isWristVertical() {
        return getWristPosition() >= WristConstants.kWristVerticalThreshold;
    }

    public boolean isWristHorizontal() {
        return getWristPosition() <= WristConstants.kWristHorizontalThreshold;
    }

    /**
     * Query of whether either the left or the right motor
     * is statically stalled below an RPM and beyond a current value.
     *
     * @return true if motor is stalled
     */
    public boolean isMotorStalled() {
        // current threshold 10, velocity threshold 100 rpm
        boolean bool = false;
        if ((Math.abs(m_wristMotor.getOutputCurrent()) >= 10)
                & Math.abs(m_wristEncoder.getVelocity()) <= 100) {
            bool = true;
        }
        return bool;
    }

    /**
     * set intake lift motor speed. Unit = duty cycle
     */
    public void setMotorSpeed(double speed) {
        m_wristMotor.set(speed);
    }

    /**
     * stop intake motors
     */
    public void stopMotor() {
        m_wristMotor.stopMotor();
    }

    /**
     * zero both encoders
     */
    public void zeroEncoder() {
        m_wristEncoder.setPosition(0.0);
    }

    /**
     * gets the intake position as the average of the two encoders
     */
    public double getWristPosition() {
        return m_wristEncoder.getPosition();
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