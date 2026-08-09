package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Safety;
import frc.robot.SafetyUtilities;
import frc.robot.subsystems.elevator2;
import frc.robot.subsystems.intakelift;

public class armlift extends SubsystemBase 
{
    private final SparkMax m_armSparkMax = new SparkMax(ArmConstants.kMotorCanID,MotorType.kBrushless);

    private final RelativeEncoder m_armEncoder; 

    private final SparkClosedLoopController m_armController;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);

    private final MutAngle m_distance = Degrees.mutable(0);

    private final MutAngularVelocity m_velocity = DegreesPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
        voltage -> {m_armSparkMax.setVoltage(voltage);},
        log -> {log.motor("arm-motor")
                    .voltage(m_appliedVoltage.mut_replace(m_armSparkMax.getBusVoltage()*m_armSparkMax.getAppliedOutput(),Volts))
                .angularPosition(m_distance.mut_replace(getArmPosition(),Degrees))
            .angularVelocity(m_velocity.mut_replace(getArmVelocity(),DegreesPerSecond));}, this));

    public armlift () {
        m_armSparkMax.configure(ArmConstants.kArmMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armEncoder = m_armSparkMax.getEncoder();

        m_armController = m_armSparkMax.getClosedLoopController();

        zeroEncoder();
    }

    public Command simpleSetDutyCycleCommand(double dutyCycle){
        return runOnce(() -> m_armSparkMax.set(dutyCycle));
    }

    public Command stopMotorCommand(){
        return runOnce(() -> stopMotor());
    }

    public Command setAngleCommand (double Angle){
        return runOnce(() -> setCommandedPosition(Angle));
    }

    public Command goToAngleCommand (double Angle){
        return runOnce(() -> setCommandedPosition(Angle))
        .andThen(new WaitUntilCommand(() -> isTargetReached()))
        .withTimeout(5.0);
    }

    public Command goToBottomCommand(){
        return runOnce(() -> setCommandedPosition(ArmConstants.kStartingPosition))
            .andThen(new WaitUntilCommand(() -> isArmBottomed()))
            .withTimeout(5.0)
            .andThen(runOnce(() -> stopMotor()));
    }

    public Command increaseSetAngleCommand (double Angle){
        return runOnce(() -> setCommandedPosition(m_armEncoder.getPosition() + Angle));
    }

    public Command jogWithSafetyCommand (double speed, elevator2 m_elevator, intakelift m_intakelift){
        return run(() -> setMotorSpeed(speed))
        .until(
            () -> (m_intakelift.isIntakeDeployed() & SafetyUtilities.isInterfereIntakeDown(getArmPosition(),m_elevator.getElevatorPosition()))
                | (!m_intakelift.isIntakeDeployed() & SafetyUtilities.isInterfereIntakeUp(getArmPosition(),m_elevator.getElevatorPosition()))
                | isArmBottomed())
        .finallyDo(() -> {
            if (isArmBottomed()) {
                stopMotor();}
            else{
                setCommandedPosition(getArmPosition());}});
    }

    public void setMotorSpeed(double dutyCycle){
        m_armSparkMax.set(dutyCycle);
    }

    public void stopMotor(){
        m_armSparkMax.stopMotor();
    }

    public void setEncoderPosition(double position){
        m_armEncoder.setPosition(position);
    }

    public void setCommandedPosition(double position){
        m_armController.setSetpoint(position,SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void zeroEncoder(){
        m_armEncoder.setPosition(ArmConstants.kStartingPosition);
    }

    public double getArmPosition(){
        return m_armEncoder.getPosition();
    }

    public double getArmVelocity(){
        return m_armEncoder.getVelocity();
    }

    public boolean isArmBottomed(){
        double pos = getArmPosition();
        return pos <= ArmConstants.kLowerLimitHard + 3.0;
    }

    public boolean isTargetReached(){
        return Math.abs(getArmPosition()-m_armController.getSetpoint()) 
        <= ArmConstants.kAllowedCommandEndError;
    }


    /**
   * Run sysId on the {@link Elevator}
   */
    // public Command sysId() { 
    //    return m_elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    // }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public void periodic() {
        //m_elevator.updateTelemetry();
        //m_armController.get
    }

    public void simulationPeriodic() {
        //m_elevator.simIterate();
    }



}
