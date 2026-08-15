package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Safety;
import frc.robot.SafetyUtilities;
import yams.mechanisms.positional.Elevator;

public class elevator2 extends SubsystemBase 
{
    private final SparkMax m_elevatorSparkMax = new SparkMax(ElevatorConstants.kMotorCanID,MotorType.kBrushless);

    private final RelativeEncoder m_elevatorEncoder; 

    private final SparkClosedLoopController m_elevatorController;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Inches.mutable(0);
    private final MutLinearVelocity m_velocity = InchesPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
        voltage -> {m_elevatorSparkMax.setVoltage(voltage);},
        log -> {log.motor("elevator-motor")
                    .voltage(m_appliedVoltage.mut_replace(m_elevatorSparkMax.getBusVoltage()*m_elevatorSparkMax.getAppliedOutput(),Volts))
                .linearPosition(m_distance.mut_replace(getElevatorPosition(),Inches))
            .linearVelocity(m_velocity.mut_replace(getElevatorVelocity(),InchesPerSecond));}, this));

    public elevator2 () {
        m_elevatorSparkMax.configure(ElevatorConstants.kElevatorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_elevatorEncoder = m_elevatorSparkMax.getEncoder();

        m_elevatorController = m_elevatorSparkMax.getClosedLoopController();

        zeroEncoder();
    }

    public Command simpleSetDutyCycleCommand(double dutyCycle){
        return runOnce(() -> m_elevatorSparkMax.set(dutyCycle));
    }

    public Command stopMotorCommand(){
        return runOnce(() -> stopMotor());
    }

    public Command setHeightCommand (double Height){
        return runOnce(() -> setCommandedPosition(Height));
    }

    public Command goToHeightCommand (double Height){
        return runOnce(() -> setCommandedPosition(Height))
        .andThen(new WaitUntilCommand(() -> isTargetReached()))
        .withTimeout(5.0);
    }

    public Command goToBottomCommand(){
        return runOnce(() -> setCommandedPosition(0.0))
            .andThen(new WaitUntilCommand(() -> isLiftBottomed()))
            .andThen(runOnce(() -> stopMotor()));
    }

    public Command increaseSetHeightCommand (double Height){
        return runOnce(() -> setCommandedPosition(m_elevatorController.getSetpoint() + Height));
    }

    public Command jogWithSafetyCommand(double speed, armlift m_armlift, intakelift m_intakelift){
        return run(() -> setMotorSpeed(speed))
        .until(() -> 
        (speed < 0 & isLiftBottomed())
        |(!m_intakelift.isIntakeDeployed() 
          & m_armlift.isArmBottomed() 
          & isLiftBottomed())
        |(!m_intakelift.isIntakeDeployed() 
          & SafetyUtilities.isInterfereIntakeUp(m_armlift.getArmPosition(),getElevatorPosition())
          & speed < 0)
        |(m_intakelift.isIntakeDeployed()
          & SafetyUtilities.isInterfereIntakeDown(m_armlift.getArmPosition(),getElevatorPosition())
          & m_armlift.getArmPosition() <= Safety.kArmLimitBehaviorChangePoint
          & speed >= 0)
        |(m_intakelift.isIntakeDeployed()
          & SafetyUtilities.isInterfereIntakeDown(m_armlift.getArmPosition(),getElevatorPosition())
          & m_armlift.getArmPosition() > Safety.kArmLimitBehaviorChangePoint
          & speed < 0))
        .finallyDo(() -> {
            stopMotor();
            if (!isLiftBottomed()){
                setCommandedPosition(getElevatorPosition());
            }
        });
    // 1. lift wants to go down and is bottomed
    // 2. intake up, arm bottomed, lift bottomed
    // 3. intake up, interfering, lift not bottomed, lift wants down
    // 4. intake down, interfering, arm below behavior change point, lift wants up
    // 5. intake down, interfering, arm above behavior change point, lift wants down
    }

    public void setMotorSpeed(double dutyCycle){
        m_elevatorSparkMax.set(dutyCycle);
    }

    public void stopMotor(){
        m_elevatorSparkMax.stopMotor();
    }

    public void setEncoderPosition(double position){
        m_elevatorEncoder.setPosition(position);
    }

    public void setCommandedPosition(double position){
        m_elevatorController.setSetpoint(position,SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void zeroEncoder(){
        m_elevatorEncoder.setPosition(0);
    }

    public double getElevatorPosition(){
        return m_elevatorEncoder.getPosition();
    }

    public double getElevatorVelocity(){
        return m_elevatorEncoder.getVelocity();
    }

    public boolean isLiftBottomed(){
        double pos = getElevatorPosition();
        return pos <= ElevatorConstants.kL0T;
    }

    public boolean isTargetReached(){
        //System.out.println(m_elevatorController.getSetpoint());
        boolean bool = Math.abs(getElevatorPosition()-m_elevatorController.getSetpoint()) 
        <= ElevatorConstants.kAllowedCommandEndError;
        //System.out.println(bool);
        return bool;
    }


    /**
   * Run sysId on the {@link Elevator}
   */
    //public Command sysId() { 
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
    }

    public void simulationPeriodic() {
        //m_elevator.simIterate();
    }

}
