package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.ElevatorConstants;

public class elevator extends SubsystemBase 
{
    private final SparkMax m_elevatorSparkMax = new SparkMax(ElevatorConstants.kMotorCanID,MotorType.kBrushless);

    private final SmartMotorControllerConfig m_motorConfig = new SmartMotorControllerConfig(this)
    .withMechanismCircumference(Inches.of(ElevatorConstants.kSprocketCirc))
    .withClosedLoopController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
    .withTrapezoidalProfile(MetersPerSecond.of(ElevatorConstants.kMaxVelocity), MetersPerSecondPerSecond.of(ElevatorConstants.kMaxAccel))
    .withSoftLimits(Meters.of(ElevatorConstants.kLowerLimitSoft),Meters.of(ElevatorConstants.kUpperLimitSoft))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(ElevatorConstants.kGearing)))
    .withIdleMode(MotorMode.COAST)
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
    .withStatorCurrentLimit(Amps.of(ElevatorConstants.kMotorCurrentLimit))
    .withMotorInverted(ElevatorConstants.kMotorInverted)
    .withFeedforward(new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka))
    .withStartingPosition(Meters.of(0))
    .withControlMode(ControlMode.CLOSED_LOOP);
    //.withControlMode(ControlMode.OPEN_LOOP);

    private final SmartMotorController m_elevatorMotor = new SparkWrapper(m_elevatorSparkMax,DCMotor.getNEO(1),m_motorConfig);

    private ElevatorConfig m_config = new ElevatorConfig()
    .withHardLimits(Meters.of(ElevatorConstants.kLowerLimitHard),Meters.of(ElevatorConstants.kUpperLimitHard))
    .withTelemetry("Elevator", TelemetryVerbosity.HIGH);

    private final Elevator m_elevator  = new Elevator(m_config, m_elevatorMotor);

    private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
        voltage -> {m_elevatorMotor.setVoltage(voltage);},
        log -> {log.motor("elevator-motor")
                    .voltage(m_elevatorMotor.getVoltage())
                .linearPosition(m_elevator.getHeight())
            .linearVelocity(m_elevator.getVelocity());}, this));

    public elevator () {
        
    }

    public Command simpleSetDutyCycleCommand(double dutyCycle){
        return m_elevator.set(dutyCycle);
    }

    public Command setHeight (Distance Height){
        return m_elevator.setHeight(Height);
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
        m_elevator.updateTelemetry();
    }

    public void simulationPeriodic() {
        //m_elevator.simIterate();
    }

}
