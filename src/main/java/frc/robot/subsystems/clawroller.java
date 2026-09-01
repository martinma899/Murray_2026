package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawRollerConstants;

public class clawroller extends SubsystemBase{

    private final SparkMax m_clawmotor; 
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private int proximity = 1000;

    private IntegerPublisher sensorReading;

    private int loopCounter = 0; 

    public clawroller() {
        m_clawmotor = new SparkMax(ClawRollerConstants.kClawRollerMotorCanID,MotorType.kBrushless);
        m_clawmotor.configure(ClawRollerConstants.kClawRollerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("MySubtable");
        sensorReading = table.getIntegerTopic("ClawColorSensorIRReading").publish();
    }

    public Command oneButtonWhileTrueCommand (){
        // first run the claw for 0.5s at intake speed
        // this ensures when coral is gripped incorrectly I can press intake button and correct without stalling motor
        // otherwise if coral is not in claw it acts like just turning on the claw
        return run(() -> setClawMotorSpeed(ClawRollerConstants.kClawRollerInwardSpeed))
        .withTimeout(0.2)
        .andThen(
        run(()-> {
            if (isCoralInClaw()) {
                setClawMotorSpeed(ClawRollerConstants.kClawRollerGripSpeed);
            }else{
                setClawMotorSpeed(ClawRollerConstants.kClawRollerInwardSpeed);
            }
        }))
        .finallyDo(()-> {
            stopClawMotor();
        });
    }

    public Command turnOffRollerCommand (){
        return runOnce(()-> {
            stopClawMotor();
        });
    }

    public Command releaseCoralCommand (){
        return run(() -> setClawMotorSpeed(ClawRollerConstants.kClawRollerReleaseSpeed))
            .withTimeout(ClawRollerConstants.kClawRollerReleaseTime)
            .andThen(runOnce(() -> stopClawMotor() ));
    }

    // method to set a claw motor speed
    public void setClawMotorSpeed(double speed){
        m_clawmotor.set(speed);
    }

    // method to stop claw motor
    public void stopClawMotor(){
        m_clawmotor.stopMotor();
    }

    // check if coral is in claw by checking if the sensor distance value is below a threshold
    public boolean isCoralInClaw(){
        //proximity = m_colorSensor.getProximity();
        return proximity > ClawRollerConstants.kCoralInClawThreshold;
    }

    public boolean isCoralNotInClaw(){
        return !isCoralInClaw();
    }

    @Override
    public void periodic (){


        loopCounter++;
  
        // Only query the I2C device once every 25 loops (Approx. once per 0.5 second)
        if (loopCounter >= 25) {
            loopCounter = 0;
            proximity = m_colorSensor.getProximity();
            sensorReading.set(proximity);
        }

    }

    
}
