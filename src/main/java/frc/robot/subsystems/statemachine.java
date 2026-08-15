package frc.robot.subsystems;

import frc.robot.Constants.Safety;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.armlift;
import frc.robot.subsystems.intakelift;
import frc.robot.subsystems.elevator2;
import frc.robot.SafetyUtilities;


public class statemachine extends SubsystemBase {
    // state machine class that keeps track of combined system position transitions
    // determines which state region is the robot in
    // returns proxy command to move the robot to the desired positions safely

    private final elevator2 m_elevator; 
    private final armlift m_armlift; 
    private final intakelift m_intakelift;

    private int currentState;

    public statemachine(elevator2 elevatorin, armlift armliftin, intakelift intakeliftin) {
        // pass in the object references
        m_elevator = elevatorin; 
        m_armlift = armliftin;
        m_intakelift = intakeliftin;
    }

    public void calcRegion (){
        // calculate which region the elevator and arm are in
        double armPosition = m_armlift.getArmPosition();
        double liftPosition = m_elevator.getElevatorPosition();
        boolean L1Bool = SafetyUtilities.isL1(armPosition,liftPosition);
        boolean L2Bool = SafetyUtilities.isL2(armPosition,liftPosition);
        boolean L3Bool = SafetyUtilities.isL3(armPosition,liftPosition);
        boolean L4Bool = SafetyUtilities.isL4(liftPosition);
        //boolean intakeUpBool = !m_intakelift.isIntakeDeployed();
        boolean liftBottomedBool = m_elevator.isLiftBottomed();
        boolean armBottomedBool = m_armlift.isArmBottomed();

        // check for state 1
        if (liftBottomedBool & armBottomedBool){
            currentState = 1; 
            return; 
        }

        // check for state 2
        if (armBottomedBool & !L3Bool){
            currentState = 2; 
            return; 
        }

        // check for state 8
        if (!L2Bool & !L3Bool){
            currentState = 8; 
            return; 
        }

        // check for state 3
        if (!L4Bool & L2Bool){
            currentState = 3; 
            return; 
        }

        // check for state 4
        if (L3Bool & !L4Bool & !L1Bool){
            currentState = 4; 
            return; 
        }

        // check for state 5
        if (L1Bool & !L4Bool){
            currentState = 5; 
            return; 
        }

        // check for state 6 and 7
        if (!L1Bool){
            currentState = 6; 
        }
        else{
            currentState = 7; 
        }
        

    }

    public void periodic() {
        calcRegion();
        System.out.println("current state: " + currentState);
    }

}
