package frc.robot.subsystems;

import frc.robot.Constants.Safety;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TestPositions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.armlift;
import frc.robot.subsystems.intakelift;
import frc.robot.subsystems.elevator2;
import frc.robot.SafetyUtilities;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;



public class statemachine extends SubsystemBase {
    // state machine class that keeps track of combined system position transitions
    // determines which state region is the robot in
    // returns proxy command to move the robot to the desired positions safely

    private final elevator2 m_elevator; 
    private final armlift m_armlift; 
    private final intakelift m_intakelift;

    private int currentState;

    private double armTarget = TestPositions.a51; 
    private double liftTarget = TestPositions.l51;

    // make basic building block commands
    private Command doNothing = runOnce(() -> {});

    public statemachine(elevator2 elevatorin, armlift armliftin, intakelift intakeliftin) {
        // pass in the object references
        m_elevator = elevatorin; 
        m_armlift = armliftin;
        m_intakelift = intakeliftin;
    }

    // the main deferred command returning the correct command to move the things to the correct positions safely
    public Command moveSystemCommandDefer(){
        // for now assume intake is up
        return Commands.defer(
            () -> {return moveSystemCommand();},
            Set.of(m_elevator,m_armlift,m_intakelift));// update current state
            
    }

    // the method that returns the correct command to move things based on the current and desired state
    public Command moveSystemCommand(){
        updateState(); // first get the current state of the system
        int desiredState = calcState(armTarget, liftTarget); // get the desired state of the system

        Command returnCommand = runOnce(() -> {}); 

        System.out.println("current state: " + currentState);
        System.out.println("new state: " + desiredState);



        switch (currentState) {
            case 1:
                switch (desiredState) {
                    case 1:
                        returnCommand = runOnce(() -> {}); break; //1 to 1, intake up to up, do nothing
                        
                    case 5:
                        returnCommand = m_intakelift.deployIntakeCommand()
                                .andThen(m_elevator.goToHeightCommand(ElevatorConstants.kLS))
                                .andThen(m_armlift.goToAngleCommand(armTarget))
                                .andThen(m_intakelift.retractIntakeCommand())
                                .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                    case 7:
                        break;
                    default:
                        break;
                }
                break;
            case 5:
                switch (desiredState) {
                    case 1:
                        break;
                    case 5:
                        break;
                    case 7:
                        break;
                    default:
                        break;
                }
                break;
            case 7:
                switch (desiredState) {
                    case 1:
                        break;
                    case 5:
                        break;
                    case 7:
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }

        return returnCommand;
         
    }

    public int calcState (double armPos, double liftPos){
        // calculate which region the elevator and arm are in
        boolean L1Bool = SafetyUtilities.isL1(armPos,liftPos);
        boolean L2Bool = SafetyUtilities.isL2(armPos,liftPos);
        boolean L3Bool = SafetyUtilities.isL3(armPos,liftPos);
        boolean L4Bool = SafetyUtilities.isL4(liftPos);
        //boolean intakeUpBool = !m_intakelift.isIntakeDeployed();
        boolean liftBottomedBool = liftPos < ElevatorConstants.kL0T;
        boolean armBottomedBool = armPos < ArmConstants.kA0T;

        int state = 0; 

        // check for state 1
        if (liftBottomedBool & armBottomedBool){
            state = 1; 
            return state; 
        }

        // check for state 2
        if (armBottomedBool & !L3Bool){
            state = 2; 
            return state; 
        }

        // check for state 8
        if (!L2Bool & !L3Bool){
            state = 8; 
            return state; 
        }

        // check for state 3
        if (!L4Bool & L2Bool){
            state = 3; 
            return state; 
        }

        // check for state 4
        if (L3Bool & !L4Bool & !L1Bool){
            state = 4; 
            return state; 
        }

        // check for state 5
        if (L1Bool & !L4Bool){
            state = 5; 
            return state; 
        }

        // check for state 6 and 7
        if (!L1Bool){
            state = 6; 
        }
        else{
            state = 7; 
        }
        return state;
    }

    public void updateState(){
        currentState = calcState(m_armlift.getArmPosition(),m_elevator.getElevatorPosition());
    }
    public void periodic() {
        //updateState();
        //System.out.println("current state: " + currentState);
    }

}
