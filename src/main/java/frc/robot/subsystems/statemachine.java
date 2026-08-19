package frc.robot.subsystems;

import frc.robot.Constants.Safety;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TestPositions;
import frc.robot.Constants.IntakeLiftingConstants;
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
    private IntakeLiftingConstants.IntakePositions intakeState;

    private int testArrayInd = 0; 
    private int testArrayLength = TestPositions.armTestTargetArray.length;

    private double armTarget = TestPositions.a51; 
    private double liftTarget = TestPositions.l51;
    
    private IntakeLiftingConstants.IntakePositions intakeTarget = IntakeLiftingConstants.IntakePositions.RETRACTED;

    private int stateTestOrderArrayLength = TestPositions.stateTestOrderArray.length;
    private int[] testValueAlt_arm = {0,0,0,0,0,0,0};
    private int[] testValueAlt_lift = {0,0,0,0,0,0,0};

    // make static basic building block commands
    private Command doNothing = runOnce(() -> {});
    private Command liftToLS;

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

    // the method that returns the correct command to move things based on the
    // current and desired state
    public Command moveSystemCommand() {
        updateState(); // first get the current state of the system
        int desiredState = calcState(armTarget, liftTarget); // get the desired state of the system

        Command returnCommand = runOnce(() -> {
        });

        System.out.println("current lift position: " + m_elevator.getElevatorPosition() + " in");
        System.out.println("current arm position: " + m_armlift.getArmPosition() + " deg");
        System.out.println("current lift + arm state: " + currentState);
        System.out.println("current intake state: " + intakeState);

        System.out.println("new lift position: " + liftTarget + " in");
        System.out.println("new arm position: " + armTarget + " deg");
        System.out.println("new lift + arm state: " + desiredState);
        System.out.println("new intake state: " + intakeTarget);

        if (intakeTarget == IntakeLiftingConstants.IntakePositions.RETRACTED
                & intakeState == IntakeLiftingConstants.IntakePositions.RETRACTED) { // intake up tp intake up
                                                                                     // transitions

            switch (currentState) {
                case 1:
                    switch (desiredState) {
                        case 1: // 1 to 1, intake up to up, do nothing tested on 8/15/26
                            returnCommand = doNothing;
                            break;
                        case 5: // 1 to 5, intake up to up, works, tested on 8/15/26
                            returnCommand = m_intakelift.deployIntakeCommand()
                                    .andThen(m_elevator.goToHeightCommand(ElevatorConstants.kLS))
                                    .andThen(m_armlift.goToAngleCommand(armTarget))
                                    .andThen(m_intakelift.retractIntakeCommand())
                                    .andThen(m_elevator.goToHeightCommand(liftTarget));
                            break;
                        case 7: // 1 to 7 transition tested on 8/15/26
                            returnCommand = m_intakelift.deployIntakeCommand()
                                    .andThen(m_elevator.goToHeightCommand(ElevatorConstants.kLS))
                                    .andThen(Commands.parallel(m_armlift.goToAngleCommand(armTarget),
                                            m_elevator.goToHeightCommand(liftTarget)))
                                    .andThen(m_intakelift.retractIntakeCommand());
                            break;
                        default:
                            System.out.println("Commanded position will self interfere. Doing nothing.");
                            break;
                    }
                    break;
                case 5: 
                    switch (desiredState) {
                        case 1: // 5 to 1 transition tested on 8/15/26
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                    .andThen(Commands.parallel(m_armlift.goToBottomCommand(),
                                            m_intakelift.deployIntakeCommand()))
                                    .andThen(m_elevator.goToBottomCommand())
                                    .andThen(m_intakelift.retractIntakeCommand());
                            break;
                        case 5: // 5 to 5 transition tested on 8/15/26
                            returnCommand = Commands.parallel(m_armlift.goToAngleCommand(armTarget),
                                    m_elevator.goToHeightCommand(liftTarget));
                            break;
                        case 7: // 5 to 7 transition tested on 8/15/26
                            returnCommand = Commands.parallel(m_armlift.goToAngleCommand(armTarget),
                                    m_elevator.goToHeightCommand(liftTarget));
                            break;
                        default:
                            System.out.println("Commanded position will self interfere. Doing nothing.");
                            break;
                    }
                    break;
                case 7: 
                    switch (desiredState) {
                        case 1: // 7 to 1 transition tested on 8/15/26
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(ElevatorConstants.kLSU),
                                    m_armlift.goToBottomCommand())
                                    .andThen(m_intakelift.deployIntakeCommand())
                                    .andThen(m_elevator.goToBottomCommand())
                                    .andThen(m_intakelift.retractIntakeCommand());
                            break;
                        case 5: // 7 to 5 transition tested on 8/15/26
                            returnCommand = Commands.parallel(m_armlift.goToAngleCommand(armTarget),
                                    m_elevator.goToHeightCommand(liftTarget));
                            break;
                        case 7: // 7 to 7 transition tested on 8/15/26
                            returnCommand = Commands.parallel(m_armlift.goToAngleCommand(armTarget),
                                    m_elevator.goToHeightCommand(liftTarget));
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    System.out.println("Current position is self interfering. Doing nothing.");
                    break;
            }
        } else if (intakeTarget == IntakeLiftingConstants.IntakePositions.DEPLOYED
                & intakeState == IntakeLiftingConstants.IntakePositions.DEPLOYED) { // deployed to deployed transitions
            switch (currentState) {
                case 1:
                    switch (desiredState) {
                        case 1: // 1 to 1 transition
                            // return do nothing command by default
                            break;
                        case 2: // 1 to 2 transition
                            returnCommand = m_elevator.goToHeightCommand(liftTarget);
                            break;
                        case 3: // 1 to 3 transition
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                            m_armlift.goToAngleCommand(armTarget));
                            break;
                        case 4, 5: // 1 to 4 transition
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(m_armlift.goToAngleCommand(armTarget))
                                            .andThen(m_elevator.goToHeightCommand(liftTarget));
                            break;
                        case 6, 7: // 1 to 6 transition
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                        m_armlift.goToAngleCommand(armTarget)));
                            break;
                        default:
                            System.out.println("Commanded position will self interfere. Doing nothing.");
                            break;
                    }
                    break;
                case 2:
                    switch (desiredState) {
                        case 1: // 2 to 1 transition
                            returnCommand = m_elevator.goToBottomCommand();
                        break;
                        case 2: // 2 to 2 transition
                            returnCommand = m_elevator.goToHeightCommand(liftTarget);
                        break;
                        case 3: // 2 to 3 transition
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                        m_armlift.goToAngleCommand(armTarget));
                        break;
                        case 4, 5: // 2 to 4 transition
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(m_armlift.goToAngleCommand(armTarget))
                                            .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                        case 6, 7: // 2 to 6 transition
                            if (m_elevator.getElevatorPosition() > ElevatorConstants.kLS) { // if lift above safety
                                returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                  m_armlift.goToAngleCommand(armTarget));
                            } else { // if lift below safety
                                returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                  m_armlift.goToAngleCommand(armTarget)));
                            }
                        break;
                        default:
                        System.out.println("Commanded position will self interfere. Doing nothing.");
                        break;
                    }
                    break;
                case 3:
                    switch (desiredState) {
                        case 1: // 3 to 1 transition
                            returnCommand = Commands.parallel(m_elevator.goToBottomCommand(),
                                                            m_armlift.goToBottomCommand());
                        break;
                        case 2: // 3 to 2 transition
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                               m_armlift.goToBottomCommand());
                        break;
                        case 3: // 3 to 3 transition
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                m_armlift.goToAngleCommand(armTarget));
                        break;
                        case 4, 5: // 3 to 4 transition
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(ElevatorConstants.kLS),
                                                                m_armlift.goToBottomCommand())
                                                    .andThen(m_armlift.goToAngleCommand(armTarget))
                                                    .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                        case 6, 7: // 3 to 6 7 transition
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(ElevatorConstants.kLS),
                                                            m_armlift.goToBottomCommand())
                                                    .andThen(Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                                m_armlift.goToAngleCommand(armTarget)));
                        break;
                        default:
                        System.out.println("Commanded position will self interfere. Doing nothing.");
                        break;
                    }
                    break;
                case 4 , 5:
                   switch (desiredState) {
                        case 1: // 4 to 1
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(m_armlift.goToBottomCommand())
                                            .andThen(m_elevator.goToBottomCommand());
                        break;
                        case 2: // 4 to 2
                            if (liftTarget > ElevatorConstants.kLS) { // if lift target above safety
                                returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                                .andThen(Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                                m_armlift.goToBottomCommand()));
                            }else{ // if lift target below safety
                                returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                                .andThen(m_armlift.goToBottomCommand())
                                                .andThen(m_elevator.goToHeightCommand(liftTarget));
                            }
                        break;
                        case 3: // 4 to 3
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(m_armlift.goToAngleCommand(armTarget))
                                            .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                        case 4, 5: // 4 to 4
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                m_armlift.goToAngleCommand(armTarget));
                        break;
                        case 6, 7: // 4 to 6
                            returnCommand = m_elevator.goToHeightCommand(ElevatorConstants.kLS)
                                            .andThen(Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                m_armlift.goToAngleCommand(armTarget)));
                        break;
                        default:
                            System.out.println("Commanded position will self interfere. Doing nothing.");
                        break;
                    }
                    break;
                case 6:
                    switch (desiredState) {
                        case 1: // 6 to 1
                            returnCommand = m_armlift.goToBottomCommand()
                                            .andThen(m_elevator.goToBottomCommand());
                        break; 
                        case 2: // 6 to 2
                            returnCommand = m_armlift.goToBottomCommand()
                                            .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                        case 3: // 6 to 3
                            returnCommand = m_armlift.goToBottomCommand()
                                            .andThen(m_elevator.goToHeightCommand(liftTarget))
                                            .andThen(m_armlift.goToAngleCommand(armTarget));
                        break;
                        case 4, 5: // 6 to 4 5
                            returnCommand = m_armlift.goToAngleCommand(armTarget)
                                            .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                        case 6, 7: // 6 to 6 7
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                m_armlift.goToAngleCommand(armTarget));
                        break;
                        default:
                            System.out.println("Commanded position will self interfere. Doing nothing.");
                        break;
                    }
                    break;
               case 7:
                    switch (desiredState) {
                        case 1: // 7 to 1
                            returnCommand = m_armlift.goToBottomCommand()
                                            .andThen(m_elevator.goToBottomCommand());
                        break; 
                        case 2: // 7 to 2
                            returnCommand = m_armlift.goToBottomCommand()
                                            .andThen(m_elevator.goToHeightCommand(liftTarget));
                        break;
                        case 3: // 7 to 3
                            returnCommand = m_armlift.goToBottomCommand()
                                            .andThen(m_elevator.goToHeightCommand(liftTarget))
                                            .andThen(m_armlift.goToAngleCommand(armTarget));
                        break;
                        case 4, 5, 6, 7: // 7 to 4 5 6 7
                            returnCommand = Commands.parallel(m_elevator.goToHeightCommand(liftTarget),
                                                                m_armlift.goToAngleCommand(armTarget));
                        break;
                        default:
                            System.out.println("Commanded position will self interfere. Doing nothing.");
                        break;
                    }
                default:
                    System.out.println("Current position is self interfering. Doing nothing.");
                    break;
            }
        } else {
            System.out.println("Intake down commands are not written yet.");
        }

        returnCommand = returnCommand.finallyDo(() -> {
            System.out.println("Multi subsystem movement command completed.");
        });
        return returnCommand;

    }

    public Command testNextTransitionCommand (){
        // command such that when executed, tries to go to the next set of test positions specified in the test arrays
        return runOnce(() -> incrementTestArrayIndex())
                .andThen(() -> setTestTargets())
                .andThen(moveSystemCommandDefer());
    }

    public void setCommandedPositions(double armPos, double liftPos, IntakeLiftingConstants.IntakePositions intakePos){
        armTarget = armPos;
        liftTarget = liftPos;
        intakeTarget = intakePos;
    }

    public Command setCommandedPositionsCommand(double armPos, double liftPos, IntakeLiftingConstants.IntakePositions intakePos){
        return runOnce(() -> setCommandedPositions(armPos, liftPos, intakePos));
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
        if (armBottomedBool & !L1Bool){
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
        if (!m_intakelift.isIntakeDeployed()){
            intakeState = IntakeLiftingConstants.IntakePositions.RETRACTED;
        }else{
            intakeState = IntakeLiftingConstants.IntakePositions.DEPLOYED;
        }
    }

    public void incrementTestArrayIndex(){
        if (testArrayInd == testArrayLength - 1){// if at the end of array
            testArrayInd = 0; // loop back to 0 index
        } else { // if not at end of array
            testArrayInd ++; // increment index
        }
    }

    public void setTestTargets(){
        // this method sets the internal targets using the current test array index
        armTarget = TestPositions.armTestTargetArray[testArrayInd];
        liftTarget = TestPositions.liftTestTargetArray[testArrayInd];
        intakeTarget = TestPositions.intakeTestTargetArray[testArrayInd];
    }

    public Command testNextTransitionCommand2 (){
        // command such that when executed, tries to go to the next arm and lift state specified in the stateTestOrderArray
        // actual arm and lift targets will alternate between set 1 and 2 every time a new target needs to be set
        return runOnce(() -> incrementTestArrayIndex2())
                .andThen(() -> setTestTargets2())
                .andThen(moveSystemCommandDefer());
    }

    public void incrementTestArrayIndex2(){
        if (testArrayInd == stateTestOrderArrayLength - 1){// if at the end of array
            testArrayInd = 0; // loop back to 0 index
        } else { // if not at end of array
            testArrayInd ++; // increment index
        }
    }

    public void setTestTargets2(){
        // this method sets the internal targets using the current test array index
        if (testValueAlt_arm[TestPositions.stateTestOrderArray[testArrayInd]-1] == 0){
            switch (TestPositions.stateTestOrderArray[testArrayInd]){
                case 1 :
                    armTarget = TestPositions.a11;
                    break;
                case 2 :
                    armTarget = TestPositions.a21;
                    break;
                case 3 : 
                    armTarget = TestPositions.a31;
                    break;
                case 4 :
                    armTarget = TestPositions.a41;
                    break;
                case 5 :
                    armTarget = TestPositions.a51;
                    break;
                case 6 :
                    armTarget = TestPositions.a61;
                    break;
                case 7 :
                    armTarget = TestPositions.a71;
                    break;
            }
            testValueAlt_arm[TestPositions.stateTestOrderArray[testArrayInd]-1] = 1;
        }else{
            switch (TestPositions.stateTestOrderArray[testArrayInd]){
                case 1 :
                    armTarget = TestPositions.a12;
                    break;
                case 2 :
                    armTarget = TestPositions.a22;
                    break;
                case 3 : 
                    armTarget = TestPositions.a32;
                    break;
                case 4 :
                    armTarget = TestPositions.a42;
                    break;
                case 5 :
                    armTarget = TestPositions.a52;
                    break;
                case 6 :
                    armTarget = TestPositions.a62;
                    break;
                case 7 :
                    armTarget = TestPositions.a72;
                    break;
            }
            testValueAlt_arm[TestPositions.stateTestOrderArray[testArrayInd]-1] = 0;
        }
        if (testValueAlt_lift[TestPositions.stateTestOrderArray[testArrayInd]-1] == 0){
            switch (TestPositions.stateTestOrderArray[testArrayInd]){
                case 1 :
                    liftTarget = TestPositions.l11;
                    break;
                case 2 :
                    liftTarget = TestPositions.l21;
                    break;
                case 3 : 
                    liftTarget = TestPositions.l31;
                    break;
                case 4 :
                    liftTarget = TestPositions.l41;
                    break;
                case 5 :
                    liftTarget = TestPositions.l51;
                    break;
                case 6 :
                    liftTarget = TestPositions.l61;
                    break;
                case 7 :
                    liftTarget = TestPositions.l71;
                    break;
            }
            testValueAlt_lift[TestPositions.stateTestOrderArray[testArrayInd]-1] = 1;
        }else{
            switch (TestPositions.stateTestOrderArray[testArrayInd]){
                case 1 :
                    liftTarget = TestPositions.l12;
                    break;
                case 2 :
                    liftTarget = TestPositions.l22;
                    break;
                case 3 : 
                    liftTarget = TestPositions.l32;
                    break;
                case 4 :
                    liftTarget = TestPositions.l42;
                    break;
                case 5 :
                    liftTarget = TestPositions.l52;
                    break;
                case 6 :
                    liftTarget = TestPositions.l62;
                    break;
                case 7 :
                    liftTarget = TestPositions.l72;
                    break;
            }
            testValueAlt_lift[TestPositions.stateTestOrderArray[testArrayInd]-1] = 0;
        }
        intakeTarget = IntakeLiftingConstants.IntakePositions.DEPLOYED; // for now test all deployed transitions
    }

    public void periodic() {
        //updateState();
        //System.out.println("current state: " + currentState);
    }

}

                //    switch (desiredState) {
                //         case 1:
                //         break;
                //         case 2:
                //         break;
                //         case 3:
                //         break;
                //         case 4:
                //         break;
                //         case 5:
                //         break;
                //         case 6:
                //         break;
                //         case 7:
                //         break;
                //         default:
                //         break;
                //     }