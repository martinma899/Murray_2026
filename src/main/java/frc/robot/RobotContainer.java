// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.intakelift;
import frc.robot.subsystems.intakeroller;
import frc.robot.subsystems.elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.urcl.URCL;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.config.SignalsConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final drivetrain m_drivetrain = new drivetrain();
  private final intakelift m_intakelift = new intakelift();
  private final intakeroller m_intakeroller = new intakeroller(); 
  private final elevator m_elevator = new elevator();

  //private final Command m_simpleLiftIntakeCommand = m_intakelift.simpleMotorSpeedControlCommand(-0.5);
  //private final Command m_simpleDeployIntakeCommand = m_intakelift.simpleMotorSpeedControlCommand(0.2);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // start data log
    DataLogManager.start();
    URCL.start(); 

    // Configure the trigger bindings
    configureBindings();
    
    // set the driving with stick command as the default drivetrain command 
    Command driveCommand = m_drivetrain.driveWithStick(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(),OperatorConstants.kDriveDeadband) * -1.0, 
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(),OperatorConstants.kDriveDeadband) * -1.0,
        () -> MathUtil.applyDeadband(m_driverController.getRightX(),OperatorConstants.kDriveDeadband) * -1.0);
    m_drivetrain.setDefaultCommand(driveCommand);

    // set the stall safety check command as the default command
    m_intakelift.setDefaultCommand(m_intakelift.defaultCurrentSafetyCheck());

    // set the stall safey check command as the default command
    m_intakeroller.setDefaultCommand(m_intakeroller.defaultCurrentSafetyCheck());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule intake deploy and retract using y and a buttons,
    m_driverController.y().onTrue(m_intakelift.deployIntakeCommand());
    m_driverController.a().onTrue(m_intakelift.retractIntakeCommand());

    // test setting encoder command
    m_driverController.b().onTrue(Commands.runOnce(m_intakelift::zeroEncoders));

    //m_driverController.x().onTrue(m_intakelift.simpleMotorSpeedControlCommand(-0.5));

    // intake roller manual control commands and logic
    Trigger intakeDown = new Trigger (m_intakelift::isIntakeDeployed);
    Trigger intakeUp = new Trigger (m_intakelift::isIntakeRetracted);
    m_driverController.rightBumper().and(intakeDown).onTrue(m_intakeroller.turnOnIntakeCommand());
    m_driverController.rightTrigger().onTrue(m_intakeroller.turnOffIntakeCommand());
    intakeUp.onTrue(m_intakeroller.turnOffIntakeCommand());

    m_driverController.leftBumper().onTrue(m_elevator.setHeight(Inches.of(12)));
    m_driverController.leftTrigger().onTrue(m_elevator.setHeight(Inches.of(2)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_drivetrain);
  }
}
