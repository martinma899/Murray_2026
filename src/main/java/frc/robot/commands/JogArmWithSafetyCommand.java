// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.armlift;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.intakelift;
import frc.robot.subsystems.elevator2;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class JogArmWithSafetyCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final armlift m_armlift;
  private final intakelift m_intakelift;
  private final elevator2 m_elevator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JogArmWithSafetyCommand(double speed, armlift m_armlift, elevator2 m_elevator, intakelift m_intakelift) {
    this.m_armlift = m_armlift;
    this.m_elevator = m_elevator;
    this.m_intakelift = m_intakelift;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakelift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // only the initialize -> runOnce(lambda) 
  // only do the execute -> run(lambda)
  // run continuously until a condition -> run(lambda).until(lambda);
}
