// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Maldito extends Command {
  /** Creates a new Maldito. */
  TestSubsystem tonto;
  public Maldito(TestSubsystem tonto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tonto = tonto;
    addRequirements(tonto);
    System.out.println("Maldito");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialize");
    new PrintCommand("getName()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
