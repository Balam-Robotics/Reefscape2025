// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
  /** Creates a new TestSubsystem. */
  public TestSubsystem() {
    new PrintCommand("Subsystem created");
    System.out.println("Subsystem created");
  }

  public void runCommandTest() {
    System.out.print("Run Command Test");
    new PrintCommand("sss");
  }

  public void startCommandTest() {
    System.out.println("Start Command Test");
  }

  public void endCommandTest() {
    System.out.println("End Command Test");
  }

  public void paralelCommadnTest() {
    System.out.println("Paralel Command Test");
  }

  public void paralel2CommadnTest() {
    System.out.println("Paralel 2 Command Test");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
