// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand.isFinished()) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.tejuino_board.rainbow_effect(m_robotContainer.tejuino_board.LED_STRIP_1);
    //m_robotContainer.tejuino_board.escuderia_effect(m_robotContainer.tejuino_board.LED_STRIP_0);
  }

  int loopCounter = 0;
  /*
  @Override
  public void teleopPeriodic() {

    // all_led_control

    if (loopCounter < 50) {
      m_robotContainer.tejuino_board.all_leds_green(m_robotContainer.tejuino_board.LED_STRIP_1);
    } else if (loopCounter < 100) {
      m_robotContainer.tejuino_board.all_leds_white(m_robotContainer.tejuino_board.LED_STRIP_1);
    } else if (loopCounter < 150) {
      m_robotContainer.tejuino_board.all_leds_red(m_robotContainer.tejuino_board.LED_STRIP_1);
    } else {
      loopCounter = 0; // Reset the counter to loop the flag colors
    }

    loopCounter++;
  } */

  // total LEDs
private static final int NUM_LEDS = 16;
// strip reference;

// animation state
private int offset = 0;        // where the flag starts
private static final int STEP_DELAY = 5; // how many cycles to wait before moving
 @Override
 public void teleopPeriodic() {
  loopCounter++;

  if (loopCounter >= STEP_DELAY) {
      loopCounter = 0;      // reset counter
      offset = (offset + 1) % NUM_LEDS; // advance animation
  }

  for (int i = 0; i < NUM_LEDS; i++) {
       // rotate index
       int pos = (i + offset) % NUM_LEDS;

       // divide circle into 3 equal parts (5, 5, 6)
       int section;
       if (pos < 5) section = 0;          // Green
       else if (pos < 10) section = 1;    // White
       else section = 2;                  // Red

       int r=0, g=0, b=0;
       switch (section) {
           case 0: g = 255; break;                       // Green
           case 1: r = 255; g = 255; b = 255; break;     // White
           case 2: r = 255; break;                       // Red
       }

      m_robotContainer.tejuino_board.single_led_control(m_robotContainer.tejuino_board.LED_STRIP_0, i, r, g, b);
  }
}

  

  @Override
  public void teleopExit() {
    m_robotContainer.tejuino_board.all_leds_blue(m_robotContainer.tejuino_board.LED_STRIP_1);}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
