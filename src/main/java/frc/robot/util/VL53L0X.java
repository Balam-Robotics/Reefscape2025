// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 




        :::::::::      :::     :::            :::       :::   :::          ::::::::  ::::::::::  ::::::::  ::::::::::: 
     :+:    :+:   :+: :+:   :+:          :+: :+:    :+:+: :+:+:        :+:    :+: :+:    :+: :+:    :+: :+:     :+:  
    +:+    +:+  +:+   +:+  +:+         +:+   +:+  +:+ +:+:+ +:+              +:+ +:+              +:+         +:+    
   +#++:++#+  +#++:++#++: +#+        +#++:++#++: +#+  +:+  +#+           +#++:  +#++:++#+      +#+          +#+      
  +#+    +#+ +#+     +#+ +#+        +#+     +#+ +#+       +#+              +#+        +#+   +#+           +#+        
 #+#    #+# #+#     #+# #+#        #+#     #+# #+#       #+#       #+#    #+# #+#    #+#  #+#           #+#          
#########  ###     ### ########## ###     ### ###       ###        ########   ########  ##########     ###   
  




*/

/*
 * VL53L0X Time-of-Flight Distance Sensor
 * Custom library for VL53L0X sensor using I2C communication
 * Based on the sensor's datasheet and application notes
 * Author: BALAM 3527
 */

package frc.robot.util;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VL53L0X {
  
  private static final int ADDRESS = 0x29;
  private I2C i2c;
  
  private static final int SAMPLE_SIZE = 11;
  private final Queue<Double> samples = new LinkedList<>();

  private double filteredDistance = 0.0;
  private final double alpha;
  private double lastReadTime = 0.0;
  private final double readInterval = 0.02;

  public VL53L0X(I2C.Port port, double smoothingFactor) {
    i2c = new I2C(port, ADDRESS);
    this.alpha = smoothingFactor;
    init();
        SmartDashboard.putString("VL53L0X", "Initialized on " + port.toString());
  }

  private void writeReg(int reg, int value) {
    i2c.write(reg, value);
  }

  public double readWord(int reg) {
    byte[] buffer = new byte[2];
    i2c.read(reg, 2, buffer);
    return ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
  }

  private void init() {
    try {
        Thread.sleep(100);
        // Start continuous ranging mode
        writeReg(0x00, 0x02); // start measurement
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, 0x3C);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);
        Thread.sleep(10);
    } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
    }
  }

  public double readRawDistance() {
    byte[] buffer = new byte[2];
    boolean fail = i2c.read(0x14 + 10, 2, buffer); // Result register
    if (fail) return -1;
    int distance = ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
    return distance;
  }

  public double getDistance() {
    double val = readRawDistance();
    if (val <= 0) return getAverage();
    samples.add(val);
    if (samples.size() > SAMPLE_SIZE) samples.poll();
    return getAverage();
  }
  
  private double getAverage() {
    if (samples.isEmpty()) return 0;
    double sum = 0;
    for (double v : samples) sum += v;
    return sum / samples.size();
  }

  public double getFilteredDistance() {
    double now = Timer.getFPGATimestamp();

    if (now - lastReadTime >= readInterval) {
        double newDistance = readRawDistance();
        
        if (newDistance > 20 && newDistance < 2000) {
          filteredDistance = alpha * newDistance + (1 - alpha) * filteredDistance;
        }
        lastReadTime = now;
    }
    return filteredDistance;
  }

  public double getSmoothDistance() {
    double val = readRawDistance();

    if (val > 20 && val < 2000) {
      samples.add(val);
      if (samples.size() > SAMPLE_SIZE) samples.poll();
    }

    double sum = 0;
    for (double v : samples) sum += v;
    return sum / samples.size();
  }

}
