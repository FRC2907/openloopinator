// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MinRobot extends TimedRobot {

  private OpenLoopInator Ao;

  @Override
  public void robotInit() {
    // need this (1/2)
    CANSparkMax m = new CANSparkMax(1, MotorType.kBrushless);
    m.getEncoder().setVelocityConversionFactor(1d/5.95);
    Ao = new OpenLoopInator(m, DCMotor.getNEO(1));

    if (Robot.isSimulation()) REVPhysicsSim.getInstance().addSparkMax(m, DCMotor.getNEO(1));//.withReduction(5.95));
  }

  @Override
  public void robotPeriodic() {
    double T = 500d * Math.sin(Timer.getFPGATimestamp());

    // need this (2/2)
    Ao.setReference(T);
    Ao.onLoop();

    SmartDashboard.putNumber("r", Ao.getReference());
    SmartDashboard.putNumber("x", Ao.getState());
  }

}