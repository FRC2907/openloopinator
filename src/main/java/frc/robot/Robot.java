// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private CANSparkMax A, B;
  private OpenLoopInator Ao;

  @Override
  public void robotInit() {
    A = new CANSparkMax(1, MotorType.kBrushless);
    B = new CANSparkMax(2, MotorType.kBrushless);
    Ao = new OpenLoopInator(A, DCMotor.getNEO(1), 0.1);
    //Ao = new OpenLoopInator(fr, DCMotor.getNEO(1), 1d);
    REVPhysicsSim.getInstance().addSparkMax(A, DCMotor.getNEO(1));//.withReduction(5.95));
    REVPhysicsSim.getInstance().addSparkMax(B, DCMotor.getNEO(1));//.withReduction(5.95));

    SmartDashboard.putNumber("A_0_r_in", 0);
    SmartDashboard.putNumber("B_0_u_in", 0);
  }

  int T = 0;
  @Override
  public void robotPeriodic() {
    T += 1;
    // ramp
    //Ao.setReference(Math.min(10*T, 550));
    // sawtooth
    //Ao.setReference(10*T % 1000 - 500);
    // sine
    Ao.setReference((500 * Math.sin((double)T / 100.0)));
    // manual
    //Ao.setReference(SmartDashboard.getNumber("A_0_r_in", 0));
    Ao.onLoop();
    if (T % 100 == 1)
    System.out.println(Ao);
    SmartDashboard.putNumber("A_0_x_out", Ao.getDryRun(SmartDashboard.getNumber("A_0_r_in", 0)));

    double B_volts = SmartDashboard.getNumber("B_0_u_in", 0);
    B.setVoltage(B_volts / (Robot.isReal() ? 1d : B.getBusVoltage()));
    SmartDashboard.putNumber("B_1_x_radps", Units.rotationsPerMinuteToRadiansPerSecond(B.getEncoder().getVelocity()));
    SmartDashboard.putNumber("B_3_u_out_raw", B.getAppliedOutput());
    SmartDashboard.putNumber("B_4_u_out_bussin", B.getAppliedOutput() * B.getBusVoltage());

    SmartDashboard.putNumber("A_1_r", Ao.getReference());
    SmartDashboard.putNumber("A_2_x", Ao.getState());
    SmartDashboard.putNumber("A_3_e", Ao.getError());
    SmartDashboard.putNumber("A_4_u", Ao.getInput());
  }
}