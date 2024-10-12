// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private CANSparkMax fl = new CANSparkMax(1, MotorType.kBrushless);
  private OpenLoopInator flo = new OpenLoopInator(
    fl
    , fl.getEncoder()::getVelocity
    , () -> {return fl.getAppliedOutput() * fl.getBusVoltage();}
    , 0.00211
    , (int) (1/this.getPeriod()) * 3 // 3 seconds
    );

  @Override
  public void robotInit() {
    //SmartDashboard.putNumber("fl_r_in", 0);
  }

  double refVelSweep = 0;
  double refVelIncrement = 1;
  double refVelUpper = 500;
  double refVelLower = 0;
  @Override
  public void robotPeriodic() {
    //flo.setReference(SmartDashboard.getNumber("fl_r_in", 0));
    if (refVelSweep > refVelUpper || refVelSweep < refVelLower) refVelIncrement *= -1;
    if (refVelSweep > refVelUpper) {
      System.out.println(flo.dump());
      throw new Error();
    }
    flo.setReference(refVelSweep += refVelIncrement);
    flo.onLoop();
    SmartDashboard.putNumber("fl_u", flo.getInput());
    SmartDashboard.putNumber("fl_x", flo.getState());
    SmartDashboard.putNumber("fl_r", flo.getReference());
    SmartDashboard.putNumber("fl_e", flo.getError());
    if (refVelSweep % 100 == 0)
    System.out.println(flo);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
