package frc.robot;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestRobot extends TimedRobot {
    private OpenLoopInator fl, fr, rl, rr;
    private List<OpenLoopInator> os;
    private MecanumDrive m;

    private double velConFac = 1d/5.95;
    private double simGearRed = 1d; //5.95;
    private double maxSpeedRPM = 100d;

    private XboxController drv;

    private enum Mode {
        DRIVER_RAW
        , DRIVER_CART
        , DASHBOARD
        , SINE
    }

    private Mode mode = Mode.DRIVER_RAW;

    @Override
    public void robotInit() {

        drv = new XboxController(0);

        CANSparkMax
              flm = new CANSparkMax(1, MotorType.kBrushless)
            , frm = new CANSparkMax(2, MotorType.kBrushless)
            , rlm = new CANSparkMax(3, MotorType.kBrushless)
            , rrm = new CANSparkMax(4, MotorType.kBrushless)
            ;

        for (CANSparkMax m : List.of(flm, frm, rlm, rrm)) {
            m.getEncoder().setVelocityConversionFactor(velConFac);
            if (Robot.isSimulation()) REVPhysicsSim.getInstance().addSparkMax(m, DCMotor.getNEO(1).withReduction(simGearRed));
        }

        fl = new OpenLoopInator(flm, DCMotor.getNEO(1).withReduction(simGearRed)).setName("fl").setID(1);
        fr = new OpenLoopInator(frm, DCMotor.getNEO(1).withReduction(simGearRed)).setName("fr").setID(2);
        rl = new OpenLoopInator(rlm, DCMotor.getNEO(1).withReduction(simGearRed)).setName("rl").setID(3);
        rr = new OpenLoopInator(rrm, DCMotor.getNEO(1).withReduction(simGearRed)).setName("rr").setID(4);

        os = List.of(fl, fr, rl, rr);

        m = new MecanumDrive(
              fl.getDoubleConsumer()
            , rl.getDoubleConsumer()
            , fr.getDoubleConsumer()
            , rr.getDoubleConsumer()
            );

        SmartDashboard.putNumber("r_RPM", 0);
    }

    @Override
    public void robotPeriodic() {
        switch (mode) {
            case DRIVER_RAW:
                for (OpenLoopInator o : os) {
                    o.set(-1d * drv.getLeftY() * Units.rotationsPerMinuteToRadiansPerSecond(maxSpeedRPM));
                }
            case DASHBOARD:
                for (OpenLoopInator o : os) {
                    o.set(
                        Units.rotationsPerMinuteToRadiansPerSecond(
                            SmartDashboard.getNumber("r_RPM", 0)
                        )
                    );
                }
                break;
            case DRIVER_CART:
                double dx = -1d * drv.getLeftX() ;
                double dy = -1d * drv.getLeftY() ;
                double dz = -1d * drv.getRightX();
                m.driveCartesian(dy, dx, dz);
                break;
            case SINE:
                for (OpenLoopInator o : os) {
                    o.setReference(Units.rotationsPerMinuteToRadiansPerSecond(maxSpeedRPM) * Math.sin(Timer.getFPGATimestamp()));
                }
                break;
            default:
                for (OpenLoopInator o : os) {
                    o.setReference(0);
                }
                break;
        }
        for (OpenLoopInator o : os) {
            SmartDashboard.putNumber("r_" + o.getName(), o.getReference());
            SmartDashboard.putNumber("x_" + o.getName(), o.getState());
            SmartDashboard.putNumber("e_" + o.getName(), o.getError());
            SmartDashboard.putNumber("u_" + o.getName(), o.getInput());
        }
    }
}
