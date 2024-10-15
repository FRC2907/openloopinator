package frc.robot;

import java.util.HashMap;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// thoughts on data filtering:
// - rolling window
// - bins
// - everything forever - running totals
// - bins but only use a smaller subset of bins around the commanded velocity?
//     retains information but reduces weight of irrelevant portions of the graph
//     that may not be updated under new conditions

public class OpenLoopInator {
    private MotorController m;
    private double ref_velocity, ref_voltage, freeSpeedRadPerSec, nominalVoltage, resolution;
    private double seed_A, seed_B;
    private DoubleSupplier voltage_provider, velocity_provider;
    private LinearRegression positive_model, negative_model;
    private HashMap<Double, Pair<Double, Double>> positive_data, negative_data;

    private String name = "";
    private int id = 0;

    private static final double DEFAULT_RESOLUTION = 0.1; // V

    public OpenLoopInator(CANSparkMax ctlr, DCMotor motor) {
        this(ctlr, motor, DEFAULT_RESOLUTION);
    }
    public OpenLoopInator(CANSparkMax ctlr, DCMotor motor, double resolution) {
        this(
          (MotorController) ctlr
        , () -> {return (double) (ctlr.getAppliedOutput() * ctlr.getBusVoltage());}
        , () -> {return Units.rotationsPerMinuteToRadiansPerSecond(ctlr.getEncoder().getVelocity());}
        , motor.freeSpeedRadPerSec
        , motor.nominalVoltageVolts
        , resolution
        );
    }

    public OpenLoopInator(MotorController ctlr, DoubleSupplier voltage_provider, DoubleSupplier velocity_provider, double freeSpeedRadPerSec) {
        this(ctlr, voltage_provider, velocity_provider, freeSpeedRadPerSec, 12.0, DEFAULT_RESOLUTION);
    }
    public OpenLoopInator(MotorController ctlr, DoubleSupplier voltage_provider, DoubleSupplier velocity_provider, double freeSpeedRadPerSec, double nominalVoltage) {
        this(ctlr, voltage_provider, velocity_provider, freeSpeedRadPerSec, nominalVoltage, DEFAULT_RESOLUTION);
    }
    public OpenLoopInator(MotorController ctlr, DoubleSupplier voltage_provider, DoubleSupplier velocity_provider, double freeSpeedRadPerSec, double nominalVoltage, double resolution) {
        this.m = ctlr;
        this.freeSpeedRadPerSec = freeSpeedRadPerSec;
        this.nominalVoltage = nominalVoltage;
        this.resolution = resolution;
        this.voltage_provider  = voltage_provider;
        this.velocity_provider = velocity_provider;

        this.positive_data = new HashMap<>();
        this.negative_data = new HashMap<>();

        // seed the models

        this.seed_A = 100d*this.nominalVoltage;
        this.seed_B = 101d*this.nominalVoltage;
        this.positive_data.put(this.seed_A, new Pair<>( this.freeSpeedRadPerSec                    , this.nominalVoltage));
        this.positive_data.put(this.seed_B, new Pair<>( this.freeSpeedRadPerSec/this.nominalVoltage, 1d));
        this.negative_data.put(this.seed_A, new Pair<>(-this.freeSpeedRadPerSec                    , -this.nominalVoltage));
        this.negative_data.put(this.seed_B, new Pair<>(-this.freeSpeedRadPerSec/this.nominalVoltage, -1d));
        this.positive_model = new LinearRegression(this.positive_data.values());
        this.negative_model = new LinearRegression(this.negative_data.values());
    }

    public void set(double velocityRadPerSec) {
        setReference(velocityRadPerSec);
    }

    public double getDryRun(double velocity) {
        double voltage = 0;
        if (velocity == 0.0) voltage = 0;
        else {
            LinearRegression working_model = velocity > 0 ? positive_model : negative_model;
            voltage = working_model.apply(velocity);
        }
        return voltage;
    }

    public void setReference(double velocityRadPerSec) {
        this.ref_velocity = velocityRadPerSec;
    }

    public double getReference() {
        return ref_velocity;
    }

    public double getState() {
        return velocity_provider.getAsDouble();
    }

    public double getInput() {
        return voltage_provider.getAsDouble();
    }

    public double getRefVoltage() {
        return ref_voltage;
    }

    public double getError() {
        return getReference() - getState();
    }

    public MotorController getMotorController() {
        return m;
    }

    public double getFreeSpeedRadPerSec() {
        return freeSpeedRadPerSec;
    }

    private void updateMotor(double velocity) {
        ref_voltage = getDryRun(velocity);
        m.setVoltage(ref_voltage / (Robot.isReal() ? 1d : 12d)); // bug in REV sim treats setVoltage as a duty cycle
    }

    private void handleNewData(double voltage, double velocity) {
        if (Math.abs(velocity) < 1e1) return; // too smol, noisy
        if (Math.abs(voltage) < 1e-2) return; // too smol, noisy
        if (Math.abs(voltage) > nominalVoltage) return; // bogus
        if (Math.signum(voltage) != Math.signum(velocity)) return; // there may be cases where a positive voltage results in a negative velocity...we'll burn that bridge when we get to it
        HashMap<Double, Pair<Double, Double>> working_data = velocity > 0 ? positive_data : negative_data;
        double bin = Math.round(voltage/resolution) * resolution;
        System.out.println(String.format("Putting value %.2f rad/s, %.4f V in bin for %.2f V", velocity, voltage, bin));
        working_data.put(bin, new Pair<>(velocity, voltage));
        if (velocity > 0)
            positive_model = new LinearRegression(positive_data.values());
        else
            negative_model = new LinearRegression(negative_data.values());
    }

    private void attemptUnseed() {
        if (positive_data.size() > 5 && positive_data.containsKey(seed_A)) positive_data.remove(seed_A);
        if (positive_data.size() > 4 && positive_data.containsKey(seed_B)) positive_data.remove(seed_B);
        if (negative_data.size() > 5 && negative_data.containsKey(seed_A)) negative_data.remove(seed_A);
        if (negative_data.size() > 4 && negative_data.containsKey(seed_B)) negative_data.remove(seed_B);
    }

    public void onLoop() {
        handleNewData(voltage_provider.getAsDouble(), velocity_provider.getAsDouble());
        attemptUnseed();
        updateMotor(ref_velocity);
    }

    public String toString() {
        return String.format(
            "Positive model: %s from %d points with error %.3f\n"
            + "Negative model: %s from %d points with error %.3f"
            , positive_model
            , positive_data.size()
            , positive_model.getError()
            , negative_model
            , negative_data.size()
            , negative_model.getError()
        );
    }

    public String dump() {
        return "Positive data:\n" + positive_model.dump() + "\nNegative data:\n" + negative_model.dump();
    }

    // convenience

    public DoubleConsumer getDoubleConsumer(double arbitraryMaxSpeedRadPerSec) {
        return x -> this.setReference(x * arbitraryMaxSpeedRadPerSec);
    }

    public DoubleConsumer getDoubleConsumer() {
        return x -> this.setReference(x * freeSpeedRadPerSec);
    }

    public OpenLoopInator setName(String name) {
        this.name = name;
        return this;
    }

    public String getName() {
        return name;
    }

    public OpenLoopInator setID(int id) {
        this.id = id;
        return this;
    }

    public int getID() {
        return id;
    }
}