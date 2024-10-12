package frc.robot;

import java.security.InvalidAlgorithmParameterException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// thoughts on data filtering:
// - rolling window
// - bins
// - everything forever - running totals

public class OpenLoopInator {
    private MotorController m;
    private double ref_velocity;
    private DoubleSupplier velocity_provider, voltage_provider;
    private OffsetLinear negative_model, positive_model;
    private RingBuffer<Double> negative_voltage, negative_velocity, positive_voltage, positive_velocity;

    public OpenLoopInator(MotorController m, DoubleSupplier velocity_provider, DoubleSupplier voltage_provider, double initial_ff) {
        this(m, velocity_provider, voltage_provider, initial_ff, (int) ((1/Robot.kDefaultPeriod) * 60 * 2.5));
    }

    public OpenLoopInator(MotorController m, DoubleSupplier velocity_provider, DoubleSupplier voltage_provider, double initial_ff, int memory_depth) {
        this.m = m;
        this.velocity_provider = velocity_provider;
        this.voltage_provider  = voltage_provider;

        this.negative_voltage  = new RingBuffer<Double>(memory_depth);
        this.negative_velocity = new RingBuffer<Double>(memory_depth);
        this.positive_voltage  = new RingBuffer<Double>(memory_depth);
        this.positive_velocity = new RingBuffer<Double>(memory_depth);

        // seed the models
        // leave counters at 0:
        // first real data will result in (data, ff) which is better than (0, ff) since we are highly confident that (0, 0) is not correct
        // second real data will result in (data, data), and from there we're golden...well...unless...
        this.negative_voltage .add( 0.0);
        this.negative_voltage .add(-initial_ff);
        this.positive_voltage .add( 0.0);
        this.positive_voltage .add( initial_ff);
        this.negative_velocity.add( 0.0);
        this.negative_velocity.add(-1.0);
        this.positive_velocity.add( 0.0);
        this.positive_velocity.add( 1.0);

        try {
            this.negative_model = new OffsetLinear(this.negative_voltage, this.negative_velocity);
            this.positive_model = new OffsetLinear(this.positive_voltage, this.positive_velocity);
        } catch (InvalidAlgorithmParameterException e) {
            e.printStackTrace();
        }
    }

    public void set(double velocity) {
        setReference(velocity);
    }

    public void setReference(double velocity) {
        this.ref_velocity = velocity;
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

    public double getError() {
        return getReference() - getState();
    }

    private void updateMotor(double velocity) {
        double voltage = 0;
        if (velocity > 0) voltage = positive_model.apply(velocity);
        else if (velocity < 0) voltage = negative_model.apply(velocity);
        if (-1 > voltage) voltage = -1;
        if (voltage > 1) voltage = 1;
        m.setVoltage(voltage);
    }

    private void handleNewData(double voltage, double velocity) {
        if (velocity > 0) {
            positive_voltage.add(voltage);
            positive_velocity.add(velocity);
            try {
                this.positive_model = new OffsetLinear(positive_voltage, positive_velocity);
                //System.out.println(positive_model.toString());
            } catch (InvalidAlgorithmParameterException e) {
                e.printStackTrace();
            }
        }
        else if (velocity < 0) {
            negative_voltage.add(voltage);
            negative_velocity.add(velocity);
            try {
                this.negative_model = new OffsetLinear(negative_voltage, negative_velocity);
                //System.out.println(negative_model.toString());
            } catch (InvalidAlgorithmParameterException e) {
                e.printStackTrace();
            }
        }
    }

    public void onLoop() {
        handleNewData(voltage_provider.getAsDouble(), velocity_provider.getAsDouble());
        updateMotor(ref_velocity);
    }

    public String toString() {
        return String.format(
            "Positive model: %s from %d points with error %.3f"
            , positive_model
            , positive_velocity.size()
            , positive_model.r2()
        );
    }

    public String dump() {
        return positive_model.dump();
    }
}