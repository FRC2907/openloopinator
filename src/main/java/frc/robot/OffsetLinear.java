package frc.robot;

import java.security.InvalidAlgorithmParameterException;
import java.util.List;
import java.util.function.Function;

public class OffsetLinear implements Function<Double, Double> {

    private double slope = 0, intercept = 0, r2 = Double.POSITIVE_INFINITY;
    private RingBuffer<Double> input, output;

    // we will use system terms here rather than algebra
    // input is voltage, output is velocity
    // this functional interface will intake an output and return the corresponding open-loop input
    // so if a line is y = mx + b:
    // then y is "input" and x is "output"
    // basically they are backwards. if you even care
    // formula is common, found it at https://statisticsbyjim.com/regression/least-squares-regression-line/
    //public OffsetLinear(List<Double> input, List<Double> output)
    //throws InvalidAlgorithmParameterException {
    //    if (input.size() != output.size())
    //        throw new InvalidAlgorithmParameterException("Mismatched number of arguments: " + input.size() + " inputs, " + output.size() + " outputs");
    //    int N = input.size();
    //    double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    //    for (int i = 0; i < N; i++) {
    //        sumX  += output.get(i);
    //        sumY  +=  input.get(i);
    //        sumXX += output.get(i) * output.get(i);
    //        sumXY += output.get(i) *  input.get(i);
    //    }
    //    setVars(N, sumX, sumY, sumXX, sumXY, input, output);
    //}

    public OffsetLinear(RingBuffer<Double> input, RingBuffer<Double> output)
    throws InvalidAlgorithmParameterException {
        if (input.asList().size() != output.asList().size())
            throw new InvalidAlgorithmParameterException("Mismatched number of arguments: " + input.size() + " inputs, " + output.size() + " outputs");
        this.input = input;
        this.output = output;
        int N = input.asList().size();
        double sumX = output.asStream().mapToDouble(x->x).sum()
        , sumY = input.asStream().mapToDouble(x->x).sum()
        , sumXX = input.asStream().mapToDouble(x->x*x).sum()
        , sumXY = 0
        ;
        for (int i = 0; i < N; i++) sumXY += output.get(i) *  input.get(i);
        setVars(N, sumX, sumY, sumXX, sumXY, input.asList(), output.asList());
    }

    private void setVars(int N, double sumX, double sumY, double sumXX, double sumXY, List<Double> input, List<Double> output) {
        this.slope = (N * sumXY - sumX * sumY) / (N * sumXX - sumX * sumX);
        this.intercept = (sumY - this.slope * sumX) / N;
        this.r2 = 0;
        for (int i = 0; i < N; i++) {
            double r = input.get(i) - this.apply(output.get(i));
            this.r2 += r * r;
        }
    }

    @Override
    public Double apply(Double output) {
        return intercept + slope * output;
    }

    public Double r2() {
        return r2;
    }

    public String toString() {
        return String.format(
            "u = %.5f + %.5f * x"
            , intercept
            , slope
        );
    }

    public String dump() {
        return input.toString() + "\n" + output.toString();
    }
}