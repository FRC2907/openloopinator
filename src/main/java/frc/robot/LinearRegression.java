package frc.robot;

import java.util.Collection;
import java.util.function.Function;

import edu.wpi.first.math.Pair;

public class LinearRegression implements Function<Double, Double> {

    private double slope = 0, intercept = 0, r2 = Double.POSITIVE_INFINITY;
    private Collection<Pair<Double, Double>> data;

    // formula is common, found it at https://statisticsbyjim.com/regression/least-squares-regression-line/
    public LinearRegression(Collection<Pair<Double, Double>> data) {
        this.data = data.stream().filter(p -> p != null).toList();
        int N = this.data.size();
        double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
        sumX  = this.data.stream().mapToDouble(Pair::getFirst ).sum();
        sumY  = this.data.stream().mapToDouble(Pair::getSecond).sum();
        sumXX = this.data.stream().mapToDouble(p -> p.getFirst() * p.getFirst ()).sum();
        sumXY = this.data.stream().mapToDouble(p -> p.getFirst() * p.getSecond()).sum();
        this.slope = (N * sumXY - sumX * sumY) / (N * sumXX - sumX * sumX);
        this.intercept = (sumY - this.slope * sumX) / N;
        this.r2 = this.data.stream().map(datum -> datum.getSecond() - this.apply(datum.getFirst())).mapToDouble(r -> r*r).sum();
    }

    @Override
    public Double apply(Double output) {
        return intercept + slope * output;
    }

    public double getIntercept() {
        return intercept;
    }

    public double getSlope() {
        return slope;
    }

    public double getError() {
        return r2;
    }

    public Collection<Pair<Double, Double>> getData() {
        return data;
    }

    public String toString() {
        return String.format(
            "u = %.5f + %.5f * x"
            , intercept
            , slope
        );
    }

    public String dump() {
        return String.join("\n",
            data.stream()
            .sorted((a,b) -> (int)Math.signum((a.getFirst() - b.getFirst())))
            .<CharSequence>map(p -> String.format("%.4f V -> %.1f rad/s", p.getSecond(), p.getFirst()))
            .toList()
        );
    }
}