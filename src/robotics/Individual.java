package robotics;

import java.util.Random;

public class Individual implements Comparable<Individual> {
    private double q1;
    private double q2;
    private double fitness;
    private static final Random random = new Random();
    
    public Individual(double q1, double q2) {
        this.q1 = q1;
        this.q2 = q2;
        this.fitness = Double.MAX_VALUE;
    }
    
    public static Individual createRandom(double minAngle, double maxAngle) {
        double q1 = minAngle + random.nextDouble() * (maxAngle - minAngle);
        double q2 = minAngle + random.nextDouble() * (maxAngle - minAngle);
        return new Individual(q1, q2);
    }
    
    public double getQ1() {
        return q1;
    }
    
    public double getQ2() {
        return q2;
    }
    
    public void setQ1(double q1) {
        this.q1 = q1;
    }
    
    public void setQ2(double q2) {
        this.q2 = q2;
    }
    
    public double getFitness() {
        return fitness;
    }
    
    public void setFitness(double fitness) {
        this.fitness = fitness;
    }
    
    public Individual copy() {
        Individual copy = new Individual(this.q1, this.q2);
        copy.fitness = this.fitness;
        return copy;
    }
    
    @Override
    public int compareTo(Individual other) {
        return Double.compare(this.fitness, other.fitness);
    }
}
