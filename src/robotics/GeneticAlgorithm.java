package robotics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

public class GeneticAlgorithm {
    private int populationSize;
    private double mutationRate;
    private double crossoverRate;
    private double minAngle;
    private double maxAngle;
    private SelectionType selectionType;
    private CrossoverType crossoverType;
    private Random random;
    
    public enum SelectionType {
        TOURNAMENT,
        ROULETTE_WHEEL
    }
    
    public enum CrossoverType {
        SINGLE_POINT,
        UNIFORM
    }
    
    public GeneticAlgorithm(int populationSize, double mutationRate, double crossoverRate,
                           double minAngle, double maxAngle, SelectionType selectionType,
                           CrossoverType crossoverType) {
        this.populationSize = populationSize;
        this.mutationRate = mutationRate;
        this.crossoverRate = crossoverRate;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.selectionType = selectionType;
        this.crossoverType = crossoverType;
        this.random = new Random();
    }
    
    public List<Individual> initializePopulation() {
        List<Individual> population = new ArrayList<>();
        for (int i = 0; i < populationSize; i++) {
            population.add(Individual.createRandom(minAngle, maxAngle));
        }
        return population;
    }
    
    public void evaluateFitness(List<Individual> population, RobotArm2D robot, 
                                RobotArm2D.Point2D target) {
        for (Individual individual : population) {
            robot.setJointAngles(individual.getQ1(), individual.getQ2());
            RobotArm2D.Point2D endEffector = robot.getEndEffectorPosition();
            double distance = endEffector.distanceTo(target);
            individual.setFitness(distance);
        }
    }
    
    public Individual select(List<Individual> population) {
        if (selectionType == SelectionType.TOURNAMENT) {
            return tournamentSelection(population);
        } else {
            return rouletteWheelSelection(population);
        }
    }
    
    private Individual tournamentSelection(List<Individual> population) {
        int tournamentSize = 3;
        Individual best = population.get(random.nextInt(population.size()));
        for (int i = 1; i < tournamentSize; i++) {
            Individual competitor = population.get(random.nextInt(population.size()));
            if (competitor.getFitness() < best.getFitness()) {
                best = competitor;
            }
        }
        return best.copy();
    }
    
    private Individual rouletteWheelSelection(List<Individual> population) {
        double maxFitness = population.stream()
            .mapToDouble(Individual::getFitness)
            .max()
            .orElse(1.0);
        
        double totalInverseFitness = population.stream()
            .mapToDouble(ind -> maxFitness - ind.getFitness() + 1.0)
            .sum();
        
        double spin = random.nextDouble() * totalInverseFitness;
        double current = 0;
        
        for (Individual individual : population) {
            current += maxFitness - individual.getFitness() + 1.0;
            if (current >= spin) {
                return individual.copy();
            }
        }
        
        return population.get(population.size() - 1).copy();
    }
    
    public Individual[] crossover(Individual parent1, Individual parent2) {
        if (random.nextDouble() > crossoverRate) {
            return new Individual[]{parent1.copy(), parent2.copy()};
        }
        
        if (crossoverType == CrossoverType.SINGLE_POINT) {
            return singlePointCrossover(parent1, parent2);
        } else {
            return uniformCrossover(parent1, parent2);
        }
    }
    
    private Individual[] singlePointCrossover(Individual parent1, Individual parent2) {
        Individual child1 = new Individual(parent1.getQ1(), parent2.getQ2());
        Individual child2 = new Individual(parent2.getQ1(), parent1.getQ2());
        return new Individual[]{child1, child2};
    }
    
    private Individual[] uniformCrossover(Individual parent1, Individual parent2) {
        double q1_child1 = random.nextBoolean() ? parent1.getQ1() : parent2.getQ1();
        double q2_child1 = random.nextBoolean() ? parent1.getQ2() : parent2.getQ2();
        
        double q1_child2 = random.nextBoolean() ? parent1.getQ1() : parent2.getQ1();
        double q2_child2 = random.nextBoolean() ? parent1.getQ2() : parent2.getQ2();
        
        Individual child1 = new Individual(q1_child1, q2_child1);
        Individual child2 = new Individual(q1_child2, q2_child2);
        return new Individual[]{child1, child2};
    }
    
    public void mutate(Individual individual) {
        if (random.nextDouble() < mutationRate) {
            double perturbation = (random.nextDouble() - 0.5) * 0.5;
            double newQ1 = individual.getQ1() + perturbation;
            newQ1 = Math.max(minAngle, Math.min(maxAngle, newQ1));
            individual.setQ1(newQ1);
        }
        
        if (random.nextDouble() < mutationRate) {
            double perturbation = (random.nextDouble() - 0.5) * 0.5;
            double newQ2 = individual.getQ2() + perturbation;
            newQ2 = Math.max(minAngle, Math.min(maxAngle, newQ2));
            individual.setQ2(newQ2);
        }
    }
    
    public List<Individual> evolve(List<Individual> population) {
        List<Individual> newPopulation = new ArrayList<>();
        
        Collections.sort(population);
        int eliteCount = Math.max(1, populationSize / 10);
        for (int i = 0; i < eliteCount; i++) {
            newPopulation.add(population.get(i).copy());
        }
        
        while (newPopulation.size() < populationSize) {
            Individual parent1 = select(population);
            Individual parent2 = select(population);
            
            Individual[] children = crossover(parent1, parent2);
            
            mutate(children[0]);
            mutate(children[1]);
            
            newPopulation.add(children[0]);
            if (newPopulation.size() < populationSize) {
                newPopulation.add(children[1]);
            }
        }
        
        return newPopulation;
    }
    
    public Individual getBest(List<Individual> population) {
        return Collections.min(population);
    }
}
