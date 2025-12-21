package robotics;

import java.util.ArrayList;
import java.util.List;

public class InverseKinematicsController {
    private RobotArm2D robot;
    private GeneticAlgorithm ga;
    private List<GenerationResult> generationHistory;
    
    public InverseKinematicsController(RobotArm2D robot, GeneticAlgorithm ga) {
        this.robot = robot;
        this.ga = ga;
        this.generationHistory = new ArrayList<>();
    }
    
    public SolutionResult solve(RobotArm2D.Point2D target, int maxGenerations, double errorThreshold) {
        generationHistory.clear();
        
        List<Individual> population = ga.initializePopulation();
        ga.evaluateFitness(population, robot, target);
        
        Individual bestOverall = ga.getBest(population);
        
        for (int generation = 0; generation < maxGenerations; generation++) {
            ga.evaluateFitness(population, robot, target);
            
            Individual best = ga.getBest(population);
            
            robot.setJointAngles(best.getQ1(), best.getQ2());
            RobotArm2D.Point2D endEffector = robot.getEndEffectorPosition();
            
            GenerationResult result = new GenerationResult(
                generation,
                best.getQ1(),
                best.getQ2(),
                endEffector,
                best.getFitness()
            );
            generationHistory.add(result);
            
            if (best.getFitness() < bestOverall.getFitness()) {
                bestOverall = best.copy();
            }
            
            if (best.getFitness() < errorThreshold) {
                break;
            }
            
            population = ga.evolve(population);
        }
        
        return new SolutionResult(bestOverall, generationHistory);
    }
    
    public List<GenerationResult> getGenerationHistory() {
        return new ArrayList<>(generationHistory);
    }
    
    public static class GenerationResult {
        public final int generation;
        public final double q1;
        public final double q2;
        public final RobotArm2D.Point2D endEffectorPosition;
        public final double error;
        
        public GenerationResult(int generation, double q1, double q2, 
                              RobotArm2D.Point2D endEffectorPosition, double error) {
            this.generation = generation;
            this.q1 = q1;
            this.q2 = q2;
            this.endEffectorPosition = endEffectorPosition;
            this.error = error;
        }
        
        @Override
        public String toString() {
            return String.format("Gen %d: q1=%.4f, q2=%.4f, pos=%s, error=%.6f",
                generation, q1, q2, endEffectorPosition, error);
        }
    }
    
    public static class SolutionResult {
        public final Individual bestIndividual;
        public final List<GenerationResult> history;
        
        public SolutionResult(Individual bestIndividual, List<GenerationResult> history) {
            this.bestIndividual = bestIndividual;
            this.history = new ArrayList<>(history);
        }
    }
}
