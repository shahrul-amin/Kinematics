package robotics;

import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.stage.Stage;

public class RobotKinematicsApp extends Application {
    private RobotArm2D robot;
    private RobotVisualization visualization;
    private TextField q1Field, q2Field, a1Field, a2Field;
    private TextField targetXField, targetYField;
    private TextArea outputArea;
    private CheckBox reverseMotionCheck;
    
    @Override
    public void start(Stage primaryStage) {
        robot = new RobotArm2D(2.0, 1.5);
        
        BorderPane root = new BorderPane();
        root.setPadding(new Insets(10));
        
        visualization = new RobotVisualization(800, 600);
        visualization.setRobot(robot);
        root.setCenter(visualization);
        
        VBox controlPanel = createControlPanel();
        root.setRight(controlPanel);
        
        Scene scene = new Scene(root, 1200, 650);
        primaryStage.setTitle("2D SCARA Robot - Forward & Inverse Kinematics");
        primaryStage.setScene(scene);
        primaryStage.show();
    }
    
    private VBox createControlPanel() {
        VBox panel = new VBox(15);
        panel.setPadding(new Insets(10));
        panel.setPrefWidth(350);
        panel.setStyle("-fx-background-color: #f0f0f0;");
        
        Label titleLabel = new Label("PART A: FORWARD KINEMATICS");
        titleLabel.setStyle("-fx-font-weight: bold; -fx-font-size: 14px;");
        
        GridPane fkGrid = new GridPane();
        fkGrid.setHgap(10);
        fkGrid.setVgap(10);
        
        a1Field = new TextField("2.0");
        a2Field = new TextField("1.5");
        q1Field = new TextField("0.0");
        q2Field = new TextField("0.0");
        
        fkGrid.add(new Label("Arm 1 Length (a1):"), 0, 0);
        fkGrid.add(a1Field, 1, 0);
        fkGrid.add(new Label("Arm 2 Length (a2):"), 0, 1);
        fkGrid.add(a2Field, 1, 1);
        fkGrid.add(new Label("Joint 1 Angle (q1):"), 0, 2);
        fkGrid.add(q1Field, 1, 2);
        fkGrid.add(new Label("Joint 2 Angle (q2):"), 0, 3);
        fkGrid.add(q2Field, 1, 3);
        
        reverseMotionCheck = new CheckBox("Reverse Motion");
        
        Button updateButton = new Button("Update Robot Pose");
        updateButton.setMaxWidth(Double.MAX_VALUE);
        updateButton.setOnAction(e -> updateRobotPose());
        
        Separator sep1 = new Separator();
        
        Label ikTitle = new Label("PART B: INVERSE KINEMATICS (GA)");
        ikTitle.setStyle("-fx-font-weight: bold; -fx-font-size: 14px;");
        
        GridPane ikGrid = new GridPane();
        ikGrid.setHgap(10);
        ikGrid.setVgap(10);
        
        targetXField = new TextField("2.0");
        targetYField = new TextField("2.0");
        
        ikGrid.add(new Label("Target X:"), 0, 0);
        ikGrid.add(targetXField, 1, 0);
        ikGrid.add(new Label("Target Y:"), 0, 1);
        ikGrid.add(targetYField, 1, 1);
        
        Button solveButton = new Button("Solve using GA");
        solveButton.setMaxWidth(Double.MAX_VALUE);
        solveButton.setOnAction(e -> solveInverseKinematics());
        
        Button testButton = new Button("Run Tests (3 Targets)");
        testButton.setMaxWidth(Double.MAX_VALUE);
        testButton.setOnAction(e -> runTests());
        
        outputArea = new TextArea();
        outputArea.setPrefHeight(150);
        outputArea.setEditable(false);
        outputArea.setWrapText(true);
        
        panel.getChildren().addAll(
            titleLabel,
            fkGrid,
            reverseMotionCheck,
            updateButton,
            sep1,
            ikTitle,
            ikGrid,
            solveButton,
            testButton,
            new Label("Output:"),
            outputArea
        );
        
        return panel;
    }
    
    private void updateRobotPose() {
        try {
            double a1 = Double.parseDouble(a1Field.getText());
            double a2 = Double.parseDouble(a2Field.getText());
            double q1 = Math.toRadians(Double.parseDouble(q1Field.getText()));
            double q2 = Math.toRadians(Double.parseDouble(q2Field.getText()));
            
            robot.setA1(a1);
            robot.setA2(a2);
            
            if (reverseMotionCheck.isSelected()) {
                visualization.setAnimationSpeed(0.05);
                visualization.animateToAngles(q1, q2);
            } else {
                robot.setJointAngles(q1, q2);
                visualization.draw();
            }
            
            RobotArm2D.Point2D endEffector = robot.getEndEffectorPosition();
            outputArea.setText(String.format("End Effector Position: %s", endEffector));
            
        } catch (NumberFormatException ex) {
            showAlert("Invalid Input", "Please enter valid numeric values.");
        }
    }
    
    private void solveInverseKinematics() {
        try {
            double targetX = Double.parseDouble(targetXField.getText());
            double targetY = Double.parseDouble(targetYField.getText());
            RobotArm2D.Point2D target = new RobotArm2D.Point2D(targetX, targetY);
            
            visualization.setTargetPoint(target);
            
            double a1 = Double.parseDouble(a1Field.getText());
            double a2 = Double.parseDouble(a2Field.getText());
            robot.setA1(a1);
            robot.setA2(a2);
            
            GeneticAlgorithm ga = new GeneticAlgorithm(
                100,
                0.1,
                0.8,
                -Math.PI,
                Math.PI,
                GeneticAlgorithm.SelectionType.TOURNAMENT,
                GeneticAlgorithm.CrossoverType.UNIFORM
            );
            
            InverseKinematicsController ikController = new InverseKinematicsController(robot, ga);
            
            outputArea.setText("Running Genetic Algorithm...\n");
            
            InverseKinematicsController.SolutionResult result = 
                ikController.solve(target, 200, 0.01);
            
            StringBuilder output = new StringBuilder();
            output.append(String.format("Target Position: (%.4f, %.4f)\n\n", targetX, targetY));
            output.append("Generation Results:\n");
            
            for (int i = 0; i < Math.min(10, result.history.size()); i++) {
                output.append(result.history.get(i)).append("\n");
            }
            
            if (result.history.size() > 10) {
                output.append("...\n");
                output.append(result.history.get(result.history.size() - 1)).append("\n");
            }
            
            Individual best = result.bestIndividual;
            robot.setJointAngles(best.getQ1(), best.getQ2());
            RobotArm2D.Point2D finalPos = robot.getEndEffectorPosition();
            
            output.append(String.format("\nBest Solution:\n"));
            output.append(String.format("q1 = %.4f rad (%.2f deg)\n", best.getQ1(), Math.toDegrees(best.getQ1())));
            output.append(String.format("q2 = %.4f rad (%.2f deg)\n", best.getQ2(), Math.toDegrees(best.getQ2())));
            output.append(String.format("Final Position: %s\n", finalPos));
            output.append(String.format("Error: %.6f units\n", best.getFitness()));
            
            outputArea.setText(output.toString());
            
            visualization.animateToAngles(best.getQ1(), best.getQ2());
            
        } catch (NumberFormatException ex) {
            showAlert("Invalid Input", "Please enter valid numeric values for target position.");
        }
    }
    
    private void runTests() {
        RobotArm2D.Point2D[] testTargets = {
            new RobotArm2D.Point2D(2.0, 2.0),
            new RobotArm2D.Point2D(1.5, 1.0),
            new RobotArm2D.Point2D(-1.0, 2.5)
        };
        
        StringBuilder output = new StringBuilder();
        output.append("Running Tests on 3 Target Positions\n");
        output.append("=====================================\n\n");
        
        double a1 = Double.parseDouble(a1Field.getText());
        double a2 = Double.parseDouble(a2Field.getText());
        robot.setA1(a1);
        robot.setA2(a2);
        
        for (int i = 0; i < testTargets.length; i++) {
            RobotArm2D.Point2D target = testTargets[i];
            
            output.append(String.format("Test %d: Target = %s\n", i + 1, target));
            
            GeneticAlgorithm ga = new GeneticAlgorithm(
                100,
                0.1,
                0.8,
                -Math.PI,
                Math.PI,
                GeneticAlgorithm.SelectionType.TOURNAMENT,
                GeneticAlgorithm.CrossoverType.UNIFORM
            );
            
            InverseKinematicsController ikController = new InverseKinematicsController(robot, ga);
            InverseKinematicsController.SolutionResult result = 
                ikController.solve(target, 200, 0.01);
            
            Individual best = result.bestIndividual;
            robot.setJointAngles(best.getQ1(), best.getQ2());
            RobotArm2D.Point2D finalPos = robot.getEndEffectorPosition();
            
            output.append(String.format("  Solution: q1=%.4f, q2=%.4f\n", best.getQ1(), best.getQ2()));
            output.append(String.format("  Reached: %s\n", finalPos));
            output.append(String.format("  Error: %.6f units", best.getFitness()));
            
            if (best.getFitness() < 0.01) {
                output.append(" [PASS]\n");
            } else {
                output.append(" [FAIL]\n");
            }
            output.append("\n");
        }
        
        outputArea.setText(output.toString());
    }
    
    private void showAlert(String title, String message) {
        Alert alert = new Alert(Alert.AlertType.ERROR);
        alert.setTitle(title);
        alert.setHeaderText(null);
        alert.setContentText(message);
        alert.showAndWait();
    }
    
    public static void main(String[] args) {
        launch(args);
    }
}
