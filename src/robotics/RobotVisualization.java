package robotics;

import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.util.Duration;

public class RobotVisualization extends Canvas {
    private RobotArm2D robot;
    private double scale;
    private double centerX;
    private double centerY;
    private Timeline animation;
    private double targetQ1;
    private double targetQ2;
    private double animationSpeed;
    private RobotArm2D.Point2D targetPoint;
    
    public RobotVisualization(double width, double height) {
        super(width, height);
        this.centerX = width / 2;
        this.centerY = height / 2;
        this.scale = 50;
        this.animationSpeed = 0.05;
        this.targetPoint = null;
    }
    
    public void setRobot(RobotArm2D robot) {
        this.robot = robot;
        draw();
    }
    
    public void setTargetPoint(RobotArm2D.Point2D point) {
        this.targetPoint = point;
        draw();
    }
    
    public void clearTargetPoint() {
        this.targetPoint = null;
        draw();
    }
    
    public void draw() {
        if (robot == null) return;
        
        GraphicsContext gc = getGraphicsContext2D();
        gc.setFill(Color.WHITE);
        gc.fillRect(0, 0, getWidth(), getHeight());
        
        gc.setStroke(Color.LIGHTGRAY);
        gc.setLineWidth(1);
        for (int i = -10; i <= 10; i++) {
            gc.strokeLine(centerX + i * scale, 0, centerX + i * scale, getHeight());
            gc.strokeLine(0, centerY - i * scale, getWidth(), centerY - i * scale);
        }
        
        gc.setStroke(Color.BLACK);
        gc.setLineWidth(2);
        gc.strokeLine(0, centerY, getWidth(), centerY);
        gc.strokeLine(centerX, 0, centerX, getHeight());
        
        RobotArm2D.Point2D joint1 = robot.getJoint1Position();
        RobotArm2D.Point2D joint2 = robot.getJoint2Position();
        RobotArm2D.Point2D endEffector = robot.getEndEffectorPosition();
        
        double x1 = centerX + joint1.x * scale;
        double y1 = centerY - joint1.y * scale;
        double x2 = centerX + joint2.x * scale;
        double y2 = centerY - joint2.y * scale;
        double x3 = centerX + endEffector.x * scale;
        double y3 = centerY - endEffector.y * scale;
        
        gc.setStroke(Color.BLUE);
        gc.setLineWidth(4);
        gc.strokeLine(x1, y1, x2, y2);
        
        gc.setStroke(Color.RED);
        gc.strokeLine(x2, y2, x3, y3);
        
        gc.setFill(Color.BLACK);
        gc.fillOval(x1 - 6, y1 - 6, 12, 12);
        gc.fillOval(x2 - 6, y2 - 6, 12, 12);
        
        gc.setFill(Color.GREEN);
        gc.fillOval(x3 - 8, y3 - 8, 16, 16);
        
        if (targetPoint != null) {
            double tx = centerX + targetPoint.x * scale;
            double ty = centerY - targetPoint.y * scale;
            gc.setFill(Color.ORANGE);
            gc.fillOval(tx - 6, ty - 6, 12, 12);
            gc.setStroke(Color.ORANGE);
            gc.setLineWidth(2);
            gc.strokeOval(tx - 10, ty - 10, 20, 20);
        }
    }
    
    public void animateToAngles(double q1, double q2) {
        if (animation != null) {
            animation.stop();
        }
        
        targetQ1 = q1;
        targetQ2 = q2;
        
        animation = new Timeline(new KeyFrame(Duration.millis(16), e -> {
            double currentQ1 = robot.getQ1();
            double currentQ2 = robot.getQ2();
            
            double deltaQ1 = targetQ1 - currentQ1;
            double deltaQ2 = targetQ2 - currentQ2;
            
            if (Math.abs(deltaQ1) < 0.001 && Math.abs(deltaQ2) < 0.001) {
                robot.setJointAngles(targetQ1, targetQ2);
                draw();
                animation.stop();
            } else {
                double newQ1 = currentQ1 + deltaQ1 * animationSpeed;
                double newQ2 = currentQ2 + deltaQ2 * animationSpeed;
                robot.setJointAngles(newQ1, newQ2);
                draw();
            }
        }));
        
        animation.setCycleCount(Timeline.INDEFINITE);
        animation.play();
    }
    
    public void setAnimationSpeed(double speed) {
        this.animationSpeed = speed;
    }
}
