package robotics;

public class RobotArm2D {
    private double a1;
    private double a2;
    private double q1;
    private double q2;
    
    public RobotArm2D(double a1, double a2) {
        this.a1 = a1;
        this.a2 = a2;
        this.q1 = 0;
        this.q2 = 0;
    }
    
    public void setJointAngles(double q1, double q2) {
        this.q1 = q1;
        this.q2 = q2;
    }
    
    public double getQ1() {
        return q1;
    }
    
    public double getQ2() {
        return q2;
    }
    
    public double getA1() {
        return a1;
    }
    
    public double getA2() {
        return a2;
    }
    
    public void setA1(double a1) {
        this.a1 = a1;
    }
    
    public void setA2(double a2) {
        this.a2 = a2;
    }
    
    public Point2D getJoint1Position() {
        return new Point2D(0, 0);
    }
    
    public Point2D getJoint2Position() {
        Matrix3x3 T01 = getTransformationMatrix(q1, a1);
        return new Point2D(T01.get(0, 2), T01.get(1, 2));
    }
    
    public Point2D getEndEffectorPosition() {
        Matrix3x3 T01 = getTransformationMatrix(q1, a1);
        Matrix3x3 T12 = getTransformationMatrix(q2, a2);
        Matrix3x3 T02 = T01.multiply(T12);
        return new Point2D(T02.get(0, 2), T02.get(1, 2));
    }
    
    private Matrix3x3 getTransformationMatrix(double angle, double length) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        
        return new Matrix3x3(
            cos, -sin, length * cos,
            sin, cos, length * sin,
            0, 0, 1
        );
    }
    
    public static class Point2D {
        public final double x;
        public final double y;
        
        public Point2D(double x, double y) {
            this.x = x;
            this.y = y;
        }
        
        public double distanceTo(Point2D other) {
            double dx = this.x - other.x;
            double dy = this.y - other.y;
            return Math.sqrt(dx * dx + dy * dy);
        }
        
        @Override
        public String toString() {
            return String.format("(%.4f, %.4f)", x, y);
        }
    }
}
