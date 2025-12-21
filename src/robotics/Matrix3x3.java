package robotics;

public class Matrix3x3 {
    private double[][] data;
    
    public Matrix3x3() {
        data = new double[3][3];
    }
    
    public Matrix3x3(double m00, double m01, double m02,
                     double m10, double m11, double m12,
                     double m20, double m21, double m22) {
        data = new double[3][3];
        data[0][0] = m00; data[0][1] = m01; data[0][2] = m02;
        data[1][0] = m10; data[1][1] = m11; data[1][2] = m12;
        data[2][0] = m20; data[2][1] = m21; data[2][2] = m22;
    }
    
    public double get(int row, int col) {
        return data[row][col];
    }
    
    public void set(int row, int col, double value) {
        data[row][col] = value;
    }
    
    public Matrix3x3 multiply(Matrix3x3 other) {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double sum = 0;
                for (int k = 0; k < 3; k++) {
                    sum += this.data[i][k] * other.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        return result;
    }
}
