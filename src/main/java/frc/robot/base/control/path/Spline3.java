package frc.robot.base.control.path;
import org.la4j.Matrix;
import org.la4j.Vector;
import org.la4j.operation.MatrixMatrixOperation;
import org.la4j.operation.MatrixOperation;
import org.la4j.operation.MatrixVectorOperation;

import java.io.Serializable;
import java.text.NumberFormat;

public class Spline3 extends Matrix {

    private double xStart;
    private double xEnd;
    private double xDirStart;
    private double yDirStart;
    private double yStart;
    private double yEnd;
    private double xDirEnd;
    private double yDirEnd;
    private double polyCoefs[];

    public Spline3(double[] point1, double[] point2){ //Format: x,y,theta
        this.xStart = point1[0];
        this.xEnd = point1[1];
        this.xDirStart = Math.cos(point1[2]);
        this.yDirStart = Math.sin(point1[2]);
        this.yStart = point2[0];
        this.yEnd = point2[1];
        this.xDirEnd = Math.cos(point2[2]);
        this.yDirEnd = Math.sin(point2[2]);
        this.polyCoefs = createPathPoly();
    }

    private double[] createPathPoly(){
       // Matrix invA = new Matrix(4,4);
        return new double[]{1, 2, 3};
    }


    @Override
    public double get(int i, int j) {
        return 0;
    }

    @Override
    public void set(int i, int j, double value) {

    }

    @Override
    public Vector getRow(int i) {
        return null;
    }

    @Override
    public Vector getColumn(int j) {
        return null;
    }

    @Override
    public Matrix blankOfShape(int rows, int columns) {
        return null;
    }

    @Override
    public Matrix copyOfShape(int rows, int columns) {
        return null;
    }

    @Override
    public <T> T apply(MatrixOperation<T> operation) {
        return null;
    }

    @Override
    public <T> T apply(MatrixMatrixOperation<T> operation, Matrix that) {
        return null;
    }

    @Override
    public <T> T apply(MatrixVectorOperation<T> operation, Vector that) {
        return null;
    }

    @Override
    public byte[] toBinary() {
        return new byte[0];
    }

    @Override
    public String toMatrixMarket(NumberFormat formatter) {
        return null;
    }
}
