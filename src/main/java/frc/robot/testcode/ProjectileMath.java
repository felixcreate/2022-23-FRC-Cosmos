package frc.robot.testcode;

public class ProjectileMath {
    public static double calculateVelocity(double x, double y, double theta) {
        double gravity = Math.sqrt(9.80665/2);
        double term1 = y - (x * Math.tan(Math.toRadians(theta)));
        double realCheck = Math.sqrt(-(term1));
        if(Double.isNaN(realCheck)) {
            return -1;
        }
        else {
            double denominator = realCheck * Math.cos(Math.toRadians(theta));
            return (x*gravity)/denominator;
        }
    }
}
