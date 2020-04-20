package org.firstinspires.ftc.teamcode.drive.robotClasses;

public class PIDcontroller {
    public static double kp;
    public static double ki;
    public static double kd;
    public double p_error = 0;
    private double i_error = 0;
    private double d_error = 0;
    private double plantState = 0;
    public double previousError = 0;
    private double[] errorArray;

    public static long loopDuration;

    public PIDcontroller(double kp, double ki,double kd, long loopDuration) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.loopDuration = loopDuration;
        errorArray = new double[]{0, 0, 0};

    }

    public void getError(double currentState, double TargetState, double previousError) {
        p_error = TargetState - currentState;
        i_error = (p_error * loopDuration);
        d_error = ((p_error-previousError)/loopDuration);
        errorArray[0] = p_error;
        errorArray[1] = i_error;
        errorArray[2] = d_error;
    }

    public double updateState(double currentState, double TargetState) {
        previousError = errorArray[0];
        getError(currentState,TargetState,previousError);
        plantState = (errorArray[0] * kp) + (errorArray[1] * ki) + (errorArray[2] * kd);
        return plantState;
    }
}
