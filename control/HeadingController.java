package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class HeadingController {

    public double kP = Constants.HEADING_KP;
    public double kD = Constants.HEADING_KD;
    public double kS = Constants.HEADING_KS;

    private double targetDeg = 0;
    private double lastHeadingDeg = 0;
    private double rateDegPerSec = 0;
    private boolean hasLast = false;
    private final ElapsedTime timer = new ElapsedTime();

    public void setTarget(double deg) {
        targetDeg = wrapDeg(deg);}
    public double getTarget() {
        return targetDeg;}
    public double getRate() {
        return rateDegPerSec;}

    public double getError(double headingDeg) {
        return wrapDeg(targetDeg - headingDeg);
    }

    public double update(double headingDeg) {
        double dt = timer.seconds();
        timer.reset();

        // V = DELTA_HEADING/TIME
        if (hasLast && dt > 1e-3) {
            double rawRate = wrapDeg(headingDeg - lastHeadingDeg) / dt;
            rateDegPerSec = Constants.RATE_FILTER * rawRate
                    + (1 - Constants.RATE_FILTER) * rateDegPerSec;
        }
        lastHeadingDeg = headingDeg;
        hasLast = true;

        double error = getError(headingDeg);
        if (isSettled(headingDeg)) return 0;

        double ccw = kP * error - kD * rateDegPerSec;
        if (Math.abs(error) > Constants.HEADING_TOLERANCE_DEG) {
            ccw += kS * Math.signum(error);
        }
        // joy negatively associates with default turning.
        double turn = -ccw;
        // return max E [-0.5;0.5] - clamp lại để ít giật (turn > 0.5 khi error > 25 độ <=> từ 25 độ là max 0.5)
        return Math.max(-Constants.HEADING_MAX_POWER, Math.min(Constants.HEADING_MAX_POWER, turn));
    }

    public boolean isSettled(double headingDeg) {
        return Math.abs(getError(headingDeg)) < Constants.HEADING_TOLERANCE_DEG
                && Math.abs(rateDegPerSec) < Constants.HEADING_RATE_TOLERANCE;
    }

    // untangle corners.
    public static double wrapDeg(double a) {
        while (a > 180)  a -= 360;
        while (a < -180) a += 360;
        return a;
    }
    public void resetState() {
        hasLast = false;
        rateDegPerSec = 0;
    }
}