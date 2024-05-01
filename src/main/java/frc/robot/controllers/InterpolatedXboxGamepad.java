package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class InterpolatedXboxGamepad extends XboxController {

    static double deadZoneThreshold;
    static double fullThrottleThreshold;

    public InterpolatedXboxGamepad(int port) {
        super(port);

        deadZoneThreshold = 0.05;
        fullThrottleThreshold = 0.9;
    }

    public static boolean inDeadZone(double axis) {
        return (axis > -deadZoneThreshold) && (axis < deadZoneThreshold);
    }

    public static boolean isCeiling(double axis) {
        return axis <= -fullThrottleThreshold || axis >= fullThrottleThreshold;
    }

    public double interpolatedLeftYAxis() {
        if (Math.abs(this.getLeftY()) <= 0.03)
            return 0.0;
        return ((Math.sin(this.getLeftY())) * 1.2);
    }

    public double interpolatedLeftXAxis() {
        if (Math.abs(this.getLeftX()) <= 0.03)
            return 0.0;
        return ((Math.sin(this.getLeftX())) * 1.2);
    }

    public double interpolatedRightXAxis() {
        if (Math.abs(this.getLeftX()) <= 0.03)
            return 0.0;
        return -(Math.sin(this.getLeftX()) * -1.0
        );
    }

    // public double getLeftX() {
    //     if (Math.abs(this.getLeftX()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getLeftX();
    // }

    // public double getRightX() {
    //     if (Math.abs(this.getRightX()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getRightX();
    // }

    // public double getLeftY() {
    //     if (Math.abs(this.getLeftY()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getLeftY();
    // }

    // public double getRightY() {
    //     if (Math.abs(this.getRightY()) <= 0.05) {
    //         return 0.0;
    //     }
    //     return super.getRightY();
    // }

}