package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controlsystems.PID;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RobotStateEstimator;
import frc.util.DriveSignal;
import frc.util.Point;
import frc.util.Util;

/**
 * DriveToPoint
 */
public class DriveToPointFinalHeadingControl implements Action {

    private double x;
    private double y;
    private double theta;
    private double minVelocity;
    private double maxVelocity;
    private double absTurnMax;
    private double epsilonDistance;
    private double epsilonAngle;
    private double turnRate;
    private boolean slowTurn;
    private double maxTurn;
    private boolean isFinished;

    private DriveTrain drive;

    private PID turnPID;
    private PID drivePID;

    public DriveToPointFinalHeadingControl(double x, double y, double theta, double minVelocity, double maxVelocity, double absTurnMax, double epsilonDist, double epsilonAngle,
            double turnRate, boolean slowTurn, double maxTurn) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.absTurnMax = absTurnMax;
        this.epsilonDistance = epsilonDist;
        this.epsilonAngle = epsilonAngle;
        this.turnRate = turnRate;
        this.slowTurn = slowTurn;
        this.maxTurn = maxTurn;
        this.isFinished = false;
        this.drive = DriveTrain.getInstance();
    }

    private Point getRotatedError() {
        double currentX = RobotStateEstimator.getInstance().getCurrentPose().getX();
        double currentY = RobotStateEstimator.getInstance().getCurrentPose().getY();
        double rotation = -this.theta;

        Point currentPosition = new Point(currentX, currentY);
        Point finalPosition = new Point(this.x, this.y);

        currentPosition.rotateByAngleDegrees(rotation);
        finalPosition.rotateByAngleDegrees(rotation);

        double xError = finalPosition.getX() - currentPosition.getX();
        double yError = finalPosition.getY() - currentPosition.getY();

        //Flip x and y to start at 0 heading not 90
        return new Point(yError, xError);
    }

    @Override
    public void start() {
        drivePID = new PID(Constants.DriveTrain.STRAIGHT_KP, Constants.DriveTrain.STRAIGHT_KI, Constants.DriveTrain.STRAIGHT_KD, minVelocity, maxVelocity);
        turnPID = new PID(Constants.DriveTrain.TURN_KP, Constants.DriveTrain.TURN_KI, Constants.DriveTrain.TURN_KD, -1000, 1000);
    }

    @Override
    public void update() {
        Point error = getRotatedError();
        double targetHeading;

        if (error.getY() < 0) { // flip X if we are going backwards
            error.setX(-error.getX());
        }

        double turningOffset = (error.getX() * this.turnRate); // based on how far we are in x turn more

        if (turningOffset > this.maxTurn) { // limit it to be within 90* from the target angle
            turningOffset = this.maxTurn;
        } else if (turningOffset < -this.maxTurn) {
            turningOffset = -this.maxTurn;
        }

        targetHeading = this.theta - turningOffset;

        double angle = drive.getHeadingDegrees();
        double offset = angle % 360;

        // Corrects the target to work with Gyro position
        double desiredTurnValue = 0;
        if (targetHeading - offset < -180) {
            desiredTurnValue = angle + 360 + targetHeading - offset;
        } else if (targetHeading - offset < 180) {
            desiredTurnValue = angle + targetHeading - offset;
        } else {
            desiredTurnValue = angle - 360 + targetHeading - offset;
        }

        double yError = error.getY();
        double yOutput;

        yOutput = drivePID.getValue(yError, 0);

        double absDeltaHeading = Math.abs(desiredTurnValue - drive.getHeadingDegrees());
        if (absDeltaHeading > 90) { // prevents the y output from being reversed in the next calculation
            absDeltaHeading = 90;
        }

        if (slowTurn) { // slow down y if we aren't facing the correct angle
            yOutput = yOutput * (((-1 * absDeltaHeading) / 90.0) + 1);
        }

        double xOutput = turnPID.getValue(drive.getHeadingDegrees(), desiredTurnValue);

        if (!this.slowTurn) {
            xOutput *= 0.85;
        }

        xOutput = Util.clamp(xOutput, -absTurnMax, absTurnMax);

        double leftOut = yOutput + xOutput;
        double rightOut = yOutput - xOutput;
        DriveSignal ds = new DriveSignal(leftOut, rightOut);

        SmartDashboard.putString("signal", ds.toString());

        double dist = (yError);
        SmartDashboard.putNumber("Heading Done", Math.abs(theta - drive.getHeadingDegrees()));
        if (Math.abs(dist) < this.epsilonDistance && Math.abs(theta - drive.getHeadingDegrees()) < this.epsilonAngle) {
            isFinished = true;
        } else if(Math.abs(dist) < this.epsilonDistance) {
            double turnSpeed = turnPID.getValue(drive.getHeadingDegrees(), theta);
            drive.setSpeed(new DriveSignal(turnSpeed, -turnSpeed));
        } else {
            drive.setSpeed(ds);
        }
    }

    @Override
    public void done() {
        System.out.println("Done drive");
        drive.setSpeed(new DriveSignal(0,0));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}