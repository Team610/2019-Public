package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controlsystems.PID;
import frc.loops.Loop;
import frc.robot.Controls;
import frc.util.Toggle;

public class Limelight extends Subsystem {

    private static Limelight instance;
    private NetworkTable tableFront;
    private NetworkTable tableBack;
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ta;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry camMode;
    private Toggle visionToggle;
    private PID visionPID;
    private PID drivePID;

    private boolean isFrontCam;

    private double turnkP = -0.007;
    private double turnkI = 0.0;
    private double turnkD = 0.0;
    private double straightkP = 0.2;
    private double straightkI = 0.0;
    private double straightkD = 0.0;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    private Limelight() {
        isFrontCam = true;

        tableFront = NetworkTableInstance.getDefault().getTable("limelight-front");
        tableBack = NetworkTableInstance.getDefault().getTable("limelight-back");
        table = tableFront;
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        visionPID = new PID(turnkP, turnkI, turnkD);
        drivePID = new PID(straightkP, straightkI, straightkD, 0, 0.25);
        visionToggle = new Toggle();
    }

    public void setTrackingMode(boolean track) {
        if (track) {
            camMode.setDouble(0);
            ledMode.setDouble(0);
        } else {
            camMode.setDouble(1);
            ledMode.setDouble(1);
        }
    }

    public void setVisionModeState(boolean isTracking) {
        visionToggle.setToggleState(isTracking);
    }

    public boolean getTrackingMode() {
        return table.getEntry("camMode").getDouble(0) == 0;
    }

    public double getValue() {
        return visionPID.getUncappedValue(tx.getDouble(0.0), 0);
    }

    public boolean isFrontCam() {
        return isFrontCam;
    }

    public double getThrottle(double inches) {
        if (getArea() < 1.4) {
            drivePID.setMax(0.2);
        } else {
            drivePID.setMax(0.15);
        }
        return (isFrontCam() ? 1 : -1) * drivePID.getValue(getArea(), inches);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    @Override
    protected Loop loop() {
        return new Loop() {

            @Override
            public void onFirstStart(double time) {
            
            }

            @Override
            public void onEnable(double time) {
                // Set LED to pipeline mode
                table = tableBack;
                updateTable();
                ledMode.setDouble(0);
                table = tableFront;
                updateTable();
                ledMode.setDouble(0);
            }

            @Override
            public void onDisable(double time) {
                // Set LED to off
                table = tableBack;
                updateTable();
                ledMode.setDouble(1);
                table = tableFront;
                updateTable();
                ledMode.setDouble(1);
            }

            @Override
            public void enabledLoop(double time) {
                // Put value onto SmartDashboard
                SmartDashboard.putNumber("tx PID Value", getValue());
                SmartDashboard.putNumber("tx", tx.getDouble(0.0));
                SmartDashboard.putNumber("ta", getArea());

                // Update required values
                updateTable();

                visionToggle.update(Controls.getInstance().shouldVisionAssist());
                setTrackingMode(visionToggle.getToggleState());

                turnkP = SmartDashboard.getNumber("Limelight kP", turnkP);
                turnkI = SmartDashboard.getNumber("Limelight kI", turnkI);
                turnkD = SmartDashboard.getNumber("Limelight kD", turnkD);
            }

            @Override
            public void disabledLoop(double time) {
                table = tableBack;
                updateTable();
                setTrackingMode(false);
                table = tableFront;
                updateTable();
                setTrackingMode(false);
            }

            @Override
            public void allLoop(double time) {
                if (getArea() > 5.9 && visionToggle.getToggleState()) {
                    Controls.getInstance().driverRumble(0.7, 100);
                }

                if (SubsystemManager.getInstance().getIsFrontPreset()) {
                    isFrontCam = true;
                    table = tableFront;
                    updateTable();
                } else {
                    isFrontCam = false;
                    table = tableBack;
                    updateTable();
                }
            }
        };
    }

    public void updateTable() {
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
    }

    @Override
    public void zeroSensors() {

    }

}