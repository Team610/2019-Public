package frc.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * LoopManager
 */
public class LoopManager {

    private Notifier notifier;
    private Object lock = new Object();
    private List<Loop> loops;
    private double timestamp;
    private double dt;
    private boolean running;
    private boolean calledEnable, calledDisable;
    private boolean isEnabled, isDisabled;

    private static LoopManager looper;

    public static LoopManager getLooper() {
        if (looper == null) {
            looper = new LoopManager();
        }
        return looper;
    }

    private final Runnable runnable = new Runnable() {

        @Override
        public void run() {
            try {
                synchronized (lock) {
                    if (running) {
                        double now = Timer.getFPGATimestamp();

                        if (isEnabled) {
                            for (Loop loop : loops) {
                                loop.enabledLoop(now);
                            }
                        }
                        if (isDisabled) {
                            for (Loop loop : loops) {
                                loop.disabledLoop(now);
                            }
                        }
                        for (Loop loop : loops) {
                            loop.allLoop(now);
                        }

                        dt = now - timestamp;
                        timestamp = now;
                        SmartDashboard.putNumber("Loop Dt", dt);
                    }
                }
            } catch (Exception e) {
                SmartDashboard.putString("Errored", e.toString());
            }
        }

    };

    private LoopManager() {
        notifier = new Notifier(runnable);
        running = false;
        loops = new ArrayList<>();
        calledEnable = false;
        calledDisable = false;
        isEnabled = true;
        isDisabled = true;
    }

    public synchronized void addLoop(Loop loop) {
        synchronized (lock) {
            if (loop != null) {
                loops.add(loop);
            }
        }
    }

    public void callOnEnable() {
        if (!calledEnable) {
            isEnabled = true;
            isDisabled = false;
            calledEnable = true;
            calledDisable = false;
            double now = Timer.getFPGATimestamp();
            for (Loop loop : loops) {
                loop.onEnable(now);
            }
        }
    }

    public void callOnDisable() {
        if (!calledDisable) {
            calledDisable = true;
            calledEnable = false;
            isEnabled = false;
            isDisabled = true;
            double now = Timer.getFPGATimestamp();
            for (Loop loop : loops) {
                loop.onDisable(now);
            }
        }
    }

    public synchronized void start() {
        if (!running) {
            synchronized (lock) {
                timestamp = Timer.getFPGATimestamp();
                for (Loop loop : loops) {
                    loop.onFirstStart(timestamp);
                }
                running = true;
            }
            notifier.startPeriodic(Constants.Timing.LOOP_PERIOD);
        }
    }

    public synchronized void stop() {
        if (running) {
            notifier.stop();
            synchronized (lock) {
                running = false;
            }
        }
    }

    public double getDt() {
        return dt;
    }
}