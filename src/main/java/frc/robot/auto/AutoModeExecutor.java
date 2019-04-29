package frc.robot.auto;
/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 */
public class AutoModeExecutor {
    private AutoBase autoMode;
    private Thread thread = null;
    private boolean started = false;

    public void setAutoMode(AutoBase newAutoMode) {
        started = false;
        autoMode = newAutoMode;
        thread = new Thread(new Runnable(){
        
            @Override
            public void run() {
                if (autoMode != null) {
                    autoMode.run();
                }
            }
        });
    }

    public void start() {
        if (thread != null && !started) {
            thread.start();
            started = true;
        }
    }

    public void stop() {
        if (autoMode != null) {
            autoMode.stop();
            started = false;
        }

        thread = null;
    }

    public AutoBase getAutoMode() {
        return autoMode;
    }
}
