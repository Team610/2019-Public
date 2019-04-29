package frc.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class TimedRumble {

    private Joystick controller;
    private double startTime;
    private double duration;
    private double intensity;
    private boolean isDone;

    /**
     * 
     * @param joystick  The instance of the joystick to rumble
     * @param duration  Length of rumble in milliseconds
     * @param intensity Power of the rumble [0,1]
     */
    public TimedRumble(Joystick joystick, double duration, double intensity) {
        this.controller = joystick;
        this.duration = duration;
        this.intensity = intensity;
        this.isDone = true;
    }

    public void set(Joystick joystick, double duration, double intensity) {
        this.controller = joystick;
        this.duration = duration;
        this.intensity = intensity;
        this.isDone = true;
    }

    /**
     * Will not restart the rumble until the last rumble has completed. To override
     * the last rumble call interrupt before calling start.
     */
    public void start() {
        if (isDone) {
            this.startTime = System.currentTimeMillis();
            isDone = false;
            controller.setRumble(RumbleType.kLeftRumble, intensity);
            controller.setRumble(RumbleType.kRightRumble, intensity);
        }
    }

    /**
     * Must be called periodically.
     */
    public void update() {
        double deltaTime = System.currentTimeMillis() - this.startTime;
        if (deltaTime < this.duration) {
            controller.setRumble(RumbleType.kLeftRumble, intensity);
            controller.setRumble(RumbleType.kRightRumble, intensity);
        } else {
            interrupt();
        }
    }

    public void interrupt() {
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
        isDone = true;
    }

}