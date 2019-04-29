package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import frc.loops.Loop;
import frc.robot.Constants;
import frc.util.Colour;

/**
 * AFrameRGBs
 */
public class AFrameRGBs extends Subsystem {

    private static AFrameRGBs instance;

    private RGBState state = RGBState.BLINK_FAST;
    private Colour colour;
    private CANifier canifier;
    private double lastBlinkTime = 0;
    private boolean isBlinkLEDOn = false;

    public enum RGBState {
        OFF, SOLID, PULSATING, BLINK_SLOW, BLINK_FAST
    }

    public AFrameRGBs() {
        colour = new Colour(0, 0, 0);
        canifier = new CANifier(Constants.RGB.CANIFIER);
    }

    public static AFrameRGBs getInstance() {
        if (instance == null) {
            instance = new AFrameRGBs();
        }
        return instance;
    }

    public void setColour(double r, double g, double b) {
        colour.setRGB(r,g,b);
    }

    public void setColour(Colour c) {
        colour.setRGB(c);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    protected Loop loop() {
        return new Loop() {

            @Override
            public void onFirstStart(double time) {
                setColour(Colour.PINK);
            }

            @Override
            public void onEnable(double time) {
                setColour(Colour.GREEN);
                setState(RGBState.SOLID);
            }

            @Override
            public void onDisable(double time) {
                setColour(Colour.RED);
                setState(RGBState.SOLID);
            }

            @Override
            public void enabledLoop(double time) {

            }

            @Override
            public void disabledLoop(double time) {

            }

            @Override
            public void allLoop(double time) {
                switch (state) {
                case OFF:
                    setCANifierLEDWithColour(Colour.BLACK);
                    break;
                case SOLID:
                    setCANifierLEDWithColour(colour);
                    break;
                case PULSATING:
                    double multiplier = 1 / 2.0 * Math.sin(time * 3) + 0.5;
                    setCANifierLEDWithColour(colour, multiplier);
                    break;
                case BLINK_SLOW:
                    if (time - lastBlinkTime > 1) {
                        lastBlinkTime = time;
                        isBlinkLEDOn = !isBlinkLEDOn;
                    }
                    if (isBlinkLEDOn) {
                        setCANifierLEDWithColour(colour, 1);
                    } else {
                        setCANifierLEDWithColour(colour, 0);
                    }
                    break;
                case BLINK_FAST:
                    if (time - lastBlinkTime > 0.3) {
                        lastBlinkTime = time;
                        isBlinkLEDOn = !isBlinkLEDOn;
                    }
                    if (isBlinkLEDOn) {
                        setCANifierLEDWithColour(colour, 1);
                    } else {
                        setCANifierLEDWithColour(colour, 0);
                    }
                    break;
                }
            }
        };
    }

    public void setState(RGBState state) {
        if(state != this.state) {
            this.state = state;
        }
    }

    private void setCANifierLEDWithColour(Colour c) {
        setCANifierLEDWithColour(c, 1);
    }

    private void setCANifierLEDWithColour(Colour c, double multiplier) {
        double[] rgb = c.getRGB();
        canifier.setLEDOutput(rgb[0] * multiplier, LEDChannel.LEDChannelA);
        canifier.setLEDOutput(rgb[1] * multiplier, LEDChannel.LEDChannelB);
        canifier.setLEDOutput(rgb[2] * multiplier, LEDChannel.LEDChannelC);
    }

}