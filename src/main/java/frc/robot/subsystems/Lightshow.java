package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Lightshow class represents the subsystem responsible for controlling the
 * LED light strip.
 */
public class Lightshow extends SubsystemBase {
    private static final int kNumLeds = 60; // Replace with your total number of LEDs
    private static final int kPwmPort = 5; // Replace with your PWM port number

    private static Lightshow instance = null;

    public static final Color kRed = new Color(1.0, 0.0, 0.0);
    public static final Color kBlue = new Color(0.0, 0.0, 1.0);
    public static final Color kGreen = new Color(0.0, 1.0, 0.0);
    public static final Color kYellow = new Color(1.0, 1.0, 0.0);
    public static final Color kMagenta = new Color(1.0, 0.0, 1.0);
    public static final Color kCyan = new Color(0.0, 1.0, 1.0);
    public static final Color kWhite = new Color(1.0, 1.0, 1.0);
    public static final Color kOrange = new Color(1.0, 0.5, 0.0);
    public static final Color kPink = new Color(1.0, 0.75, 0.8);
    public static final Color kLime = new Color(0.75, 1.0, 0.0);
    public static final Color kLightGray = new Color(0.75, 0.75, 0.75);
    public static final Color kDarkGray = new Color(0.25, 0.25, 0.25);
    public static final Color kMediumGray = new Color(0.5, 0.5, 0.5);
    public static final Color kVeryLightGray = new Color(0.9, 0.9, 0.9);
    public static final Color kVeryDarkGray = new Color(0.1, 0.1, 0.1);
    public static final Color kLightBlue = new Color(0.5, 0.5, 1.0);
    public static final Color kDarkBlue = new Color(0.0, 0.0, 0.5);
    public static final Color kLightGreen = new Color(0.5, 1.0, 0.5);
    public static final Color kDarkGreen = new Color(0.0, 0.5, 0.0);
    public static final Color kLightRed = new Color(1.0, 0.5, 0.5);

    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    
    /**
     * The Lightshow class represents the subsystem responsible for controlling the
     * LED light strip.
     */
    private Lightshow() {
        ledStrip = new AddressableLED(kPwmPort);
        ledBuffer = new AddressableLEDBuffer(kNumLeds);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
    }


    /**
     * Returns the instance of the Lightshow class.
     * If the instance does not exist, it creates a new instance and returns it.
     * 
     * @return The instance of the Lightshow class.
     */
    public static Lightshow getInstance() {
        if (instance == null) {
            instance = new Lightshow();
        }
        return instance;
    }

    /**
     * Sets the colors of the LED strip.
     * If the length of the colors array is equal to the number of LEDs, each LED
     * will be set to the corresponding color.
     * If the length of the colors array is half the number of LEDs, the first half
     * of the LEDs will be set to the corresponding colors,
     * and the second half of the LEDs will be set to the colors in reverse order.
     *
     * @param colors an array of Color objects representing the colors to set for
     *               the LEDs
     */
    public void setColors(Color[] colors) {
        if (colors.length == kNumLeds) { // If the length of the colors array is equal to the number of LEDs
            for (int i = 0; i < kNumLeds; i++) {
                ledBuffer.setLED(i, colors[i]);
            }
        } else if (colors.length == kNumLeds / 2) { // If the length of the colors array is half the number of LEDs
            for (int i = 0; i < kNumLeds / 2; i++) {
                ledBuffer.setLED(i, colors[i]);
                ledBuffer.setLED(kNumLeds - 1 - i, colors[i]);
            }
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Returns the total number of lights in the lightshow.
     *
     * @return The total number of lights.
     */
    public int getTotalLights() {
        return kNumLeds;
    }
}