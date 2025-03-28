package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;

public interface LEDIO {
    public default void animate(Animation animation) {}

    public default void setLEDs(int r, int g, int b, int start, int count) {}

    public default void setBrightness(double percent) {}
}
