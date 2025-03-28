package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.Constants;

import static frc.robot.util.PhoenixUtil.tryUntilOkV5;

public class LEDIOHardware implements LEDIO {
    private final CANdle candle = new CANdle(1, Constants.canivoreBusName);

    public LEDIOHardware() {
        super();

        var config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 0.1;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        tryUntilOkV5(() -> candle.configAllSettings(config, 100));
    }

    @Override
    public void animate(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void setLEDs(int r, int g, int b, int start, int count) {
        candle.setLEDs(r, g, b, 0, start, count);
    }

    @Override
    public void setBrightness(double percent) {
        candle.configBrightnessScalar(percent, 0);
    }
}
