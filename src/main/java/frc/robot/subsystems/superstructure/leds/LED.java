package frc.robot.subsystems.superstructure.leds;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.util.PhoenixUtil.tryUntilOkV5;

public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(1, Constants.canivoreBusName);
    private final int LedCount = 62;

    private Animation toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }

    private AnimationTypes currentAnimation;

    public LED() {
        changeAnimation(AnimationTypes.SetAll);
        var config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 0.1;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        tryUntilOkV5(() -> candle.configAllSettings(config, 100));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setLEDsToWhite();
        } else if (toAnimate == null) {
            setLEDsToWhite();
        } else {
            candle.animate(toAnimate);
        }
    }

    void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    public void configBrightness(double percent) {
        candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        candle.configStatusLedState(offWhenActive, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {
        currentAnimation = toChange;

        switch (toChange) {
            case ColorFlow:
                toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                toAnimate = null;
                break;
        }
        System.out.println("Changed to " + currentAnimation.toString());
    }

    public void setLEDsToWhite() {
        candle.setLEDs(255, 255, 255, 0, 0, LedCount);
    }
}
