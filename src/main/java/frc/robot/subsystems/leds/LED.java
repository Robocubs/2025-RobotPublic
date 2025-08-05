package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.ClimbState;

public class LED extends SubsystemBase {
    private static final int ledCount = 62;
    private static final int maxBrightness = 128;

    private final LEDIO io;
    private final RobotState robotState;

    private Animation toAnimate = null;
    private Color color = Color.kWhite;
    private AnimationType animationType = null;

    public enum AnimationType {
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

    public LED(LEDIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
        changeAnimation(AnimationType.SetAll, Color.kWhite);
    }

    @Override
    public void periodic() {
        if (robotState.getClimbState() == ClimbState.ZEROING) {
            changeAnimation(AnimationType.SetAll, Color.kYellow);
        } else if (robotState.getClimbState() == ClimbState.DEPLOYING) {
            changeAnimation(AnimationType.SetAll, Color.kRed);
        } else if (robotState.getClimbState() == ClimbState.DEPLOYED) {
            changeAnimation(AnimationType.SetAll, Color.kBlue);
        } else if (robotState.getClimbState() == ClimbState.RETRACTING) {
            changeAnimation(AnimationType.Strobe, Color.kBlue);
        } else if (robotState.getClimbState() == ClimbState.RETRACTED) {
            changeAnimation(AnimationType.Rainbow, Color.kBlack);
        } else if (robotState.hasCoralLoaded()) {
            changeAnimation(AnimationType.SetAll, Color.kBlue);
        } else if (robotState.hasCoralLoading()) {
            changeAnimation(AnimationType.Strobe, Color.kBlue);
        } else if (robotState.hasAlgae() && robotState.elevatorBlocked()) {
            changeAnimation(AnimationType.SetAll, Color.kRed);
        } else if (robotState.hasAlgae()) {
            changeAnimation(AnimationType.SetAll, Color.kGreen);
        } else {
            changeAnimation(AnimationType.SetAll, Color.kWhite);
        }

        if (toAnimate != null) {
            io.animate(toAnimate);
        } else {
            io.setLEDs(
                    (int) (color.red * maxBrightness),
                    (int) (color.green * maxBrightness),
                    (int) (color.blue * maxBrightness),
                    0,
                    ledCount);
        }
    }

    private void changeAnimation(AnimationType animation, Color color) {
        if (animationType == animation && this.color == color) {
            return;
        }

        animationType = animation;
        this.color = color;

        switch (animation) {
            case ColorFlow:
                toAnimate = new ColorFlowAnimation(
                        (int) (color.red * maxBrightness),
                        (int) (color.green * maxBrightness),
                        (int) (color.blue * maxBrightness),
                        0,
                        0.7,
                        ledCount,
                        Direction.Forward);
                break;
            case Fire:
                toAnimate = new FireAnimation(0.5, 0.7, ledCount, 0.7, 0.5);
                break;
            case Larson:
                toAnimate = new LarsonAnimation(
                        (int) (color.red * maxBrightness),
                        (int) (color.green * maxBrightness),
                        (int) (color.blue * maxBrightness),
                        0,
                        1,
                        ledCount,
                        BounceMode.Front,
                        3);
                break;
            case Rainbow:
                toAnimate = new RainbowAnimation(1, 0.1, ledCount);
                break;
            case RgbFade:
                toAnimate = new RgbFadeAnimation(0.7, 0.4, ledCount);
                break;
            case SingleFade:
                toAnimate = new SingleFadeAnimation(
                        (int) (color.red * maxBrightness),
                        (int) (color.green * maxBrightness),
                        (int) (color.blue * maxBrightness),
                        0,
                        0.5,
                        ledCount);
                break;
            case Strobe:
                toAnimate = new StrobeAnimation(
                        (int) (color.red * maxBrightness),
                        (int) (color.green * maxBrightness),
                        (int) (color.blue * maxBrightness),
                        0,
                        98.0 / 256.0,
                        ledCount);
                break;
            case Twinkle:
                toAnimate = new TwinkleAnimation(
                        (int) (color.red * maxBrightness),
                        (int) (color.green * maxBrightness),
                        (int) (color.blue * maxBrightness),
                        0,
                        0.4,
                        ledCount,
                        TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                toAnimate = new TwinkleOffAnimation(
                        (int) (color.red * maxBrightness),
                        (int) (color.green * maxBrightness),
                        (int) (color.blue * maxBrightness),
                        0,
                        0.8,
                        ledCount,
                        TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                toAnimate = null;
                break;
        }
    }
}
