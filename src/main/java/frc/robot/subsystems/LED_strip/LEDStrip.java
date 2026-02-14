package frc.robot.subsystems.LED_strip;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDStrip extends SubsystemBase {

  private LEDStripIO ledStripIO;

  private Timer timer;

  private boolean orange = false;

  // private SolidColor orangeColor = new SolidColor(0, 80);
  // private SolidColor whiteColor = new SolidColor(0, 80);

  public static class LEDStripAnimations {
    public static final ColorFlowAnimation m_slot0Animation =
        new ColorFlowAnimation(0, 7)
            .withSlot(0)
            .withColor(new RGBWColor(255, 179, 26, 0))
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(Hertz.of(25));
  }

  public LEDStrip(LEDStripIO ledStripIO) {
    this.ledStripIO = ledStripIO;
    timer = new Timer();
    timer.reset();
    timer.start();

    // orangeColor.Color = new RGBWColor(Color.kOrange);

    // whiteColor.Color = new RGBWColor(Color.kWhite);
  }

  @Override
  public void periodic() {

    // if (timer.advanceIfElapsed(1)) {
    //   if (orange) {
    //     ledStripIO.setColor(orangeColor);
    //   } else {
    //     ledStripIO.setColor(whiteColor);
    //   }

    //   orange = !orange;
    //   System.out.println("running switch");
    // }
  }

  public void setAnimation(ControlRequest animation) {
    ledStripIO.setAnimation(animation);
  }

  int colorIndex = 0;

  public Command flashBetweenColorsCommand(double timeDelay, RGBWColor... color) {
    colorIndex = 0;
    new InstantCommand() {};

    return new InstantCommand(
            () -> {
              ledStripIO.setColor(color[colorIndex]);
              colorIndex += 1;
              if (colorIndex == color.length) {
                colorIndex = 0;
              }
            },
            this)
        .andThen(new WaitCommand(timeDelay));
  }
}
