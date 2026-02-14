package frc.robot.subsystems.LED_strip;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class LEDStripTalonFXIO implements LEDStripIO {
  private CANdle candle;
  private int length;

  // for animation
  // private final CANBus kCANBus;
  // private final CANdle m_candle;
  // private final ColorFlowAnimation m_slot0Animation;

  public LEDStripTalonFXIO(int id, CANBus canbus, int length) {
    // for blinking
    this.candle = new CANdle(id, canbus);
    this.length = length;

    // for animation
    // kCANBus = new CANBus("rio");
    // m_candle = new CANdle(id, kCANBus);
    // m_slot0Animation =
    //     new ColorFlowAnimation(0, 7)
    //         .withSlot(0)
    //         .withColor(new RGBWColor(255, 179, 26, 0))
    //         .withDirection(AnimationDirectionValue.Forward)
    //         .withFrameRate(Hertz.of(25));
  }

  @Override
  public void setColor(RGBWColor color) {
    SolidColor solidColor = new SolidColor(0, length).withColor(color);
    candle.setControl(solidColor);
  }

  @Override
  public void setAnimation(ControlRequest animation) {
    candle.setControl(animation);
  }
}
