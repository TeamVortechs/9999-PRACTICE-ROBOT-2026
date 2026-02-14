package frc.robot.subsystems.LED_strip;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.RGBWColor;

public interface LEDStripIO {

  public default void setColor(RGBWColor color) {}
  ;

  public default void setAnimation(ControlRequest animation) {}
  ;
}
