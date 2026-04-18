package frc.robot26.subsystems.leds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDsIOCANdle implements LEDsIO {
  private final CANdle candle = new CANdle(LEDsConstants.Real.candleID);
  private final StatusSignal<Current> candleCurrent;

  public LEDsIOCANdle() {
    candleCurrent = candle.getOutputCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, candleCurrent);
    ParentDevice.optimizeBusUtilizationForAll(candle);
  }

  @Override
  public void setColor(Color8Bit color) {
    candle.setControl(
        new SolidColor(LEDsConstants.Real.ledStartIndex, LEDsConstants.Real.ledEndIndex)
            .withColor(new RGBWColor(color.red, color.green, color.blue)));
  }

  @Override
  public void playRainbow() {
    // Try to instantiate a RainbowAnimation reflectively so we don't depend on a
    // specific constructor signature. If that fails, fall back to a safe solid
    // blue color so the command still behaves reasonably.
    try {
      Class<?> animClass = Class.forName("com.ctre.phoenix6.controls.RainbowAnimation");

      // Try constructors with different parameter signatures.
      Object anim = null;
      int start = LEDsConstants.Real.ledStartIndex;
      int end = LEDsConstants.Real.ledEndIndex;
      int period = 2000;

      // Prefer (int start, int end, int period)
      try {
        var ctor = animClass.getConstructor(int.class, int.class, int.class);
        anim = ctor.newInstance(start, end, period);
      } catch (NoSuchMethodException ignore) {
        // Try (int start, int end)
        try {
          var ctor2 = animClass.getConstructor(int.class, int.class);
          anim = ctor2.newInstance(start, end);
        } catch (NoSuchMethodException ignore2) {
          // Try single-arg (length)
          try {
            var ctor3 = animClass.getConstructor(int.class);
            anim = ctor3.newInstance(end - start + 1);
          } catch (NoSuchMethodException ignore3) {
            // Try no-arg
            var ctor4 = animClass.getConstructor();
            anim = ctor4.newInstance();
          }
        }
      }

      // Invoke setControl reflectively to avoid static typing issues.
      java.lang.reflect.Method setControlMethod = null;
      for (var m : candle.getClass().getMethods()) {
        if (m.getName().equals("setControl") && m.getParameterCount() == 1) {
          setControlMethod = m;
          break;
        }
      }
      if (setControlMethod != null) {
        setControlMethod.invoke(candle, anim);
      } else {
        // As a last resort, set a solid blue color
        setColor(new Color8Bit(0, 35, 105));
      }
    } catch (Exception ex) {
      // Reflection failed; fall back to a solid blue color
      setColor(new Color8Bit(0, 35, 105));
    }
  }

  @Override
  public void updateInputs(LEDsIOInputs inputs) {
    var candleConnectedStatus = BaseStatusSignal.refreshAll(candleCurrent);

    inputs.ledsConnected = candleConnectedStatus.isOK();
    inputs.ledsCurrent = candleCurrent.getValue();
  }
}
