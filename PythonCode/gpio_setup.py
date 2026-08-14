"""Point gpiozero at the right gpiochip for this kernel.

gpiozero's own auto-detection assumes the Pi's GPIO controller is
/dev/gpiochip0 (or gpiochip4 on the Pi 5). Kernel 6.12 renumbered the
chardev nodes -- on a Pi 5 running 6.12 the RP1 controller shows up as
gpiochip15 -- so that guess fails with 'can not open gpiochip', gpiozero
silently falls back to the RPi.GPIO backend, and *that* dies on the Pi 5
with 'Cannot determine SOC peripheral base address'.

Rather than hard-coding another number that a future kernel can invalidate,
find the chip by its label and pin gpiozero to it.

Import this before creating any gpiozero device:

    import gpio_setup  # noqa: F401
"""

import glob
import os
import re

# Labels of the SoC pin controllers, newest first: RP1 is the Pi 5's GPIO
# chip, pinctrl-bcm* covers Pi 4 and earlier.
_PINCTRL_PATTERN = re.compile(r"^pinctrl-(rp1|bcm\d+)$")


def find_pinctrl_chip():
    """Return the gpiochip number driving the Pi's 40-pin header, or None."""
    try:
        import lgpio
    except ImportError:
        return None

    numbers = sorted(
        int(m.group(1))
        for path in glob.glob("/dev/gpiochip*")
        for m in [re.search(r"(\d+)$", path)]
        if m
    )

    for number in numbers:
        try:
            handle = lgpio.gpiochip_open(number)
        except Exception:  # noqa: BLE001 - chips we can't open simply aren't it
            continue
        try:
            label = lgpio.gpio_get_chip_info(handle)[3]
        except Exception:  # noqa: BLE001
            label = ""
        finally:
            lgpio.gpiochip_close(handle)

        if _PINCTRL_PATTERN.match(label):
            return number
    return None


def configure():
    """Set gpiozero's default pin factory. Returns the chip number used."""
    # An explicit override always wins, so this can't get in the way of e.g.
    # GPIOZERO_PIN_FACTORY=mock in tests.
    if os.environ.get("GPIOZERO_PIN_FACTORY"):
        return None

    chip = find_pinctrl_chip()
    if chip is None:
        return None

    try:
        from gpiozero import Device
        from gpiozero.pins.lgpio import LGPIOFactory

        if Device.pin_factory is None:
            Device.pin_factory = LGPIOFactory(chip=chip)
        return chip
    except Exception as exc:  # noqa: BLE001 - leave gpiozero's own default in place
        print(f"[gpio_setup] could not select gpiochip{chip}: {exc}")
        return None


configure()


if __name__ == "__main__":
    print("pinctrl gpiochip:", find_pinctrl_chip())
