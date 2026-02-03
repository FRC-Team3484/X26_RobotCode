from wpilib import Color, AddressableLED, LEDPattern, Timer
from frc3484.leds import ColorStack, ColorWave, FallingSand, Fire, Static, correct_gamma
from constants import LEDSubsystemConstants

from enum import Enum

class PatternState(Enum):
    fill = 0,
    empty = 1

class LEDSubsystem:
    def __init__(self, _led_pwm_port: int, led_strip_length: int) -> None:
        self._leds = _led_pwm_port
        self._solid_algae = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.ALGAE_GREEN_X25, LEDSubsystemConstants.GAMMA))
        self._solid_coral = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.CORAL_PINK_X25, LEDSubsystemConstants.GAMMA))
        self._solid_orange = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.DRIVE_ORANGE_X25, LEDSubsystemConstants.GAMMA))
        self._solid_team = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.TEAM_BLUE_X25, LEDSubsystemConstants.GAMMA))
        self._solid_fire = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.FIRE_RED_X25, LEDSubsystemConstants.GAMMA))
        self._solid_snow = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.SNOW_WHITE_X26, LEDSubsystemConstants.GAMMA))
        self._solid_ice = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.ICE_BLUE_X26, LEDSubsystemConstants.GAMMA))
        self._solid_charged = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.CHARGED_GREEN_X26, LEDSubsystemConstants.GAMMA))
        self._solid_static = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.STATIC_YELLOW_X26, LEDSubsystemConstants.GAMMA))
        self._solid_ancient = LEDPattern.solid(correct_gamma(LEDSubsystemConstants.ANCIENT_PURPLE_X26, LEDSubsystemConstants.GAMMA))


        self._pivot_animation_progress = int
        self._progress_mask = LEDPattern.progressMaskLayer(self._pivot_animation_progress())
        self._step_mask = list[tuple[float, Color]] = [(0.0, Color.kWhite), (0.3, Color.kBlack)]
        self._scroll_step = LEDPattern.steps(self._step_mask).scrollAtAbsoluteSpeed(LEDSubsystemConstants.SCROLLING_SPEED, LEDSubsystemConstants.LED_SPACING)
        self._combined_colors = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, LEDSubsystemConstants.COLOR_FUSION)


        self._step_fusion = self._combined_colors.scrollAtAbsoluteSpeed(LEDSubsystemConstants.SCROLLING_SPEED, LEDSubsystemConstants.LED_SPACING)
        self._step_green = self._solid_charged.mask(self._step_mask)
        self._purple_blink = self._solid_ancient.breathe(LEDSubsystemConstants.PURPLE_CYCLE_TIME)
        self._progress_purple = self._solid_ancient.mask(self._progress_mask)
        self._progress_blue = self._solid_ice.mask(self._progress_mask)

        self._colorwave = ColorWave(LEDSubsystemConstants.COLOR_WAVE_COLORS, LEDSubsystemConstants.LED_SPACING, LEDSubsystemConstants.WAVELENGTH, LEDSubsystemConstants.GAMMA, LEDSubsystemConstants.VELOCITY, LEDSubsystemConstants.BAR_SIZE)
        self._colorstack = ColorStack(LEDSubsystemConstants.COLOR_WAVE_COLORS, LEDSubsystemConstants.BAR_SIZE, LEDSubsystemConstants.VELOCITY, LEDSubsystemConstants.FILL_SIZE, LEDSubsystemConstants.EMPTY_SIZE, LEDSubsystemConstants.GAMMA)
        self._sand = FallingSand(LEDSubsystemConstants.COLOR_WAVE_COLORS, LEDSubsystemConstants.BAR_SIZE, LEDSubsystemConstants.LED_SPACING, LEDSubsystemConstants.INTAKE_VELOCITY, LEDSubsystemConstants.EXIT_ACCELERATION, LEDSubsystemConstants.FILL_SIZE, LEDSubsystemConstants.GAMMA)
        self._fire = Fire(LEDSubsystemConstants.FIRE_HEIGHT, LEDSubsystemConstants.FIRE_SPARKS, LEDSubsystemConstants.DELAY, LEDSubsystemConstants.FIRE_N_LEDS)
        self._static = Static(LEDSubsystemConstants.STATIC_YELLOW_X26, LEDSubsystemConstants.BAR_SIZE, LEDSubsystemConstants.LED_SPACING, LEDSubsystemConstants.FILL_SIZE, LEDSubsystemConstants.GAMMA, LEDSubsystemConstants.MOVE_RATE)

        self._leds = LEDSubsystemConstants.LED_PWM_PORT
        self._led_buffer = list(LEDSubsystemConstants.LED_STRIP_LENGTH, AddressableLED.LEDData)
        self._timer = Timer.start()

    def periodic(self):
        pass

    def ColorWaveAnimation(self):
        self._colorwave.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def ColorStackAnimation(self):
        self._colorstack.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def FallingSandAnimation(self):
        self._sand.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def LowBatteryAnimation(self):
        self._fire.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def DrivingAnimation(self):
        self._solid_ice.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def AutonAnimation(self):
        self._step_fusion.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def DynamicPivotAnimation(self):
        pass