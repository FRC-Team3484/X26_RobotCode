from wpilib import Color, AddressableLED, LEDPattern, Timer
from frc3484.leds import ColorStack, ColorWave, FallingSand, Fire, Static, correct_gamma
from constants import LEDSubsystemConstants
import random

from enum import Enum

class PatternState(Enum):
    fill = 0,
    empty = 1

class LEDSubsystem:
    def __init__(self) -> None:
        self._solid_algae = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.ALGAE_GREEN_X25))
        self._solid_coral = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.CORAL_PINK_X25))
        self._solid_orange = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.DRIVE_ORANGE_X25))
        self._solid_team = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.TEAM_BLUE_X25))
        self._solid_fire = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.FIRE_RED_X25))
        self._solid_snow = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.SNOW_WHITE_X26))
        self._solid_ice = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.ICE_BLUE_X26))
        self._solid_charged = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.CHARGED_GREEN_X26))
        self._solid_static = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.STATIC_YELLOW_X26))
        self._solid_ancient = LEDPattern.solid(correct_gamma(self, LEDSubsystemConstants.ANCIENT_PURPLE_X26))

        self._pivot_animation_progress = int
        self._progress_mask = LEDPattern.progressMaskLayer(self._pivot_animation_progress)
        self._step_mask: list[tuple[float, Color]] = [(0.0, Color.kWhite), (0.3, Color.kBlack)]
        self._green_scroll_step = LEDPattern.steps(self._step_mask).scrollAtAbsoluteSpeed(LEDSubsystemConstants.GREEN_SCROLL_SPEED, LEDSubsystemConstants.LED_SPACING)
        self._fusion_scroll_step = LEDPattern.steps(self._step_mask).scrollAtAbsoluteSpeed(LEDSubsystemConstants.FUSION_SCROLLING_SPEED, LEDSubsystemConstants.LED_SPACING)
        self._combined_colors = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, LEDSubsystemConstants.COLOR_FUSION)
        self._test_color = random.choice(LEDSubsystemConstants.COLORS)

        self._solid_test = LEDPattern.solid(correct_gamma(self, self._test_color))
        self._step_fusion = self._combined_colors.mask(self._fusion_scroll_step)
        self._step_green = self._solid_charged.mask(self._green_scroll_step)
        self._test_step = random.choice(LEDSubsystemConstants.COLORS)
        self._purple_blink = self._solid_ancient.breathe(LEDSubsystemConstants.PURPLE_CYCLE_TIME)
        self._progress_purple = self._solid_ancient.mask(self._progress_mask)
        self._progress_blue = self._solid_ice.mask(self._progress_mask)

        self._colorwave = ColorWave(LEDSubsystemConstants.COLOR_WAVE_COLORS, LEDSubsystemConstants.LED_SPACING, LEDSubsystemConstants.WAVELENGTH, LEDSubsystemConstants.GAMMA, LEDSubsystemConstants.VELOCITY, LEDSubsystemConstants.BAR_SIZE)
        self._colorstack = ColorStack(LEDSubsystemConstants.COLOR_WAVE_COLORS, LEDSubsystemConstants.BAR_SIZE, LEDSubsystemConstants.VELOCITY, LEDSubsystemConstants.FILL_SIZE, LEDSubsystemConstants.EMPTY_SIZE, LEDSubsystemConstants.GAMMA)
        self._sand = FallingSand(LEDSubsystemConstants.COLOR_WAVE_COLORS, LEDSubsystemConstants.BAR_SIZE, LEDSubsystemConstants.LED_SPACING, LEDSubsystemConstants.INTAKE_VELOCITY, LEDSubsystemConstants.EXIT_ACCELERATION, LEDSubsystemConstants.FILL_SIZE, LEDSubsystemConstants.GAMMA)
        self._fire = Fire(LEDSubsystemConstants.FIRE_HEIGHT, LEDSubsystemConstants.FIRE_SPARKS, LEDSubsystemConstants.DELAY, LEDSubsystemConstants.FIRE_N_LEDS)
        self._static = Static(LEDSubsystemConstants.STATIC_COLOR, LEDSubsystemConstants.BAR_SIZE, LEDSubsystemConstants.LED_SPACING, LEDSubsystemConstants.FILL_SIZE, LEDSubsystemConstants.GAMMA)

        self._leds: AddressableLED = AddressableLED(LEDSubsystemConstants.LED_PWM_PORT)
        self._led_buffer: list[AddressableLED.LEDData] = [AddressableLED.LEDData() for i in range(LEDSubsystemConstants.LED_STRIP_LENGTH)]
        self._timer = Timer()
        self._leds.setData(self._led_buffer)
        self._leds.start()
        self._timer.start()

    def periodic(self):
        pass

    def ColorWaveAnimation(self):
        self._led_buffer = self._colorwave._apply_to(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def ColorStackAnimation(self):
        self._led_buffer = self._colorstack._apply_to(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def FallingSandAnimation(self):
        self._led_buffer = self._sand._apply_to(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def LowBatteryAnimation(self):
        self._led_buffer = self._fire.apply_to(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def IntakeAnimation(self):
        self._led_buffer = self._static._apply_to(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def TurretScoreAnimation(self):
        self._purple_blink.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def TurretLoopAnimation(self):
        self._progress_purple.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def DrivingAnimation(self):
        self._solid_ice.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def AutonAnimation(self):
        self._step_fusion.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def DynamicPivotAnimation(self):
        self._progress_blue.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def ClimbAnimation(self):
        self._step_green.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)
    def TestAnimation(self):
        self._solid_test.applyTo(self._led_buffer)
        self._leds.setData(self._led_buffer)