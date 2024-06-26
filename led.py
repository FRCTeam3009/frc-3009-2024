import wpilib
import wpilib.simulation
import constants

#for led in range(self.ledLength):
#    led_data = self.ledBuff[led]
#    if self.ledLit % self.ledLength == led:
#        led_data.setRGB(250,0,0)
#    else:
#        led_data.setRGB(0,0,250)
#self.ledLit += 1
#self.leds.setData(self.ledBuff)

class color(object):
    def __init__(self, red, green, blue) -> None:
        self.red = red
        self.green = green
        self.blue = blue

kOff = color(0, 0, 0)
kWhite = color(255, 255, 255)
kBumperBlue = color(72, 12, 201)
kBumperRed = color(244, 24, 28)
kHighScalersYellow = color(250, 200, 1)
kNoteOrange = color(253, 69, 0)

kRed = color(255, 0, 0)
kOrange = color(255, 127, 0)
kYellow = color(255, 255, 0)
kGreen = color(0, 255, 0)
kBlue = color(0, 0, 255)
kPink = color(255, 0, 255)
kPurple = color(102, 0, 102)

kRainbow = [kRed, kOrange, kYellow, kGreen, kBlue, kPurple]

def setLedColor(led_data: wpilib.AddressableLED.LEDData, c: color) -> None:
    """setRGB actually uses GRB ordering, so this handles the re-ordering for us."""
    led_data.setRGB(c.green, c.red, c.blue)

class LedStrips(object):

    def __init__(self) -> None:
        self.leds = wpilib.AddressableLED(constants.LEDs)
        self.ledLength = 151
        self.leds.setLength(self.ledLength)
        
        self.setup_buffer()
       
        self.leds.start()

        self.chaseOffset = 0

    def setup_buffer(self):
        self.ledBuff = []

        for led in range(self.ledLength):
            data = self.leds.LEDData()
            setLedColor(data, kOff)
            self.ledBuff.append(data)
        self.leds.setData(self.ledBuff)

    def solid(self, color):
        for led in self.ledBuff:
            setLedColor(led, color)
        self.leds.setData(self.ledBuff)

    def solidRGB(self, red, green, blue):
        c = color(red, green, blue)
        self.solid(c)

    def gbRotate(self):
        i = 0
        while i < self.ledLength:
            led = self.ledBuff[i]
            if i % 2 == 0:
                setLedColor(led, kGreen)
            else:
                setLedColor(led, kBlue)
            i = i + 1
        self.leds.setData(self.ledBuff)

    def chase(self, colors):
        i = 0
        while i < len(self.ledBuff):
            led = self.ledBuff[i]
            start = self.chaseOffset
            end = self.chaseOffset + len(colors)
            if i >= start and i < end:
                colorIndex = i - self.chaseOffset
                setLedColor(led, colors[colorIndex])
            else:
                setLedColor(led, kWhite)
            i += 1

        self.chaseOffset += 1
        if self.chaseOffset > len(self.ledBuff):
            self.chaseOffset = 0
