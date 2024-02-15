import wpilib


#for led in range(self.ledLength):
#    led_data = self.ledBuff[led]
#    if self.ledLit % self.ledLength == led:
#        led_data.setRGB(250,0,0)
#    else:
#        led_data.setRGB(0,0,250)
#self.ledLit += 1
#self.leds.setData(self.ledBuff)

class ledStrips(object):
    def __init__(self) -> None:
        self.leds = wpilib.AddressableLED(5)
        self.ledLength = 151
        self.leds.setLength(self.ledLength)
        self.ledBuff = []
        for led in range(self.ledLength):
            self.ledBuff.append(self.leds.LEDData())
            led_data = self.ledBuff[led]
            led_data.setRGB(0,0,0)
        self.leds.setData(self.ledBuff)
        self.leds.start()


    def solid(self, red, green, blue):
        for led in self.ledBuff:
            led.setRGB(green, red, blue)
        self.leds.setData(self.ledBuff)