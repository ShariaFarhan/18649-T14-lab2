"""Count debounced physical button rising edges; MCU owns fail/recover decisions."""
class SelfTestButton:
    def __init__(self,bit):
        if not 0<=bit<32:raise ValueError('Button bit must be 0..31')
        self.mask=1<<bit;self.previous=None;self.last_press=-1.;self.counter=0

    def update(self,buttons,now):
        down=bool(buttons&self.mask)
        rising=self.previous is False and down
        self.previous=down
        if rising and now-self.last_press>=.05:
            self.last_press=now;self.counter=(self.counter+1)&31;return True
        return False
