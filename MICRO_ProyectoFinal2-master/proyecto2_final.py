from pyb import Pin, Timer, ADC
import pyb
from time import sleep
import micropython
import machine 


micropython.alloc_emergency_exception_buf(100)

p = Pin('PA0') # PA0 has TIM2, CH1
b= Pin('PA9',Pin.IN)
pyb.freq(84000000)
tim = pyb.Timer(5,prescaler=1679, period=999)

adc = pyb.ADC('PA1')                  # create an analog object from a pin
ch = tim.channel(1, Timer.PWM, pin=p)
grados=0
val=0
tick=0
def callback(tim2):
    global tick
    global val
    if tick >= 1679:
        val = adc.read()  
        print('el valor es ',val)
        tick=0
       
    tick=tick+1




tim2=pyb.Timer(4)
tim2.init(freq=12000)
tim2.callback(callback)
   
    

while(True):
    if b.value()==0:
        formula=(val*100)/4095
        grados=formula+25
        ch.pulse_width(int(grados))
        pyb.delay(5)
    elif b.value()==1:
        grados=25
        ch.pulse_width(int(grados))
        sleep(5)
    
