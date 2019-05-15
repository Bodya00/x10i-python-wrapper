import profile
# pip install pythonbenchmark
from pythonbenchmark import compare, measure
import fflyusb_wrapper
import time

@measure
def CachedReadAndResetSecuritySwitchFlags(x10i):
    x10i.CachedReadAndResetSecuritySwitchFlags()

def test_something(*args,**kwargs):
   print('changes')

def main():

    x10i = fflyusb_wrapper.PyFireFlyUSB()
    x10i.init(0)
    print(x10i.ConfigureChangedInputsCallback(test_something))
if __name__ == '__main__':
    #profile.run('main()')
    main()
    time.sleep(10000000)
