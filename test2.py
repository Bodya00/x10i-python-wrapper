import time

import fflyusb_wrapper


def main():
    x10i = fflyusb_wrapper.PyFireFlyUSB()
    x10i.init(0)
    x10i.UnlockHardware()
    x10i.GetProductVersion()
    print(x10i.ProductVersion)

    port = 'PORT_A'
    ptype = 'PORT_CCTALK_MODE1'
    config = {
        'BaudRate': 9600,
        'Parity': 'NOPARITY'
    }
    print(x10i.SetConfig(port, config, ptype))


if __name__ == '__main__':
    # profile.run('main()')
    main()
    time.sleep(10000000)
