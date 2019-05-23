import time

import fflyusb_wrapper


def main():
    x10i = fflyusb_wrapper.PyFireFlyUSB()
    x10i.init(0)
    x10i.UnlockHardware()
    x10i.GetProductVersion()
    print(f'Product_version : {x10i.ProductVersion}')

    port = 'PORT_A'
    ptype = 'PORT_CCTALK_MODE1'
    config = {
        'BaudRate': 9600,
        'Parity': 0
    }
    print(f'SetConfig: {x10i.SetConfig(port, config, ptype)}')
    poll_method = 'Once'
    cct_config_1 = {
        'device_number': 0,
        'poll_retry_count': 1,  # over
        'polling_interval': 100,
        'max_response_time': 100,
        'min_buffer_space': 100,
        'poll_msg': bytearray([2, 2, 1, 231, 1, 1, 18] + [0] * 18),
        'inhibit_msg': bytearray([2, 2, 1, 231, 1, 1, 18] + [0] * 18)
    }
    print(f'cct config1: {x10i.ConfigureCCTalkPort(port, cct_config_1, poll_method)}')
    poll_method = 'Repeated'
    cct_config_2 = {
        'device_number': 0,
        'poll_retry_count': 1,  # over
        'polling_interval': 200,
        'max_response_time': 2000,
        'min_buffer_space': 100,
        'poll_msg': bytearray([2, 0, 1, 229, 24] + [0] * 20),
        'inhibit_msg': bytearray([2, 0, 1, 229, 24] + [0] * 20)
    }
    time.sleep(2)
    print(f'cct config2: {x10i.ConfigureCCTalkPort(port, cct_config_2, poll_method)}')

    chk = None
    while True:
        time.sleep(0.1)
        rsp = x10i.ReceivePolledMessage(port, 0)
        x10i.DeletePolledMessage(port, 0)

        if rsp[1:] != chk:
            chk = rsp[1:]
            print(rsp[0])
            print(rsp[1])
            print(rsp[2])
    print('sleeping')


if __name__ == '__main__':
    # profile.run('main()')
    main()
    time.sleep(10000000)
