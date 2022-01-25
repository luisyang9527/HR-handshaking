from PID_controller import*

def test_pid(p, i, d, actual, desired, len):
    pid = PID(p, i, d)
    pid.set_sample_time(0.01)

    for i in range(len):
        pid.update(actual, desired)
        actual += pid.output
        time.sleep(0.01)

    return actual