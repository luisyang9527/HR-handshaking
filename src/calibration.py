self.calibrated = False
zero_grip_human = []
zero_grip_robot = []

def loomia_msg_cb(self, msg):
        
        msg_length = len(msg.calibrated_values)
        human_force = []
        robot_force = []

        for x in range(0,12):
            for y in range(0,11):
                index = (x * 11) + y 
                if index >= msg_length:
                    break
                self.pressure_array[x,y] = msg.raw_values[index]

        hf = calibration(pressure_array, human_force, robot_force)[0]
        rf = calibration(pressure_array, human_force, robot_force)[1]

        if not self.calibrated:
            self.zero_grip_human.append(hf)
            self.zero_grip_human.append(rf)
            if len(self.zero_grip_human) == 20:
                loomia_offset_human = mean(zero_grip_human)
                loomia_offset_robot = mean(zero_grip_robot)
                self.calibrated = True

        human = hf - loomia_offset_human
        robot = rf - loomia_offset_robot

        min_grip = 0
        max_grip = max(200, human)
        human_command = 5 * (np.clip(human, min_grip, max_grip) - min_grip)/(max_grip - min_grip)

        self.grip_pressure_topic.publish(Float32(human))
        self.sliders[0].setValue(human_command)

def calibration(pressure_array, human_force, robot_force):
    for x in range(0, 9):
        for y in range(6, 11):
            if self.pressure_array[x, y] > 10 and self.pressure_array[x, y] < 1000:
                human_force.append(self.pressure_array[x, y])
    m_hf = mean(human_force)
    
    for x in range(0, 9):
        for y in range(0, 6):
            if self.pressure_array[x, y] > 10 and self.pressure_array[x, y] < 1000:
                robot_force1.append(self.pressure_array[x, y])
    for x in range(9, 12):
        for y in range(6, 11):
            if self.pressure_array[x, y] > 10 and self.pressure_array[x, y] < 1000:
                robot_force.append(self.pressure_array[x, y])
    m_rf = mean(robot_force)
    return (m_hf, m_rf)