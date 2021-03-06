import matplotlib.pyplot as plt
import numpy as np
import math

dimension = 6

SpeedTable = [[0]for i in range(dimension)]
ThetaTable = [[0]for i in range(dimension)]
AccTable = [[0]for i in range(dimension)]

RealPosition = np.zeros(dimension).reshape(-1,1)
RealSpeed = np.zeros(dimension).reshape(-1,1)
RealEffort = np.zeros(dimension).reshape(-1,1)

def get_distance(x1, x2):
    x1 = np.array(x1)
    x2 = np.array(x2)
    temp = x1 - x2
    temp2 = temp.reshape(temp.shape[0],1)
    return np.abs(temp).max(0)

def get_min(target, pose1, pose2):
    value = np.zeros(pose1.shape[0])
    index = np.zeros(pose1.shape[0])
    for i in range(len(pose2)):
        temp = 999999999
        for j in range(len(pose1[0])):
            tmp = target[j] + get_distance(pose1[j], pose2[i])
            if tmp < temp:
                index[i] = j
                temp = tmp
        value[i] = temp
    return index, value


#到下一目标点的最短路径动态规划
def dynamic_programming(pose):
    target = np.zeros(pose.shape[:-1])
    result = np.zeros(pose.shape[:-1])
    for i in range(1,len(pose)):
        index, value = get_min(target[i-1], pose[i-1],pose[i])
        target[i] = value
        result[i] = index
    print(target)
    print(result)

'''
    Zero speed to point
'''
def tixing(start_position, target_position, a, velocity_threshold, delta,t):

    if delta == 0 and t != 0:
        current_position = np.zeros(start_position.shape)
        expected_time = np.zeros(start_position.shape)
        for i in range(len(start_position)):
            current_position[i],expected_time[i] = tixing_1d(start_position[i], target_position[i], a[i], velocity_threshold[i], i,delta,t)
        return current_position,np.max(expected_time)

    elif t == 0 and delta != 0:
        time_expected = np.zeros(start_position.shape)
        for i in range(len(start_position)):
            time_expected[i] = tixing_1d(start_position[i], target_position[i], a[i], velocity_threshold[i], i,delta,t)
        for i in range(len(start_position)):
            for j in range(len(SpeedTable[i]),len(SpeedTable[np.argmax(time_expected)])+1):
                SpeedTable[i].append(0)
                ThetaTable[i].append(ThetaTable[i][-1])
                AccTable[i].append(0)

        return np.max(time_expected),ThetaTable

    else:
        return 0



def tixing_1d(start_position, target_position, a, velocity_limit, index, delta,t):

    distance = abs(target_position - start_position)
    #print(delta)
    #print(t)
    #等控制周期dt下的梯形规划，返回table
    if t == 0 and delta != 0 :
        flag = 1
        if target_position - start_position < 0:
            flag = -1
        if distance >= velocity_limit ** 2 / a:
            expected_time = velocity_limit / a * 2 + (distance - velocity_limit ** 2 / a) / velocity_limit
            time_count = 0
            # speed up
            while time_count * delta <= velocity_limit / a:
                SpeedTable[index].append(a*time_count*flag*delta)
                ThetaTable[index].append(ThetaTable[index][-1]+SpeedTable[index][-1]*delta)
                AccTable[index].append(a*flag)
                time_count += 1
            while time_count * delta <= (velocity_limit / a + (distance - velocity_limit ** 2 / (2*a)) / velocity_limit):
                SpeedTable[index].append(velocity_limit*flag)
                ThetaTable[index].append(ThetaTable[index][-1]+SpeedTable[index][-1]*delta)
                AccTable[index].append(0)
                time_count += 1
            while time_count * delta <= expected_time:
                SpeedTable[index].append((velocity_limit - a*(time_count*delta-(velocity_limit / a + (distance - velocity_limit ** 2 / (2*a)) / velocity_limit)))*flag)
                ThetaTable[index].append(ThetaTable[index][-1]+SpeedTable[index][-1]*delta)
                AccTable[index].append(-a*flag)
                time_count += 1
            # for i in range(int((velocity_limit / a)/delta)+1):
            #     SpeedTable[index].append(a*i*flag*delta)
            # for i in range(int(((distance - velocity_limit ** 2 / a) / velocity_limit)/delta)+1):
            #     SpeedTable[index].append(velocity_limit*flag*delta)
            # for i in range(int((velocity_limit / a)/delta)+1):
            #     SpeedTable[index].append((velocity_limit-a*i*delta)*flag)
        else:
            velocity_max = np.sqrt(a*distance)
            expected_time = velocity_max / a * 2
            time_count = 0
            while time_count * delta <= velocity_max / a:
                SpeedTable[index].append(a*time_count*flag*delta)
                ThetaTable[index].append(ThetaTable[index][-1]+SpeedTable[index][-1]*delta)
                AccTable[index].append(a*flag)
                time_count += 1
            # SpeedTable[index].append(velocity_max*flag)
            # ThetaTable[index].append(ThetaTable[index][-1]+SpeedTable[index][-1]*delta)
            # AccTable[index].append(a*flag)
            while time_count * delta <= velocity_max / a * 2:
                SpeedTable[index].append((velocity_max - a*(time_count*delta-velocity_max/a))*flag)
                ThetaTable[index].append(ThetaTable[index][-1]+SpeedTable[index][-1]*delta)
                AccTable[index].append(-a*flag)
                time_count += 1
        SpeedTable[index].append(0)
        ThetaTable[index].append(ThetaTable[index][-1])
        AccTable[index].append(0)
        return expected_time
    #控制周期不定，给定时间，返回当前时间的theta
    elif t != 0 and delta == 0:
        flag = 1
        if target_position - start_position < 0:
            flag = -1
        if distance >= velocity_limit ** 2 / a:
            expected_time = velocity_limit / a * 2 + (distance - velocity_limit ** 2 / a) / velocity_limit
            current_position = 0
            acc_time = velocity_limit/a
            dec_time = velocity_limit/a
            avg_time = (distance - velocity_limit ** 2 / a) / velocity_limit
            if t >= acc_time+dec_time+avg_time:
                current_position = target_position
            elif t < acc_time:
                moved_theta = 0.5*a*t**2
                moved_theta = moved_theta * flag
                current_position = start_position + moved_theta
            elif t >= acc_time and t < acc_time+avg_time:
                moved_theta = 0.5*a*acc_time**2 + velocity_limit*(t-acc_time)
                moved_theta = moved_theta * flag
                current_position = start_position + moved_theta
            elif t >= acc_time+avg_time and t < acc_time+avg_time+dec_time:
                moved_theta = 0.5*a*acc_time**2 + velocity_limit*avg_time + (2*velocity_limit-a(t-acc_time-avg_time))*(t-acc_time-avg_time)/2
                moved_theta = moved_theta * flag
                current_position = start_position + moved_theta

            return current_position, expected_time

        else:
            velocity_max = np.sqrt(a * distance)
            expected_time = velocity_max / a * 2
            acc_time = velocity_max/a
            dec_time = velocity_max/a
            current_position = 0
            if t >= acc_time + dec_time :
                current_position = target_position
            elif t < acc_time:
                moved_theta = 0.5 * a * t ** 2
                moved_theta = moved_theta * flag
                current_position = start_position + moved_theta
            elif t >= acc_time and t < acc_time + dec_time:
                moved_theta = 0.5 * a* acc_time ** 2 + (2 * velocity_max - a * (t - acc_time))*(t-acc_time)/2
                moved_theta = moved_theta * flag
                current_position = start_position + moved_theta

            return current_position,expected_time


def plot(pub_rate):
    for i in range(6):
        plt.figure(i)
        plt.title('Trajectory of demension' + str(i + 1))
        plt.xlabel('time /s')
        plt.ylabel('m m/s m/s^2')
        time_size = 0.02
        plt.plot([time_size * j for j in range(len(ThetaTable[i]))], ThetaTable[i], 'k-', label='target position')
        plt.plot([time_size * j for j in range(len(SpeedTable[i]))], SpeedTable[i], 'b-', label='target vel')
        plt.plot([time_size * j for j in range(len(AccTable[i]))], AccTable[i], 'r-', label='target acc')
        # plt.plot(RealEffort[i], label='real effort')
        plt.legend()
        plt.grid()
    plt.show()

def chrad(th):
    res = [0,0,0,0,0,0,0]
    res[0] = th[0]*30*180/math.pi
    res[1] = th[1]*205*180/(3*math.pi)
    res[2] = th[2]*50*180/math.pi
    res[3] = th[3]*125*180/(2*math.pi)
    res[4] = th[4]*125*180/(2*math.pi)
    res[5] = th[5]*200*180/(9*math.pi)
    res[6] = 800
    return res

def callback(data):
    global RealPosition
    global RealSpeed
    global RealEffort
    position = np.array(data.position).reshape(-1,1)
    velocity = np.array(data.velocity).reshape(-1,1)
    effort = np.array(data.effort).reshape(-1,1)
    print(position)
    print(velocity)
    RealPosition = np.hstack((RealPosition, position)) 
    RealSpeed = np.hstack((RealSpeed, velocity))
    RealEffort = np.hstack((RealEffort, effort))

if __name__ == "__main__":

    current_position = np.zeros(6)
    dt = 0.02
    a_limit = np.array([400, 400, 400, 400, 400, 400])
    print(a_limit)
    velocity_threshold = np.array([60, 60, 60, 60, 60, 60])
    v_initial = np.array([0, 0, 0, 0, 0, 0])
    target_position = []
    target_position.append(np.array([-0.73841174, 0.73410972, 1.65482929, -0.84614813, -0.03075535, 1.73195386]) / math.pi * 180)
    target_position.append(np.array([-0.18845569, 0.11414149, 1.54840006, -0.5902079, 0.04097188, -0.07763637]) / math.pi * 180)
    target_position.append(np.array([0.63568646, 0.29145575, 1.61017211, -0.2354593, -0.10481398, 1.51148932]) / math.pi * 180)
    target_position.append(np.array([-0.73841174, 0.63410972, 1.45482929, -0.84614813, -0.03075535, 1.73195386]) / math.pi * 180)
    print(target_position)

    total_time = 0
    #控制周期应用例子
    for i in range(len(target_position)-1):
        time, ThetaTable = tixing(target_position[i], target_position[i + 1], a_limit, velocity_threshold, dt, 0)
        total_time += time
    expected_time = 0.0
    #控制时间应用例子
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.01)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.02)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.03)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.04)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.05)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.06)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.07)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.08)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.09)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 0.1)
    print(current_position)
    current_position, expected_time = tixing(target_position[0], target_position[1], a_limit, velocity_threshold, 0, 1)
    print(current_position)
    print(expected_time)

    #print(total_time)
    #print(ThetaTable[5][0])
    plot(1/dt)


