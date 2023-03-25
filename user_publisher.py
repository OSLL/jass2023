import roslibpy
import time
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                        prog='user_publisher',
                        description='What the program does',
                        epilog='Text at the bottom of help')

    parser.add_argument('-i', '--ip', help='bot IP address')
    parser.add_argument('-d', '--district', help='name of the district')
    parser.add_argument('-n', '--bot_name', help='bot name')
    parser.add_argument('-s', '--start', help='start cross num')
    parser.add_argument('-t', '--trajectory', help='bot trajectory')
    args = parser.parse_args()
    names = vars(args)
    print(names)
    if None in list(set([names[i] for i in names.keys()])):
        raise Exception('no params')

    client = roslibpy.Ros(host=args.ip, port=9001)
    client.run()
    bot = args.bot_name
    map_name = args.district  # 'zone1' 'zone2' 'zone3' 'full'


    axes= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    buttons[7] = 1

    map_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/map_state', 'std_msgs/String')
    begin_state_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/begin_state', 'std_msgs/Int32')
    trajectory_publisher = roslibpy.Topic(client, f'/{bot}/planning_node/trajectory', 'std_msgs/Int32MultiArray')
    start_pub = roslibpy.Topic(client, f'/{bot}/lane_controller_node/start', 'std_msgs/Bool')
    joy_pub = roslibpy.Topic(client, f'/{bot}/joy', 'sensor_msgs/Joy')

    map_publisher.publish(roslibpy.Message({'data': map_name}))
    begin_state_publisher.publish(roslibpy.Message({'data': args.start}))
    trajectory_publisher.publish(roslibpy.Message({'data': args.trajectory}))
    start_pub.publish(roslibpy.Message({'data': True}))
    joy_pub.publish(roslibpy.Message({'axes': axes, 'buttons':buttons }))
    time.sleep(1)

    client.terminate()