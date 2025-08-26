from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def _make_actions(context, *args, **kwargs):
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    qx = LaunchConfiguration('qx').perform(context)
    qy = LaunchConfiguration('qy').perform(context)
    qz = LaunchConfiguration('qz').perform(context)
    qw = LaunchConfiguration('qw').perform(context)
    delay = float(LaunchConfiguration('delay').perform(context))

    pose_yaml = (
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}'
    )

    return [TimerAction(
        period=delay,
        actions=[ExecuteProcess(
            cmd=[
                'gz', 'service',
                '-s', '/gui/move_to/pose',
                '--reqtype', 'gz.msgs.GUICamera',
                '--reptype', 'gz.msgs.Boolean',
                '-r', pose_yaml,
                '--timeout', '10000'
            ],
            output='screen',
            shell=False
        )]
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='25.16'),
        DeclareLaunchArgument('y', default_value='8.08'),
        DeclareLaunchArgument('z', default_value='7.86'),
        DeclareLaunchArgument('qx', default_value='-0.3293072'),
        DeclareLaunchArgument('qy', default_value='0.2324760'),
        DeclareLaunchArgument('qz', default_value='0.7476280'),
        DeclareLaunchArgument('qw', default_value='0.5277916'),
        DeclareLaunchArgument('delay', default_value='2.0'),
        OpaqueFunction(function=_make_actions),
    ])
