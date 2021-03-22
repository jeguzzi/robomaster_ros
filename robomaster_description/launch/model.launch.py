import os
from typing import List

import launch.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def urdf(name: str = '', model: str = 'ep') -> str:
    urdf_xacro = os.path.join(get_package_share_directory('robomaster_description'),
                              'urdf', f'robomaster_{model}.urdf.xacro')
    xacro_keys = [k for k, _ in urdf.__annotations__.items() if k not in ('return', 'model')]
    kwargs = dict(locals())
    xacro_args = [f'{arg_name}:={kwargs.get(arg_name)}' for arg_name in xacro_keys]
    opts, input_file_name = xacro.process_args([urdf_xacro] + xacro_args)
    try:
        doc = xacro.process_file(input_file_name, **vars(opts))
        return doc.toprettyxml(indent='  ')
    except Exception as e:
        print(e)
        return ''


def robot_state_publisher(context: LaunchContext,
                          **substitutions: launch.substitutions.LaunchConfiguration
                          ) -> List[Node]:
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}
    params = {'robot_description': urdf(**kwargs)}
    # with open('test.urdf', 'w+') as f:
    #     f.write(params['robot_description'])
    node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[params], output='screen')
    return [node]


def generate_launch_description() -> None:
    arguments = [
        launch.actions.DeclareLaunchArgument(
            k, default_value=str(urdf.__defaults__[i]), description='')
        for i, (k, _) in enumerate(urdf.__annotations__.items()) if k != 'return']
    kwargs = {k: launch.substitutions.LaunchConfiguration(k)
              for (k, _) in urdf.__annotations__.items() if k != 'return'}
    return LaunchDescription(
        arguments + [
            # launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('name')),
            # launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('publish_ground_truth')),
            launch.actions.OpaqueFunction(
                function=robot_state_publisher,
                kwargs=kwargs),
        ])
