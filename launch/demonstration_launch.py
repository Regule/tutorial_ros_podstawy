from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    description = LaunchDescription()

    publisher = Node(
        package='pubsub_demo',
        executable='small_publisher',
        output='screen',
        name='publisher_1',
        remappings=[
            ('/bragging', '/publisher_1/bragging')
        ],
        parameters=[{
            'count_start':10
        }]
    )

    subscriber = Node(
        package='pubsub_demo',
        executable='small_subscriber',
        output='screen',
        name='subscriber_1',
        remappings=[
            ('/bragging', '/publisher_1/bragging')
        ]
    )

    publisher2 = Node(
        package='pubsub_demo',
        executable='small_publisher',
        output='screen',
        name='publisher_2'
    )

    subscriber2 = Node(
        package='pubsub_demo',
        executable='small_subscriber',
        output='screen',
        name='subscriber_2'
    )




    description.add_action(publisher)
    description.add_action(subscriber)
    description.add_action(publisher2)
    description.add_action(subscriber2)
    return description

