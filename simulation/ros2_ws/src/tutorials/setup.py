from setuptools import setup

package_name = 'tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['tutorials.simple_publisher', 'tutorials.simple_subscriber'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Textbook Maintainer',
    maintainer_email='textbook@example.com',
    description='Tutorials for AI-Native Textbook on Physical AI & Humanoid Robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = tutorials.simple_publisher:main',
            'simple_subscriber = tutorials.simple_subscriber:main',
            'simple_service_server = tutorials.simple_service_server:main',
            'simple_service_client = tutorials.simple_service_client:main',
            'simple_action_server = tutorials.simple_action_server:main',
            'simple_action_client = tutorials.simple_action_client:main',
        ],
    },
)