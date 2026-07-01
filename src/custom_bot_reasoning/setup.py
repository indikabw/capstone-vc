from setuptools import setup
import os
from glob import glob

package_name = 'custom_bot_reasoning'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for root, dirs, files in os.walk('features'):
    target_dir = os.path.join('share', package_name, root)
    data_files.append((target_dir, [os.path.join(root, f) for f in files]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='indikabw',
    maintainer_email='indikabw@todo.todo',
    description='Reasoning node for custom bot',
    license='Apache License 2.0',
    tests_require=['pytest', 'behave'],
    entry_points={
        'console_scripts': [
            'reasoning_node = custom_bot_reasoning.reasoning_node:main'
        ],
    },
)
