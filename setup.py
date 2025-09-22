from setuptools import find_packages, setup

package_name = 'pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/pubsub.launch.xml',
          'launch/two_pubsub.launch.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='youremail@domain.edu',
    description='A pubsub demo',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pubsub = pubsub.pubsub:main'
        ],
    },
)
