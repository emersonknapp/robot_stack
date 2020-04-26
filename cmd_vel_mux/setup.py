from setuptools import setup

package_name = 'cmd_vel_mux'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emerson Knapp',
    maintainer_email='me@emersonknapp.com',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_mux = cmd_vel_mux.cmd_vel_mux:main'
        ],
    },
)
