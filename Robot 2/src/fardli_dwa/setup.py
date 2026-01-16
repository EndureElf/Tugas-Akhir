from setuptools import setup

package_name = 'fardli_dwa'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', []),
        ('share/' + package_name + '/config', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fardli',
    maintainer_email='you@example.com',
    description='Custom DWA local planner for mobile robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'simple_dwa = fardli_dwa.simple_dwa_node:main',
        'global_path = fardli_dwa.global_path:main',
        'follower = fardli_dwa.follower:main',
        'follower3= fardli_dwa.follower3state:main',
        'fl3= fardli_dwa.followerskenario3:main',
        'fl2= fardli_dwa.followerskenario2:main',
        'fl1= fardli_dwa.followerskenario1:main',
        'gp1= fardli_dwa.globalpathskenario1:main',
        'gp2= fardli_dwa.globalpathskenario2:main',
        'gp3= fardli_dwa.globalpathskenario3:main',
        ],
    },
)