from setuptools import find_packages, setup

package_name = 'ykc_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'sensor_msgs',      # sensor_msgsの依存関係
                        'geometry_msgs'],    # geometry_msgsの依存関係],
    zip_safe=True,
    maintainer='yukichi6105',
    maintainer_email='107849799+YUKICHI6105@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lider2d_localization = ykc_localization.lider2d_localization:main'
        ],
    },
)
