from setuptools import find_packages, setup


package_name = 'aria_vision_zed'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='user',
    maintainer_email='user@todo.todo',

    description='ZED 2i simulation and 2D/3D vision fusion',

    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'zed_fusion_node = aria_vision_zed.zed_fusion_node:main',
            'zed_stub_publisher = aria_vision_zed.zed_stub_publisher:main',
        ],
    },
)