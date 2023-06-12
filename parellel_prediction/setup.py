from setuptools import setup

package_name = 'parellel_prediction'

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
    maintainer='oliver',
    maintainer_email='ge89yes@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parellel_path_generator = parellel_prediction.parellel_path_generator:main',
            'pp_test = parellel_prediction.pp_test:main',
            'tier4_utils = parellel_prediction.from_tier4_utils:main',
            'parellel_map_based_prediction_node = parellel_prediction.map_based_prediction_node:main',
            'visual_node = parellel_prediction.visualization:main',
            'map_loader = parellel_prediction.map_loader:main',
            'mgrs_projector = parellel_prediction.mgrs_projector:main',
        ],
    },
)
