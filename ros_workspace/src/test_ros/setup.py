from setuptools import find_packages, setup

package_name = 'test_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reach-lab',
    maintainer_email='stobin2@vols.utk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'matrix_test = test_ros.matrix_test:main',
            'removed_elements_test = test_ros.removed_elements_test:main',
            'fg_test = test_ros.fg_test:main',
            'focal_lesion_fg_test = test_ros.focal_lesion_fg_test:main',
            'local_compliance_mat_test = test_ros.local_compliance_mat_test:main',
            'stiffness_map_optimization = test_ros.stiffness_map_optimization:main'
        ],
    },
)
